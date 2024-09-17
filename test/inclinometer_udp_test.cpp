#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <pigpiod_if2.h>
#include <thread>
#include <unistd.h>

#include "../src/matrix_operations.h"

// udp
#include <arpa/inet.h>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

// udp
int SOCKET_PORT = 12345;
// const char *SOCKET_IP = "192.168.1.199";
const char *SOCKET_IP = "192.168.11.2";

// thread
std::thread thread2;

int pi;                                   // raspberry pi
const double PI = 3.14159265358979323846; // 円周率
const double rad2deg = 180.0 / PI; // ラジアンを度に変換する定数
const double deg2rad = PI / 180.0;

// sensor
int bus_acc;
int bus_gyr;
const int ACC_ADDR = 0x19;
const int GYR_ADDR = 0x69;
const int pin1 = 24;
const int pin2 = 23;
const int IN1 = 6;
const int IN2 = 5;
const int PWM = 12;
const int LED_R = 17;
const int LED_Y = 27;
const int LED_G = 22;

int sample_num = 100;
// float meas_interval = 0.01;
float meas_interval = 10000.0f; // us micro seconds
float theta_mean;
float theta_variance;
float theta_dot_mean;
float theta_dot_variance;

// Update rate
float theta_update_freq = 400.0f; // Hz
float theta_update_interval = 1.0f / theta_update_freq;
int th1_dura =
    1000000 * 1.0f / theta_update_freq; // 2500usec for theta_update_freq=400
// State vector
//[[theta(degree)], [offset of theta_dot(degree/sec)]]
float theta_data_predict[2][1];
float theta_data[2][1];
// Covariance matrix
float P_theta_predict[2][2];
float P_theta[2][2];
//"A" of the state equation
float A_theta[2][2] = {{1, -theta_update_interval}, {0, 1}};
//"B" of the state equation
float B_theta[2][1] = {{theta_update_interval}, {0}};
//"C" of the state equation
float C_theta[1][2] = {{1, 0}};

int feedback_dura = 10; // msec

int enc_syn = 1;
int update_theta_syn_flag = 1;

// sensor data
float theta;
float theta_dot_gyro;

float xAccl = 0.0f, yAccl = 0.0f, zAccl = 0.0f;
float xGyro = 0.0f, yGyro = 0.0f, zGyro = 0.0f;

int init_udp_socket(int &sockfd, struct sockaddr_in &servaddr, const char *ip,
                    int port) {
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "Error creating socket" << std::endl;
        return -1;
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);
    servaddr.sin_addr.s_addr = inet_addr(ip);

    return 0;
}

void send_udp_packet(int sockfd, struct sockaddr_in &servaddr,
                     const char *data) {
    sendto(sockfd, data, strlen(data), 0, (struct sockaddr *)&servaddr,
           sizeof(servaddr));
}

void console_write(float elapsed_time, float theta_p, float theta_p_dot,
                   float theta_p_kf, float theta_p_dot_kf) {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << std::setw(10) << elapsed_time << "," << std::setw(10)
              << theta_p * rad2deg << "," << std::setw(10)
              << theta_p_dot * rad2deg << "," << std::setw(10)
              << theta_p_kf * rad2deg << "," << std::setw(10)
              << theta_p_dot_kf * rad2deg << std::endl;
}

void bmx055_init() {
    // Accelerometer initialization
    // range: register 0x0F, Full scale = +/- 2 G
    i2c_write_byte_data(pi, bus_acc, 0x0F, 0x03);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // band width: 0x10, Filter bandwidth = 1000 Hz
    i2c_write_byte_data(pi, bus_acc, 0x10, 0x0F);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // sleep duration: 0x11, 0.5 ms
    i2c_write_byte_data(pi, bus_acc, 0x11, 0x00);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Gyroscope initialization
    // range: register 0x0F, Full scale = +/- 1000 deg/s
    i2c_write_byte_data(pi, bus_gyr, 0x0F, 0x01);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // band width: 0x10, Filter bandwidth = 116 Hz
    i2c_write_byte_data(pi, bus_gyr, 0x10, 0x02);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // sleep duration: 0x11, 0.5 ms
    i2c_write_byte_data(pi, bus_gyr, 0x11, 0x00);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void get_acc_data(int pi, int bus) {
    char data[6];
    i2c_read_i2c_block_data(pi, bus_acc, 0x02, data, 6);

    int x = ((data[1] << 8) | (data[0] & 0xF0)) >> 4;
    if (x > 2047)
        x -= 4096;
    int y = ((data[3] << 8) | (data[2] & 0xF0)) >> 4;
    if (y > 2047)
        y -= 4096;
    int z = ((data[5] << 8) | (data[4] & 0xF0)) >> 4;
    if (z > 2047)
        z -= 4096;

    xAccl = x * 0.00098f; // range = +/-2g
    yAccl = y * 0.00098f; // range = +/-2g
    zAccl = z * 0.00098f; // range = +/-2g
}

void get_gyr_data(int pi, int bus) {
    char data[6];
    i2c_read_i2c_block_data(pi, bus_gyr, 0x02, data, 6);

    int x = (data[1] << 8) | data[0];
    if (x > 32767)
        x -= 65536;
    int y = (data[3] << 8) | data[2];
    if (y > 32767)
        y -= 65536;
    int z = (data[5] << 8) | data[4];
    if (z > 32767)
        z -= 65536;

    // +1000 (deg/sec) / 2^15 = 0.0305176
    xGyro = x * 0.0305176f; //  Full scale = +/- 1000 degree/s
    yGyro = y * 0.0305176f; //  Full scale = +/- 1000 degree/s
    zGyro = z * 0.0305176f; //  Full scale = +/- 1000 degree/s
}

float get_theta_p_deg(int pi, int bus) {
    get_acc_data(pi, bus);
    float theta_p_deg = atan2(float(zAccl), float(yAccl)) * rad2deg;
    return theta_p_deg;
}

float get_theta_p_dot_deg(int pi, int bus) {
    get_gyr_data(pi, bus);
    float theta_p_dot_deg = -1 * xGyro; // !caution!
    return theta_p_dot_deg;
}

//=========================================================
// Initialize
//=========================================================
void acc_init(int pi, int bus, int sample_num, float meas_interval,
              float &theta_mean, float &theta_variance) {
    // get data
    float theta_array[sample_num];
    for (int i = 0; i < sample_num; i++) {
        theta_array[i] = get_theta_p_deg(pi, bus);
        usleep(meas_interval);
    }

    // calculate mean
    theta_mean = 0;
    for (int i = 0; i < sample_num; i++) {
        theta_mean += theta_array[i];
    }
    theta_mean /= sample_num;

    // calculate variance
    float temp;
    theta_variance = 0;
    for (int i = 0; i < sample_num; i++) {
        temp = theta_array[i] - theta_mean;
        theta_variance += temp * temp;
    }
    theta_variance /= sample_num;

    std::cout << "theta_mean= " << theta_mean << std::endl;
    std::cout << "theta_variance= " << theta_variance << std::endl;
    std::cout << "--------------------------------------" << std::endl;

    return;
}

// statistical data of gyro
void gyr_init(int pi, int bus, int sample_num, float meas_interval,
              float &theta_dot_mean, float &theta_dot_variance) {
    // get data
    float theta_dot_array[sample_num];
    for (int i = 0; i < sample_num; i++) {
        theta_dot_array[i] = get_theta_p_dot_deg(pi, bus);
        usleep(meas_interval);
    }

    // calculate mean
    theta_dot_mean = 0;
    for (int i = 0; i < sample_num; i++) {
        theta_dot_mean += theta_dot_array[i];
    }
    theta_dot_mean /= sample_num;

    // calculate variance
    float temp;
    theta_dot_variance = 0;
    for (int i = 0; i < sample_num; i++) {
        temp = theta_dot_array[i] - theta_dot_mean;
        theta_dot_variance += temp * temp;
    }
    theta_dot_variance /= sample_num;

    std::cout << "theta_dot_mean= " << theta_dot_mean << std::endl;
    std::cout << "theta_dot_variance= " << theta_dot_variance << std::endl;
    std::cout << "--------------------------------------" << std::endl;

    return;
}

void update_theta(int bus_acc, int bus_gyr) {
    // if (update_theta_syn_flag == 0) {
    //     return;
    // }
    while (true) {
        if (update_theta_syn_flag == 1) {
            enc_syn = 0;

            // 姿勢角のセンサ値
            theta = get_theta_p_deg(pi, bus_acc); // deg
            // 姿勢角速度のセンサ値
            theta_dot_gyro = get_theta_p_dot_deg(pi, bus_gyr); // deg/s

            // calculate Kalman gain: G = P'C^T(W+CP'C^T)^-1
            float P_CT[2][1] = {};
            float tran_C_theta[2][1] = {};
            mat_tran(C_theta[0], tran_C_theta[0], 1, 2);
            mat_mul(P_theta_predict[0], tran_C_theta[0], P_CT[0], 2, 2, 2, 1);
            float G_temp1[1][1] = {};
            mat_mul(C_theta[0], P_CT[0], G_temp1[0], 1, 2, 2, 1);
            float G_temp2 = 1.0f / (G_temp1[0][0] + theta_variance);
            float G[2][1] = {};
            mat_mul_const(P_CT[0], G_temp2, G[0], 2, 1);

            // theta_data estimation: theta = theta'+G(y-Ctheta')
            float C_theta_theta[1][1] = {};
            mat_mul(C_theta[0], theta_data_predict[0], C_theta_theta[0], 1, 2,
                    2, 1);
            float delta_y = theta - C_theta_theta[0][0];
            float delta_theta[2][1] = {};
            mat_mul_const(G[0], delta_y, delta_theta[0], 2, 1);
            mat_add(theta_data_predict[0], delta_theta[0], theta_data[0], 2, 1);

            // calculate covariance matrix: P=(I-GC)P'
            float GC[2][2] = {};
            float I2[2][2] = {{1, 0}, {0, 1}};
            mat_mul(G[0], C_theta[0], GC[0], 2, 1, 1, 2);
            float I2_GC[2][2] = {};
            mat_sub(I2[0], GC[0], I2_GC[0], 2, 2);
            mat_mul(I2_GC[0], P_theta_predict[0], P_theta[0], 2, 2, 2, 2);

            // predict the next step data: theta'=Atheta+Bu
            float A_theta_theta[2][1] = {};
            float B_theta_dot[2][1] = {};
            mat_mul(A_theta[0], theta_data[0], A_theta_theta[0], 2, 2, 2, 1);
            mat_mul_const(B_theta[0], theta_dot_gyro, B_theta_dot[0], 2, 1);
            mat_add(A_theta_theta[0], B_theta_dot[0], theta_data_predict[0], 2,
                    1);

            // predict covariance matrix: P'=APA^T + BUB^T
            float AP[2][2] = {};
            float APAT[2][2] = {};
            float tran_A_theta[2][2] = {};
            mat_tran(A_theta[0], tran_A_theta[0], 2, 2);
            mat_mul(A_theta[0], P_theta[0], AP[0], 2, 2, 2, 2);
            mat_mul(AP[0], tran_A_theta[0], APAT[0], 2, 2, 2, 2);
            float BBT[2][2];
            float tran_B_theta[1][2] = {};
            mat_tran(B_theta[0], tran_B_theta[0], 2, 1);
            mat_mul(B_theta[0], tran_B_theta[0], BBT[0], 2, 1, 1, 2);
            float BUBT[2][2] = {};
            mat_mul_const(BBT[0], theta_dot_variance, BUBT[0], 2, 2);
            mat_add(APAT[0], BUBT[0], P_theta_predict[0], 2, 2);

            enc_syn = 1;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(th1_dura));
    }
}

void setup() {
    pi = pigpio_start(NULL, NULL);
    bus_acc = i2c_open(pi, 1, ACC_ADDR, 0);
    bus_gyr = i2c_open(pi, 1, GYR_ADDR, 0);

    set_mode(pi, LED_R, PI_OUTPUT);
    set_mode(pi, LED_Y, PI_OUTPUT);
    set_mode(pi, LED_G, PI_OUTPUT);
    set_mode(pi, pin1, PI_INPUT);
    set_mode(pi, pin2, PI_INPUT);
    set_mode(pi, IN1, PI_OUTPUT);
    set_mode(pi, IN2, PI_OUTPUT);
    set_mode(pi, PWM, PI_OUTPUT);

    gpio_write(pi, LED_R, 1);
    gpio_write(pi, LED_Y, 1);
    gpio_write(pi, LED_G, 1);
    sleep(1);
    gpio_write(pi, LED_R, 0);
    gpio_write(pi, LED_G, 0);

    bmx055_init();

    acc_init(pi, bus_acc, sample_num, meas_interval, theta_mean,
             theta_variance);
    gyr_init(pi, bus_gyr, sample_num, meas_interval, theta_dot_mean,
             theta_dot_variance);

    // init kalman filter
    theta_data_predict[0][0] = 0;
    theta_data_predict[1][0] = theta_dot_mean;

    P_theta_predict[0][0] = 1;
    P_theta_predict[0][1] = 0;
    P_theta_predict[1][0] = 0;
    P_theta_predict[1][1] = theta_dot_variance;

    gpio_write(pi, LED_R, 1);
    gpio_write(pi, LED_G, 1);

    gpio_write(pi, LED_R, 0);
    gpio_write(pi, LED_G, 0);
}

int main() {
    setup();

    thread2 = std::thread(update_theta, bus_acc, bus_gyr);

    gpio_write(pi, LED_Y, 0);

    // udp settings
    int sockfd;
    struct sockaddr_in servaddr;
    if (init_udp_socket(sockfd, servaddr, SOCKET_IP, SOCKET_PORT) < 0) {
        return 1;
    }

    auto start = std::chrono::system_clock::now();

    while (true) {
        auto loop_start = std::chrono::system_clock::now();

        // main
        // loop中はtheta（振子の姿勢角）の更新（kalman_filter.cpp/update_theta）を停止
        // update_theta_syn_flag = 0;
        gpio_write(pi, LED_R, 0);
        gpio_write(pi, LED_G, 0);

        //=========================================================
        // Log data
        //=========================================================

        // スタートからの経過時間の取得
        auto elapsed = std::chrono::system_clock::now() - start;
        float elapsed_time = std::chrono::duration<float>(elapsed).count();

        // データの取得（仮の値を使用）
        // 測定値
        float theta_p = get_theta_p_deg(pi, bus_acc) * deg2rad;         // rad
        float theta_p_dot = get_theta_p_dot_deg(pi, bus_gyr) * deg2rad; // rad/s

        // KFの推定値
        float theta_p_kf = theta_data[0][0] * deg2rad;
        // float theta_p_dot_kf = theta_data[1][0] * deg2rad;
        float theta_p_dot_kf =
            theta_p_dot - (theta_data[1][0] * deg2rad); // w - w_offset

        // display表示
        console_write(elapsed_time, theta_p, theta_p_dot, theta_p_kf,
                      theta_p_dot_kf);

        // UDPパケットの送信
        char buffer[1024];
        snprintf(buffer, sizeof(buffer), "data;%f;%f;%f;%f;%f", elapsed_time,
                 theta_p, theta_p_dot, theta_p_kf, theta_p_dot_kf);
        send_udp_packet(sockfd, servaddr, buffer);

        // update_theta_syn_flag = 1;

        // 処理時間を含めたスリープ時間の計算
        auto loop_end = std::chrono::system_clock::now();
        auto loop_duration =
            std::chrono::duration_cast<std::chrono::milliseconds>(loop_end -
                                                                  loop_start)
                .count();
        auto sleep_time = feedback_dura - loop_duration;
        if (sleep_time > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
        }
    }

    thread2.join();
    close(sockfd);
    return 0;
}
