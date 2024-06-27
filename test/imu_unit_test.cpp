#include <cmath>
#include <pigpiod_if2.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

int pi;     // raspberry pi
const double PI = 3.14159265358979323846;   // 円周率
const double rad2deg = 180.0 / PI;  // ラジアンを度に変換する定数


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

//Update rate
float theta_update_freq = 400.0f; //Hz
float theta_update_interval = 1.0f/theta_update_freq;
int th1_dura = 1000000 * 1.0f / theta_update_freq;  // 2500usec for theta_update_freq=400
//State vector
//[[theta(degree)], [offset of theta_dot(degree/sec)]]
float theta_data_predict[2][1];
float theta_data[2][1];
//Covariance matrix
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



int init_udp_socket(int& sockfd, struct sockaddr_in& servaddr, const char* ip, int port) {
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

void send_udp_packet(int sockfd, struct sockaddr_in& servaddr, const char* data) {
    sendto(sockfd, data, strlen(data), 0, (struct sockaddr *)&servaddr, sizeof(servaddr));
}



void console_write(float elapsed_time, float theta_p, float theta_p_dot, float theta_p_kf, float theta_p_dot_kf) {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << std::setw(10) << elapsed_time << ","
              << std::setw(10) << theta_p*rad2deg << ","
              << std::setw(10) << theta_p_dot*rad2deg << ","
              << std::setw(10) << theta_p_kf*rad2deg << ","
              << std::setw(10) << theta_p_dot_kf*rad2deg
              << std::endl;
}

float get_acc_data(int pi, int bus)
{
    unsigned char data[4];
    i2c_read_i2c_block_data(pi, bus, 0x04, (char *)data, 4);

    int y_data = ((data[0] & 0xF0) + (data[1] * 256)) / 16;
    if (y_data > 2047)
    {
        y_data -= 4096;
    }

    int z_data = ((data[2] & 0xF0) + (data[3] * 256)) / 16;
    if (z_data > 2047)
    {
        z_data -= 4096;
    }

    float theta1_deg = atan2(float(z_data), float(y_data)) * 57.29578f;
    return theta1_deg;
}


// statistical data of accelerometer
// By passing references to theta_mean and theta_variance, using "&",
// you can modify the values at their referenced locations.
void acc_init(int pi, int bus, int sample_num, float meas_interval, float &theta_mean, float &theta_variance)
{
    // initialize ACC register 0x0F (range)
    // Full scale = +/- 2 G
    i2c_write_byte_data(pi, bus, 0x0F, 0x03);
    // initialize ACC register 0x10 (band width)
    // Filter bandwidth = 1000 Hz
    i2c_write_byte_data(pi, bus, 0x10, 0x0F);

    // get data
    float theta_array[sample_num];
    for (int i = 0; i < sample_num; i++)
    {
        theta_array[i] = get_acc_data(pi, bus);
        usleep(meas_interval);
    }

    // calculate mean
    theta_mean = 0;
    for (int i = 0; i < sample_num; i++)
    {
        theta_mean += theta_array[i];
    }
    theta_mean /= sample_num;

    // calculate variance
    float temp;
    theta_variance = 0;
    for (int i = 0; i < sample_num; i++)
    {
        temp = theta_array[i] - theta_mean;
        theta_variance += temp * temp;
    }
    theta_variance /= sample_num;
    return;
}

//=========================================================
// Gyroscope (BMX055)
//=========================================================
// get data
float get_gyr_data(int pi, int bus)
{
    unsigned char data[2];
    i2c_read_i2c_block_data(pi, bus, 0x02, (char *)data, 2);

    int theta1_dot = data[0] + 256 * data[1];
    if (theta1_dot > 32767)
    {
        theta1_dot -= 65536;
    }
    theta1_dot = -1 * theta1_dot; // !caution!
    // +1000 (deg/sec) / 2^15 = 0.0305176
    return float(theta1_dot) * 0.0305176f;
}

// statistical data of gyro
void gyr_init(int pi, int bus, int sample_num, float meas_interval, float &theta_dot_mean, float &theta_dot_variance)
{
    // initialize Gyro register 0x0F (range)
    // Full scale = +/- 1000 deg/s
    i2c_write_byte_data(pi, bus, 0x0F, 0x01);
    // initialize Gyro register 0x10 (band width)
    // Data rate = 1000 Hz, Filter bandwidth = 116 Hz
    i2c_write_byte_data(pi, bus, 0x10, 0x02);

    // get data
    float theta_dot_array[sample_num];
    for (int i = 0; i < sample_num; i++)
    {
        theta_dot_array[i] = get_gyr_data(pi, bus);
        usleep(meas_interval);
    }

    // calculate mean
    theta_dot_mean = 0;
    for (int i = 0; i < sample_num; i++)
    {
        theta_dot_mean += theta_dot_array[i];
    }
    theta_dot_mean /= sample_num;

    // calculate variance
    float temp;
    theta_dot_variance = 0;
    for (int i = 0; i < sample_num; i++)
    {
        temp = theta_dot_array[i] - theta_dot_mean;
        theta_dot_variance += temp * temp;
    }
    theta_dot_variance /= sample_num;
    return;
}


void setup()
{
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

    acc_init(pi, bus_acc, sample_num, meas_interval, theta_mean, theta_variance);
    gyr_init(pi, bus_gyr, sample_num, meas_interval, theta_dot_mean, theta_dot_variance);

    gpio_write(pi, LED_R, 1);
    gpio_write(pi, LED_G, 1);

    gpio_write(pi, LED_R, 0);
    gpio_write(pi, LED_G, 0);
}

int main()
{
    setup();

    gpio_write(pi, LED_Y, 0);

    auto start = std::chrono::system_clock::now();

    while (true)
    {
        auto loop_start = std::chrono::system_clock::now();

        gpio_write(pi, LED_R, 0);
        gpio_write(pi, LED_G, 0);

        // スタートからの経過時間の取得
        auto elapsed = std::chrono::system_clock::now() - start;
        float elapsed_time = std::chrono::duration<float>(elapsed).count();

        // // データの取得（仮の値を使用）
        // // 測定値
        // float theta_p = theta;        // rad
        // float theta_p_dot = theta_dot_gyro;    // rad/s

        // // KFの推定値
        // float theta_p_kf = theta_data[0][0];
        // float theta_p_dot_kf = theta_data[1][0];

        // display表示
        console_write(elapsed_time, theta_p, theta_p_dot, theta_p_kf, theta_p_dot_kf);

        // 処理時間を含めたスリープ時間の計算
        auto loop_end = std::chrono::system_clock::now();
        auto loop_duration = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - loop_start).count();
        auto sleep_time = feedback_dura - loop_duration;
        if (sleep_time > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
        }
    }
    return 0;
}
