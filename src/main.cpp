#include <chrono>
#include <iomanip> // std::setprecision
#include <iostream>
#include <pigpiod_if2.h>
#include <sstream> // std::ostringstream
#include <thread>
#include <unistd.h>

#include "config.h"
#include "csv_writer.h" // CSV 書き込み機能をインクルード
#include "encoder.h"
#include "kalman_filter.h"
#include "motor_control.h"
#include "sensor.h"

// udp
#include <arpa/inet.h>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

std::thread thread1;
std::thread thread2;

//  TODO: move to config.cpp **********************************
// udp
int SOCKET_PORT = 12345;
const char *SOCKET_IP = "192.168.1.200";

// sensor
float theta;
float theta_dot_gyro;
// float xAccl = 0.0f, yAccl = 0.0f, zAccl = 0.0f;
// float xGyro = 0.0f, yGyro = 0.0f, zGyro = 0.0f;

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
// ************************************************************

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

    bmx055_init(pi, bus_acc, bus_gyr);

    acc_init(pi, bus_acc, sample_num, meas_interval, theta_mean,
             theta_variance);
    gyr_init(pi, bus_gyr, sample_num, meas_interval, theta_dot_mean,
             theta_dot_variance);

    encoder_value = 0;
    motor_driver_init(pi);
    kalman_filter_init();

    gpio_write(pi, LED_R, 1);
    gpio_write(pi, LED_G, 1);

    // CSV ファイルのオープン
    createDirectoryIfNotExists(LOG_DATA_DIR);
    // std::string filename =
    //     LOG_DATA_DIR + "log_" + getCurrentDateTime() + ".csv";
    // ファイル名を構築
    std::ostringstream filenameStream;
    filenameStream << LOG_DATA_DIR << "log_" << getCurrentDateTime() << "_Gain_"
                   << std::fixed << std::setprecision(1) << Gain[0] << "_"
                   << Gain[1] << "_" << Gain[2] << "_" << Gain[3] << "_MaxV"
                   << MAX_VOLTAGE << ".csv";
    std::string filename = filenameStream.str();
    openCSVFile(filename);

    gpio_write(pi, LED_R, 0);
    gpio_write(pi, LED_G, 0);
}

int main() {
    setup();

    thread1 = std::thread(rotary_encoder);
    thread2 = std::thread(update_theta, bus_acc, bus_gyr);

    // エンコーダの初期値を設定
    code = int((gpio_read(pi, pin2) << 1) + gpio_read(pi, pin1)) & 0xf;

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
        update_theta_syn_flag = 0;
        gpio_write(pi, LED_R, 0);
        gpio_write(pi, LED_G, 0);

        kalman_filter_update();

        // for safety
        // エンコーダの回転数が2回転以上(=2pi*2)回転したらモーター出力停止
        // bool motor_update = (abs(y[2][0]) < 13);
        bool motor_update = 1;
        motor_control_update(motor_update);

        //=========================================================
        // Log data
        //=========================================================
        // 現在時刻の取得
        double time = getCurrentEpochTimeUTC();

        // スタートからの経過時間の取得
        auto elapsed = std::chrono::system_clock::now() - start;
        float elapsed_time = std::chrono::duration<float>(elapsed).count();

        // データの取得（仮の値を使用）
        // 測定値
        float theta_p = y[0][0];     // rad
        float theta_p_dot = y[1][0]; // rad/s
        float theta_w = y[2][0];     // rad
        float theta_w_dot = y[3][0]; // rad/s

        // KFの推定値
        float theta_p_kf = x_data[0][0];
        float theta_p_dot_kf = x_data[1][0];
        float theta_w_kf = x_data[2][0];
        float theta_w_dot_kf = x_data[3][0];

        // 制御入力
        float log_motor_value = motor_value;
        float log_motor_direction = motor_direction;
        float log_pwm_duty = pwm_duty;

        // display表示
        // console_write(elapsed_time, theta_p, theta_p_dot, theta_w,
        // theta_w_dot,
        //               theta_p_kf, theta_p_dot_kf, theta_w_kf, theta_w_dot_kf,
        //               log_motor_value, log_motor_direction, log_pwm_duty);

        // UDPパケットの送信
        char buffer[1024];
        snprintf(buffer, sizeof(buffer),
                 "data;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f", elapsed_time,
                 theta_p, theta_p_dot, theta_w, theta_w_dot, theta_p_kf,
                 theta_p_dot_kf, theta_w_kf, theta_w_dot_kf, log_motor_value,
                 log_motor_direction, log_pwm_duty);
        send_udp_packet(sockfd, servaddr, buffer);

        // CSV書き込み
        csv_write(time, elapsed_time, theta_p, theta_p_dot, theta_w,
                  theta_w_dot, theta_p_kf, theta_p_dot_kf, theta_w_kf,
                  theta_w_dot_kf, log_motor_value, log_motor_direction,
                  log_pwm_duty);

        pre_theta2 = y[2][0];
        update_theta_syn_flag = 1;

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

    closeCSVFile();
    thread1.join();
    thread2.join();
    return 0;
}
