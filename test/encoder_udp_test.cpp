#include "pigpiod_if2.h"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
// udp
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#include <unistd.h>

int pi;     // raspberry pi
const double PI = 3.14159265358979323846;   // 円周率
const double rad2deg = 180.0 / PI;  // ラジアンを度に変換する定数

int ENC_PIN1 = 24;      // A
int ENC_PIN2 = 23;      // B
int enc_syn = 1;
int encoder_update_rate = 25;    // usec
int encoder_resolution = 100;
int encoder_value = 0;
int table[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};
int code;

float pre_theta_w = 0.0f;

int main_loop_rate_ms = 10;    // msec
double main_loop_rate = main_loop_rate_ms * 1e-3;

int SOCKET_PORT = 12345;
// const char* SOCKET_IP = "192.168.11.2";
const char* SOCKET_IP = "192.168.1.199";

std::thread thread_encoder;

void rotary_encoder() {
    while (1) {
        if (enc_syn == 1) {
            auto enc_loop_start = std::chrono::steady_clock::now();
            code = ((code << 2) + (gpio_read(pi, ENC_PIN2) << 1) + gpio_read(pi, ENC_PIN1)) & 0xf;

            int value = -1 * table[code];
            encoder_value += value;

            auto enc_loop_end = std::chrono::steady_clock::now();
            auto enc_loop_duration_us = std::chrono::duration_cast<std::chrono::microseconds>(enc_loop_end - enc_loop_start).count();
            int sleep_time_ms = encoder_update_rate - enc_loop_duration_us;
            if (sleep_time_ms > 0) {
                std::this_thread::sleep_for(std::chrono::microseconds(sleep_time_ms));
            }
        }
    }
}

void console_write(float elapsed_time, float theta_w, float theta_w_dot) {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << std::setw(10) << elapsed_time << ","
              << std::setw(10) << theta_w*rad2deg << ","
              << std::setw(10) << theta_w_dot*rad2deg << std::endl;
}

int main() {
    pi = pigpio_start(NULL, NULL);
    set_mode(pi, ENC_PIN1, PI_INPUT);
    set_mode(pi, ENC_PIN2, PI_INPUT);

    // デバッグ用にエンコーダのピンの初期状態を確認する
    int initial_pin1 = gpio_read(pi, ENC_PIN1);
    int initial_pin2 = gpio_read(pi, ENC_PIN2);
    std::cout << "Initial Pin States: ENC_PIN1 = " << initial_pin1 << ", ENC_PIN2 = " << initial_pin2 << std::endl;

    // エンコーダの初期値を設定
    code = int((gpio_read(pi, ENC_PIN2) << 1) + gpio_read(pi, ENC_PIN1)) & 0xf;

    // udp settings
    int sockfd;
    struct sockaddr_in servaddr;
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "Error creating socket" << std::endl;
        return 1;
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(SOCKET_PORT);
    servaddr.sin_addr.s_addr = inet_addr(SOCKET_IP);

    // threadを定義
    thread_encoder = std::thread(rotary_encoder);
    auto start = std::chrono::system_clock::now();

    while (1) {
        auto main_loop_start = std::chrono::system_clock::now();

        float theta_w = encoder_value * (2 * PI) / (4 * encoder_resolution);
        float theta_w_dot = (theta_w - pre_theta_w) / main_loop_rate;
        pre_theta_w = theta_w;

        auto elapsed = std::chrono::system_clock::now() - start;
        float elapsed_time = std::chrono::duration<float>(elapsed).count();
        console_write(elapsed_time, theta_w, theta_w_dot);

        // UDPパケットの送信
        char buffer[1024];
        snprintf(buffer, sizeof(buffer), "data;%f;%f;%f", elapsed_time, theta_w, theta_w_dot);
        sendto(sockfd, buffer, strlen(buffer), 0, (struct sockaddr *)&servaddr, sizeof(servaddr));

        auto main_loop_end = std::chrono::system_clock::now();
        auto main_loop_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_loop_end - main_loop_start).count();
        int sleep_time_ms = main_loop_rate_ms - main_loop_duration_ms;
        if (sleep_time_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
        }
    }

    // socketのclose
    close(sockfd);

    return 0;
}
