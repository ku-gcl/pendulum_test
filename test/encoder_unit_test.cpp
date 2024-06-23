#include "pigpiod_if2.h"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

int pi;

int ENC_PIN1 = 24;
int ENC_PIN2 = 23;
int enc_syn = 1;
int encoder_update_rate = 25;    // usec
int encoder_resolution = 100;
int encoder_value = 0;
int table[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};
int code;

float pre_theta_w = 0.0f;

int main_loop_rate_ms = 10;    // msec
double main_loop_rate = main_loop_rate_ms * 1e-3;

std::thread thread_encoder;

void rotary_encoder() {
    while(1) {
        if (enc_syn == 1) {
            // std::cout << "code: " << code << std::endl;
            // check the movement
            code = ((code << 2) + (gpio_read(pi, ENC_PIN2) << 1) + gpio_read(pi, ENC_PIN1)) & 0xf; // !caution!
            // update the encoder value
            int value = -1 * table[code]; // Fix the direction if necessary
            encoder_value += value;
            // std::cout << "Updated Encoder Value: " << encoder_value << ", Code: " << code << ", Value: " << value << std::endl; // Debug output
            // std::this_thread::sleep_for(std::chrono::microseconds(encoder_update_rate));
        }
    }
}

void console_write(float elapsed_time, float theta_w, float theta_w_dot) {
    std::cout << std::fixed << std::setprecision(3); // 固定小数点表記と精度の設定
    // 列の幅を設定して表示
    std::cout << std::setw(10) << elapsed_time << ","
              << std::setw(10) << theta_w << ","
              << std::setw(10) << theta_w_dot << ","
              << std::endl;
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
    code = ((gpio_read(pi, ENC_PIN2) << 1) + gpio_read(pi, ENC_PIN1)) & 0xf;

    thread_encoder = std::thread(rotary_encoder);
    auto start = std::chrono::system_clock::now();

    while (1) {
        auto main_loop_start = std::chrono::system_clock::now();

        // デバッグ用にencoder_valueを出力する
        std::cout << "Encoder Value: " << encoder_value << std::endl;

        float theta_w = encoder_value * (2 * 3.14f) / (4 * encoder_resolution);
        float theta_w_dot = (theta_w - pre_theta_w) / main_loop_rate;
        pre_theta_w = theta_w;

        auto elapsed = std::chrono::system_clock::now() - start;
        float elapsed_time = std::chrono::duration<float>(elapsed).count();
        console_write(elapsed_time, theta_w, theta_w_dot);

        auto main_loop_end = std::chrono::system_clock::now();
        auto main_loop_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_loop_end - main_loop_start).count();
        int sleep_time_ms = main_loop_rate_ms - main_loop_duration_ms;
        if (sleep_time_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
        }
    }

    return 0;
}
