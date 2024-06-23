#include "pigpiod_if2.h"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <vector>

int pi;     // raspberry pi
const double PI = 3.14159265358979323846;   // 円周率
const double rad2deg = 180.0 / PI;  // ラジアンを度に変換する定数

int ENC_PIN1 = 24;
int ENC_PIN2 = 23;
int enc_syn = 1;
int encoder_update_rate = 25;    // usec
int encoder_resolution = 100;
int encoder_value = 0;
int table[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};
int code;

float pre_theta_w = 0.0f;

std::thread thread_encoder;

void rotary_encoder() {
    auto start_time = std::chrono::steady_clock::now();
    std::vector<double> execution_times;
    
    while (1) {
        if (enc_syn == 1) {
            auto loop_start_time = std::chrono::steady_clock::now();

            // check the movement
            code = ((code << 2) + (gpio_read(pi, ENC_PIN2) << 1) + gpio_read(pi, ENC_PIN1)) & 0xf;
            // update the encoder value
            int value = -1 * table[code];
            encoder_value += value;
            // std::cout << "Value: " << value << std::endl;

            // auto loop_end_time = std::chrono::steady_clock::now();
            // std::chrono::duration<double> loop_duration = loop_end_time - loop_start_time;
            // std::cout << loop_duration.count() << std::endl;
            // execution_times.push_back(loop_duration.count());

            // if (std::chrono::duration_cast<std::chrono::milliseconds>(loop_end_time - start_time).count() >= 100) {
            //     double sum = 0;
            //     for (double t : execution_times) {
            //         sum += t;
            //     }
            //     double average = sum / execution_times.size();
            //     std::cout << "Average execution time over last 0.1 seconds: " << average << " seconds" << std::endl;
            //     execution_times.clear();
            //     start_time = std::chrono::steady_clock::now();
            // }

            // std::this_thread::sleep_for(std::chrono::microseconds(encoder_update_rate));
        }
    }
}

int main() {
    pi = pigpio_start(NULL, NULL);
    set_mode(pi, ENC_PIN1, PI_INPUT);
    set_mode(pi, ENC_PIN2, PI_INPUT);

    int initial_pin1 = gpio_read(pi, ENC_PIN1);
    int initial_pin2 = gpio_read(pi, ENC_PIN2);
    std::cout << "Initial Pin States: ENC_PIN1 = " << initial_pin1 << ", ENC_PIN2 = " << initial_pin2 << std::endl;

    code = ((gpio_read(pi, ENC_PIN2) << 1) + gpio_read(pi, ENC_PIN1)) & 0xf;

    thread_encoder = std::thread(rotary_encoder);
    thread_encoder.join();

    return 0;
}
