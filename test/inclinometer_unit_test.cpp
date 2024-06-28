#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <pigpiod_if2.h>
#include <thread>
#include <unistd.h>

// #define Addr_Accl 0x19;
// #define Addr_Gyro 0x69;

int pi;                                   // raspberry pi
const double PI = 3.14159265358979323846; // 円周率
const double rad2deg = 180.0 / PI; // ラジアンを度に変換する定数
const double deg2rad = PI / 180.0;

// sensor
int bus_acc, bus_gyr;
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

// update rate
int feedback_dura = 10; // msec

// sensor data
float theta;
float theta_dot_gyro;

float xAccl = 0.0f, yAccl = 0.0f, zAccl = 0.0f;
float xGyro = 0.0f, yGyro = 0.0f, zGyro = 0.0f;

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

//=========================================================
// Get Data
//=========================================================
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

float get_gyr_data(int pi, int bus) {
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
    return theta_p_dot_deg
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

    std::cout << "theta_mean= " << theta_mean << std::endl;
    std::cout << "theta_variance= " << theta_variance << std::endl;
    std::cout << "--------------------------------------" << std::endl;

    return;
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

    gpio_write(pi, LED_R, 1);
    gpio_write(pi, LED_G, 1);

    gpio_write(pi, LED_R, 0);
    gpio_write(pi, LED_G, 0);
}

int main() {
    setup();

    gpio_write(pi, LED_Y, 0);

    auto start = std::chrono::system_clock::now();

    while (true) {
        auto loop_start = std::chrono::system_clock::now();

        gpio_write(pi, LED_R, 0);
        gpio_write(pi, LED_G, 0);

        // スタートからの経過時間の取得
        auto elapsed = std::chrono::system_clock::now() - start;
        float elapsed_time = std::chrono::duration<float>(elapsed).count();

        float theta_p = get_theta_p_deg(pi, bus_acc) * deg2rad;         // rad
        float theta_p_dot = get_theta_p_dot_deg(pi, bus_gyr) * deg2rad; // rad/s
        float theta_p_kf = 0.0f * deg2rad;
        float theta_p_dot_kf = 0.0f * deg2rad;

        // display表示
        console_write(elapsed_time, theta_p, theta_p_dot, theta_p_kf,
                      theta_p_dot_kf);

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
    return 0;
}
