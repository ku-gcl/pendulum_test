#include <pigpiod_if2.h>
#include <csignal>
#include <unistd.h>
#include <iostream>
#include <thread>
#include <chrono>

#include "config.h"
#include "sensor.h"
#include "kalman_filter.h"
#include "motor_control.h"
#include "encoder.h"


std::thread thread1;
std::thread thread2;

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

    encoder_value = 0;
    motor_driver_init(pi);
    kalman_filter_init();
}

int main()
{
    setup();

    thread1 = std::thread(rotary_encoder);
    thread2 = std::thread(update_theta, bus_acc, bus_gyr);
    thread1.join();
    thread2.join();

    gpio_write(pi, LED_Y, 0);

    while (true)
    {
        update_theta_syn_flag = 0;
        gpio_write(pi, LED_R, 0);
        gpio_write(pi, LED_G, 0);

        kalman_filter_update();
        motor_control_update();

        pre_theta2 = y[2][0];
        update_theta_syn_flag = 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(feedback_dura));
    }
    return 0;
}
