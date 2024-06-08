#include "pigpiod_if2.h"
#include <cmath>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <chrono>
// for detect ctrol+c
#include <csignal>
#include <cstdlib>

#include "matrix_operations.h"
#include "sensor.h"

std::ofstream csvFile;

//=========================================================
// Port Setting
int pi;
const int ACC_ADDR = 0x19;
const int GYR_ADDR = 0x69;
const int pin1 = 24; // to A
const int pin2 = 23; // to B

const int IN1 = 6;  // Motor driver input 1
const int IN2 = 5;  // Motor driver input 2
const int PWM = 12; // Motor driver PWM input

const int LED_R = 17;
const int LED_Y = 27;
const int LED_G = 22;

//=========================================================
// Accelerometer and gyro statistical data
int sample_num = 1000;
float meas_interval = 2500; // usec
float theta_mean;
float theta_variance;
float theta_dot_mean;
float theta_dot_variance;

//=========================================================
// Main
//=========================================================
int main()
{
    pi = pigpio_start(NULL, NULL);
    int bus_acc = i2c_open(pi, 1, ACC_ADDR, 0);
    int bus_gyr = i2c_open(pi, 1, GYR_ADDR, 0);
    set_mode(pi, LED_R, PI_OUTPUT);
    set_mode(pi, LED_Y, PI_OUTPUT);
    set_mode(pi, LED_G, PI_OUTPUT);
    set_mode(pi, pin1, PI_INPUT);
    set_mode(pi, pin2, PI_INPUT);
    set_mode(pi, IN1, PI_OUTPUT);
    set_mode(pi, IN2, PI_OUTPUT);
    set_mode(pi, PWM, PI_OUTPUT);

    //-------------------------------------------
    // LED
    //-------------------------------------------
    gpio_write(pi, LED_R, 1);
    gpio_write(pi, LED_Y, 1);
    gpio_write(pi, LED_G, 1);
    sleep(1);
    gpio_write(pi, LED_R, 0);
    gpio_write(pi, LED_G, 0);

    //-------------------------------------------
    // Accelerometer & Gyro initialization
    //-------------------------------------------
    
    csvFile.open("output2.csv");
    csvFile << "theta_mean" << "," << "theta_variance" << "," << "theta_dot_mean" << "," << "theta_dot_variance" << std::endl;
    
    for(int i = 0; i < 10; i++)
    {
    acc_init(pi, bus_acc, sample_num, meas_interval, theta_mean, theta_variance);
    gyr_init(pi, bus_gyr, sample_num, meas_interval, theta_dot_mean, theta_dot_variance);
    csvFile << theta_mean << "," << theta_variance << "," << theta_dot_mean << "," << theta_dot_variance << std::endl;
    }
    
    csvFile.close();
    return 0;
}