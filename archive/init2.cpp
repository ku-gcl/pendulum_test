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
int meas_interval = 2500; // usec
float theta_mean;

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
    
    // initialize ACC register 0x0F (range)
    // Full scale = +/- 2 G
    i2c_write_byte_data(pi, bus_acc, 0x0F, 0x03);
    // initialize ACC register 0x10 (band width)
    // Filter bandwidth = 1000 Hz
    i2c_write_byte_data(pi, bus_acc, 0x10, 0x0F);

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
    
    csvFile.open("output3.csv");
    csvFile << "theta" << std::endl;
    
    float theta_array[sample_num];
    for (int i = 0; i < sample_num; i++)
    {
        theta_array[i] = get_acc_data(pi, bus_acc)*3.1415926/180 ;
        csvFile << theta_array[i]  << std::endl;
        usleep(meas_interval);
    }
    
    theta_mean = 0;
    for (int i = 0; i < sample_num; i++)
    {
        theta_mean += theta_array[i];
    }
    theta_mean /= sample_num;
    std::cout << "theta_mean:" << theta_mean << std::endl;

    csvFile.close();
    return 0;
}
