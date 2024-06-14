#include "pigpiod_if2.h"
#include <cmath>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <thread>
#include <chrono>
// for detect ctrol+c
#include <csignal>
#include <cstdlib>

#include "../include/matrix_operations.h"
#include "../include/sensor.h"

std::thread thread1;
std::thread thread2;
std::thread thread_csv;
int enc_syn = 1;
int update_theta_syn_flag = 1;

bool stopThread1=false;
bool stopThread2=false;
bool stopThreadcsv=false;

std::ofstream csvFile;

int csv_rate = 10;

float time_csv=0;
float theta1_csv=0;
float theta2_csv=0;
float theta1dot_csv=0;
float theta2dot_csv=0;

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
int sample_num = 100;
float meas_interval = 10000; // usec
float theta_mean=0;
float theta_variance;
float theta_dot_mean;
float theta_dot_variance;

//=========================================================
// Rotary encoder variables
int code;
int rotary_encoder_update_rate = 25; // usec
int rotary_encoder_resolution = 100;
int encoder_value = 0;
int table[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};
float pre_theta2 = 0;

//=========================================================
// Kalman filter (for angle estimation) variables
// Update rate
float theta_update_freq = 400; // Hz
float theta_update_interval = 1.0f / theta_update_freq;
int th1_dura = 1000000 * 1.0f / theta_update_freq;
// State vector
//[[theta(degree)], [offset of theta_dot(degree/sec)]]
float theta_data_predict[2][1];
float theta_data[2][1];
// Covariance matrix
float P_theta_predict[2][2];
float P_theta[2][2];
//"A" of the state equation
float A_theta[2][2] = {{1, -theta_update_interval}, {0, 1}}; // const
//"B" of the state equation
float B_theta[2][1] = {{theta_update_interval}, {0}}; // const
//"C" of the state equation
float C_theta[1][2] = {{1, 0}}; // const

//=========================================================
// Kalman filter (for all system estimation) variables
// State vector
//[[theta1(rad)], [theta1_dot(rad/s)], [theta2(rad)]. [theta2_dot(rad/s)]]
float x_data_predict[4][1];
float x_data[4][1];
// Covariance matrix
float P_x_predict[4][4];
float P_x[4][4];

//"A" of the state equation (update freq = 100 Hz)
//"B" of the state equation (update freq = 100 Hz)
//matrix Ax (discrete time)
float A_x[4][4]=  {{1.0022101364920777, 0.01000736933759299, 0.0, 4.3210055583546874e-05},
 {0.4417962226778376, 1.0022101364920777, 0.0, 0.008560652140727404},
 {-0.0013550406779803275, -4.538359252051067e-06, 1.0, 0.009709237793470039},
 {-0.26845676831626103, -0.0013550406779803275, 0.0, 0.94241471701796}};

//matrix Bx (discrete time)
float B_x[4][1]=  {{-0.00027784243559379453},
 {-0.05504534555508876},
 {0.0018696129535105486},
 {0.3702757393392488}};

//"C" of the state equation (update freq = 100 Hz)
float C_x[4][4] = {
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 1}};

// measurement noise
float measure_variance_mat[4][4];
// System noise
float voltage_error = 0.01; // volt
float voltage_variance = voltage_error * voltage_error;

//=========================================================
// Motor control variables
float feedback_rate = 0.01; //sec
int feedback_dura = 10; //msec
float motor_value = 0;
int pwm_duty = 0;
int motor_direction = 1;
float motor_offset = 0.17; // volt

//=========================================================
// Gain vector for the state feedback
//(R=1000, Q = diag(1, 1, 10, 10), f=100Hz)
// float Gain[4] = {29.87522919, 4.59857246, 0.09293, 0.37006248};
float Gain[4] = {96.34668317353481, 15.112060762484786, 0.7925636783224236, 1.2331571996093764};

//=========================================================
// Rotary encoder polling function
// It takes 4usec. (NUCLEO-F401RE 84MHz)
//=========================================================
void rotary_encoder(){
    while(!stopThread1){
        if (enc_syn == 1)
        {
            // check the movement
            code = ((code << 2) + (gpio_read(pi, pin2) << 1) + gpio_read(pi, pin1)) & 0xf; // !caution!
            std::cout << 1 << std::endl;
            // update the encoder value
            int value = -1 * table[code];
            encoder_value += value;
            std::chrono::microseconds dura1(rotary_encoder_update_rate);
            // std::chrono::milliseconds dura1(rotary_encoder_update_rate);
            std::this_thread::sleep_for(dura1);
        }else{
            std::cout << -1 << std::endl;
        }
    }
}

void csv_write(){
    while(!stopThreadcsv){
        csvFile << time_csv << "," << theta1_csv << "," << theta2_csv << "," << theta1dot_csv << "," << theta2dot_csv << std::endl;
        // std::cout << "CSVに書き込みました" << std::endl;
        time_csv=time_csv+10; //msec
        std::chrono::milliseconds dura_csv(csv_rate);
        std::this_thread::sleep_for(dura_csv);
    }
}

void signalHandler(int signum) {
    if (signum == SIGINT) {
        std::cout << "Ctrl+Cが検出されました。スレッドを終了し、ファイルを閉じます。" << std::endl;
        set_mode(pi, LED_R, PI_OUTPUT);
        set_mode(pi, LED_Y, PI_OUTPUT);
        set_mode(pi, LED_G, PI_OUTPUT);
        set_mode(pi, IN1, PI_OUTPUT);
        set_mode(pi, IN2, PI_OUTPUT);
        gpio_write(pi, IN1, 0);
        gpio_write(pi, IN2, 0);
        gpio_write(pi, LED_R, 0);
        gpio_write(pi, LED_Y, 0);
        gpio_write(pi, LED_G, 0);
        csvFile.close();
        pigpio_stop(pi);
        exit(signum);
    }
}

int main()
{
    csvFile.open("output_test.csv"); // ファイルを開く
    std::signal(SIGINT, signalHandler);
    csvFile << "time" << "," << "theta1" << "," << "theta2" << "," << "theta1_dot" << "," << "theta2_dot" << std::endl;
    
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
    thread_csv = std::thread(csv_write);
    sleep(1);
    gpio_write(pi, LED_R, 0);
    gpio_write(pi, LED_G, 0);    

    //-------------------------------------------
    // Timer
    //-------------------------------------------
    thread1 = std::thread(rotary_encoder);

    //-------------------------------------------
    // initialization done
    //-------------------------------------------
    gpio_write(pi, LED_Y, 0);

    //===========================================
    // Main loop
    // it takes 700 usec (calculation)
    //===========================================
    float t1;
    float t1dot;
    
    while (1)
    {
        float 
        t1 = get_acc_data(pi, bus_acc); // degree
        t1dot = get_gyr_data(pi, bus_gyr); // degree/sec
        theta2_csv = encoder_value * (2 * 3.1415926f) / (4 * rotary_encoder_resolution);
        theta2dot_csv = (theta2_csv - pre_theta2) / feedback_rate;
        pre_theta2 = theta2_csv;

        theta1_csv=t1*3.1415926/180;
        theta1dot_csv=t1dot*3.1415926/180;

        std::chrono::milliseconds mainlooprate(100);
        std::this_thread::sleep_for(mainlooprate);
    }
    csvFile.close();
    //======10000//=====================================
    // Main loop (end)
    //===========================================
    return 0;
}
