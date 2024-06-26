#include "pigpiod_if2.h"
#include <cmath>
#include <unistd.h>
#include <iostream>
#include <thread>
#include <chrono>
// for detect ctrol+c
#include <csignal>
#include <cstdlib>
#include <fstream>

#include "matrix_operations.h"
#include "sensor.h"
#include "signal_handler.h"


std::thread thread1;
std::thread thread_csv;
int enc_syn = 1;
int update_theta_syn_flag = 1;

//=========================================================
// Port Setting
extern int pi;
const int ACC_ADDR = 0x19;
const int GYR_ADDR = 0x69;
const int pin1 = 24; // to A
const int pin2 = 23; // to B

extern const int IN1 = 6;  // Motor driver input 1
extern const int IN2 = 5;  // Motor driver input 2
const int PWM = 12; // Motor driver PWM input

extern const int LED_R = 17;
extern const int LED_Y = 27;
extern const int LED_G = 22;

//=========================================================
// Accelerometer and gyro statistical data
int sample_num = 100;
float meas_interval = 10000;    // us micro seconds
float theta_mean;
float theta_variance;
float theta_dot_mean;
float theta_dot_variance;

//=========================================================
// Rotary encoder variables
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
int th1_dura = 1000 * 1.0f / theta_update_freq;
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
// float A_x[4][4] = {
//     {1.00210e+00, 1.00070e-02, 0.00000e+00, 3.86060e-05},
//     {4.20288e-01, 1.00210e+00, 0.00000e+00, 7.65676e-03},
//     {-1.15751e-03, -3.87467e-06, 1.00000e+00, 9.74129e-03},
//     {-2.29569e-01, -1.15751e-03, 0.00000e+00, 9.48707e-01}};
//"B" of the state equation (update freq = 100 Hz)
// float B_x[4][1] = {
//     {-2.70805e-04},
//     {-5.37090e-02},
//     {1.81472e-03},
//     {3.59797e-01}};
//matrix Ax (discrete time)
float A_x[4][4]=  {{1.0020824276428177, 0.01000694384099276, 0.0, 4.08544363598033e-05},
 {0.41622688505520855, 1.0020824276428177, 0.0, 0.00809455762284398},
 {-0.001467099554375857, -4.913480777961718e-06, 1.0, 0.009712010551520779},
 {-0.2906788818907422, -0.001467099554375857, 0.0, 0.9429583658672117}};

//matrix Bx (discrete time)
float B_x[4][1]=  {{-0.00026269570704606055},
 {-0.05204833862425401},
 {0.0018517840051390114},
 {0.3667800548661786}};

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
int feedback_dura = 500; //msec
float motor_value = 0;
int pwm_duty = 0;
int motor_direction = 1;
float motor_offset = 0.17; // volt
int i=0;
//=========================================================
// Gain vector for the state feedback
//(R=1000, Q = diag(1, 1, 10, 10), f=100Hz)
// float Gain[4] = {29.87522919, 4.59857246, 0.09293, 0.37006248};

float Gain[4] = {29.30755259, 4.80340051, 0.02968736, 0.3196894};
float p;
float theta2;
float theta2_csv;

void rotary_encoder()
{
    if (enc_syn == 1)
    {
        static int code;
        // check the movement
        code = ((code << 2) + (gpio_read(pi, pin2) << 1) + gpio_read(pi, pin1)) & 0xf; // !caution!
        // update the encoder value
        int value = -1 * table[code];
        encoder_value += value;
        std::chrono::microseconds dura1(rotary_encoder_update_rate);
        std::this_thread::sleep_for(dura1);
        return;
    }
}
float p_csv;
float time_csv=0;
bool stopThread_csv = false;
std::ofstream csvFile;
void csv_wirte(){
    while(!stopThread_csv){
        theta2 = encoder_value * (2 * 3.14f) / (4 * rotary_encoder_resolution);
        std::cout << theta2 << std::endl;
        theta2_csv=theta2;
        p_csv = (theta2_csv-pre_theta2)/0.1;
        csvFile << time_csv << "," << theta2_csv << "," << p_csv << std::endl;
        time_csv=time_csv+0.1; //sec
        pre_theta2 = theta2_csv;
        std::chrono::milliseconds dura_csv(100);
        std::this_thread::sleep_for(dura_csv);
    }
}
//=========================================================
// Main
//=========================================================
int main()
{
    
    csvFile.open("output.csv"); // ファイルを開く
    std::signal(SIGINT, signalHandler);
    csvFile << "time" << "," << "theta" <<"," << "p" << std::endl;
    

    thread_csv = std::thread(csv_wirte);
    thread1 = std::thread(rotary_encoder);
    thread1.detach();
    thread_csv.detach();
    // Ctrl+Cによる中断をキャッチするためのシグナルハンドラを設定
    std::signal(SIGINT, signalHandler);
    // Ctrl+Z
    std::signal(SIGTSTP, signalHandler);

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
    // Rotary encoder initialization
    //-------------------------------------------
    encoder_value = 0;
    //-------------------------------------------
    // Motor driver intialization
    //-------------------------------------------
    gpio_write(pi, IN1, 0);
    gpio_write(pi, IN2, 0);
    set_PWM_frequency(pi, PWM, 10000);
    set_PWM_range(pi, PWM, 100);
    set_PWM_dutycycle(pi, PWM, 0);
    
    //-------------------------------------------
    // initialization done
    //-------------------------------------------
    gpio_write(pi, LED_Y, 0);
    
    //===========================================
    // Main loop
    // it takes 700 usec (calculation)
    //===========================================
    
    while (1)
    {
        i=i+1;

        pwm_duty=pwm_duty+20;
        if (pwm_duty>100){pwm_duty=0;}
        // reverse
        set_PWM_range(pi, PWM, 100);
        set_PWM_dutycycle(pi, PWM, pwm_duty);
        gpio_write(pi, IN1, 0);
        gpio_write(pi, IN2, 1);
        if(i%2==0){
        gpio_write(pi, LED_R, 1);
        gpio_write(pi, LED_G, 0);
        }else{
            gpio_write(pi, LED_R, 0);
            gpio_write(pi, LED_G, 1);
        }
        motor_direction = 2;
     
        // wait
        std::chrono::seconds dura3(1);
        std::this_thread::sleep_for(dura3);
    }
    csvFile.close();
    //===========================================
    // Main loop (end)
    //===========================================
    return 0;
}
