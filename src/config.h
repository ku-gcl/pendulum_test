#pragma once

#include <string>

extern int pi;
extern int bus_acc;
extern int bus_gyr;
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
extern std::string LOG_DATA_DIR;

extern int sample_num;
extern float meas_interval;
extern float theta_mean;
extern float theta_variance;
extern float theta_dot_mean;
extern float theta_dot_variance;

extern int encoder_update_rate;
extern int encoder_resolution;
extern int encoder_value;
extern int table[16];
extern float pre_theta2;

extern float theta_update_freq;
extern float theta_update_interval;
extern int th1_dura;
extern float theta_data_predict[2][1];
extern float theta_data[2][1];
extern float P_theta_predict[2][2];
extern float P_theta[2][2];
extern float A_theta[2][2];
extern float B_theta[2][1];
extern float C_theta[1][2];

extern float x_data_predict[4][1];
extern float x_data[4][1];
extern float P_x_predict[4][4];
extern float P_x[4][4];
extern float A_x[4][4];
extern float B_x[4][1];
extern float C_x[4][4];
extern float measure_variance_mat[4][4];
extern float voltage_error;
extern float voltage_variance;

extern float feedback_rate;
extern int feedback_dura;
extern float motor_value;
extern int pwm_duty;
extern int motor_direction;
extern float motor_offset;
extern float MAX_VOLTAGE;
extern float Gain[4];

// encoder
extern int enc_syn;
extern int update_theta_syn_flag;

extern float y[4][1];