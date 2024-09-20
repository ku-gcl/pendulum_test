#include "config.h"

// グローバル変数の定義
int pi;
int bus_acc;
int bus_gyr;
int sample_num = 100;
float meas_interval = 10000.0f; // us micro seconds
float theta_mean;
float theta_variance;
float theta_dot_mean;
float theta_dot_variance;
float y[4][1];

const double PI = 3.14159265358979323846; // 円周率
const double rad2deg = 180.0 / PI; // ラジアンを度に変換する定数
const double deg2rad = PI / 180.0;
// フォルダ名は最後に"/"を付けること
std::string LOG_DATA_DIR = "/home/ubuntu/pendulum_pj/pendulum_test/data/";

int encoder_update_rate = 25; // usec
int encoder_resolution = 100;
int encoder_value = 0;
int table[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};
float pre_theta2 = 0.0f;

float theta_update_freq = 400.0f; // Hz
float theta_update_interval = 1.0f / theta_update_freq;
int th1_dura =
    1000000 * 1.0f / theta_update_freq; // 2500usec for theta_update_freq=400
float theta_data_predict[2][1] = {{0}, {0}};
float theta_data[2][1] = {{0}, {0}};
float P_theta_predict[2][2] = {{1, 0}, {0, 0}};
float P_theta[2][2] = {{0}};
float A_theta[2][2] = {{1, -theta_update_interval}, {0, 1}};
float B_theta[2][1] = {{theta_update_interval}, {0}};
float C_theta[1][2] = {{1, 0}};

float x_data_predict[4][1] = {{0}, {0}, {0}, {0}};
float x_data[4][1] = {{0}, {0}, {0}, {0}};
float P_x_predict[4][4] = {{0}};
float P_x[4][4] = {{0}};
// float A_x[4][4] = {
//     {1.0022261662585084, 0.010007423157527657, 0.0, 4.465092819988847e-05},
//     {0.4449564215185254, 1.0022261662585084, 0.0, 0.008846819829339195},
//     {-0.0014716701781144724, -4.9287847971381375e-06, 1.0,
//      0.009711484266907367},
//     {-0.2915863440890973, -0.0014716701781144724, 0.0, 0.9428549643835731}};
// float B_x[4][1] = {{-0.0002871073058120404},
//                    {-0.05688541556931065},
//                    {0.0018551680368610664},
//                    {0.36744493066118095}};
// float C_x[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
// // float Gain[4] = {26.987014073601006, 4.147178701122192,
// 0.009365626359250269,
// //                  0.3061630717935332};
// float Gain[4] = {29.280615696374042, 4.508435265182227, 0.041799188964630814,
//                  0.32713225613691527};

// 2024.07.31
// matrix A_x
// matrix A_x
// float A_x[4][4] = {
//     {1.00045109094739, 0.01000150378447773, 0, 8.386731587674336e-06},
//     {0.090201897595168, 1.00045109094739, 0, 0.001664173048119428},
//     {-0.0004095056588252441, -1.370415174936198e-06, 1,
//     0.009763990740001295},
//     {-0.08125790999093362, -0.000409505658825244, 0, 0.953172320999233}};

// // matrix B_x
// float B_x[4][1] = {{-5.392702924173313e-05},
//                    {-0.0107007011838955},
//                    {0.001517549254106898},
//                    {0.3011039030399116}};

// 2024.09.17
float A_x[4][4] = {
    {1.002095034906568, 0.01000698521167948, 0, 4.373991285837984e-05},
    {0.4188267072641069, 1.002095034906568, 0, 0.008664755775936872},
    {-0.001109702169674044, -3.716839929508615e-06, 1, 0.0097068833062686},
    {-0.2198289309671251, -0.001109702169674044, 0, 0.9419534619478218}};

// matrix B_x
float B_x[4][1] = {{-0.0002812494396757963},
                   {-0.05571473621358585},
                   {0.001884752403108296},
                   {0.3732416284219275}};

// matrix C_x
float C_x[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

// matrix Gain
// float Gain[4] = {46.39588288288118, 15.61230765832237, 0.04331076212710311,
//                  0.3503207707244065};
// float Gain[4] = {213.2233413566366, 72.50692577965587, 0.8402908518176702,
//                  1.612941214198182};

// // Q=[1000;1000;100;100], R=1
// float Gain[4] = {230.6435174327925, 37.65196816823165, 2.161898197361982,
//                  3.05486813595216};

// // Q=[1;1;10;10], R=1000
float Gain[4] = {30.96868168661306, 4.893479084441405, 0.09308744122875257,
                 0.3711858579440717};

// matrix Gain
// Q=[1;1;1;1], R=1
// float Gain[4] = {93.72631845295078, 15.01855314980138, 0.7925542489192221,
//                  1.238042162779275};

// measurement noise matrix
float measure_variance_mat[4][4] = {{0}};
float voltage_error = 0.01f; // volt
float voltage_variance = voltage_error * voltage_error;

float feedback_rate = 0.01f; // sec
int feedback_dura = 10;      // msec
float motor_value = 0.0f;
int pwm_duty = 0;
int motor_direction = 1;
// float motor_offset = 0.17f; // volt
float motor_offset = 0.00f;   // volt
float MAX_VOLTAGE = 3.3f;     // モーターに入力する最大電圧 [V]
float BATTERY_VOLTAGE = 5.0f; // バッテリー電圧

// encoder
int enc_syn = 1;
int update_theta_syn_flag = 1;
int code;

// imu
float xAccl = 0.0f, yAccl = 0.0f, zAccl = 0.0f;
float xGyro = 0.0f, yGyro = 0.0f, zGyro = 0.0f;
