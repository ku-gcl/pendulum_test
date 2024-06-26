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
float A_x[4][4] = {
    {1.0022261662585084, 0.010007423157527657, 0.0, 4.465092819988847e-05},
    {0.4449564215185254, 1.0022261662585084, 0.0, 0.008846819829339195},
    {-0.0014716701781144724, -4.9287847971381375e-06, 1.0,
     0.009711484266907367},
    {-0.2915863440890973, -0.0014716701781144724, 0.0, 0.9428549643835731}};
float B_x[4][1] = {{-0.0002871073058120404},
                   {-0.05688541556931065},
                   {0.0018551680368610664},
                   {0.36744493066118095}};
float C_x[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

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
float motor_offset = 0.00f; // volt
float MAX_VOLTAGE = 3.3f;   // エンコーダーに入力する最大電圧 [V]
// float Gain[4] = {26.987014073601006, 4.147178701122192, 0.009365626359250269,
//                  0.3061630717935332};
float Gain[4] = {29.280615696374042, 4.508435265182227, 0.041799188964630814,
                 0.32713225613691527};

// encoder
int enc_syn = 1;
int update_theta_syn_flag = 1;
int code;

// imu
float xAccl = 0.0f, yAccl = 0.0f, zAccl = 0.0f;
float xGyro = 0.0f, yGyro = 0.0f, zGyro = 0.0f;
