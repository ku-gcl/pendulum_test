#include <chrono>
#include <thread>

#include "config.h"
#include "kalman_filter.h"
#include "matrix_operations.h"
#include "sensor.h"

void kalman_filter_init() {
    // 初期値の設定
    theta_data_predict[0][0] = 0;
    theta_data_predict[1][0] = theta_dot_mean;

    P_theta_predict[0][0] = 1;
    P_theta_predict[0][1] = 0;
    P_theta_predict[1][0] = 0;
    P_theta_predict[1][1] = theta_dot_variance;

    for (int i = 0; i < 4; i++) {
        x_data_predict[i][0] = 0;
    }

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            P_x_predict[i][j] = 0;
        }
    }
    for (int i = 0; i < 4; i++) {
        P_x_predict[i][i] = 1e-4;
    }

    float deg_rad_coeff = (3.14 * 3.14) / (180 * 180);
    measure_variance_mat[0][0] = theta_variance * deg_rad_coeff;
    measure_variance_mat[1][1] = theta_dot_variance * deg_rad_coeff;
    float encoder_error = 0.1f * 2 * 3.14f / (4 * encoder_resolution);
    measure_variance_mat[2][2] = encoder_error * encoder_error;
    float encoder_rate_error = encoder_error / feedback_rate;
    measure_variance_mat[3][3] = encoder_rate_error * encoder_rate_error;
}

void update_theta(int bus_acc, int bus_gyr) {
    // if (update_theta_syn_flag == 0) {
    //     return;
    // }
    while (true) {
        if (update_theta_syn_flag == 1) {
            enc_syn = 0;

            // 姿勢角のセンサ値
            float theta = get_acc_data(pi, bus_acc);
            // 姿勢角速度のセンサ値
            float theta_dot_gyro = get_gyr_data(pi, bus_gyr);

            // calculate Kalman gain: G = P'C^T(W+CP'C^T)^-1
            float P_CT[2][1] = {};
            float tran_C_theta[2][1] = {};
            mat_tran(C_theta[0], tran_C_theta[0], 1, 2);
            mat_mul(P_theta_predict[0], tran_C_theta[0], P_CT[0], 2, 2, 2, 1);
            float G_temp1[1][1] = {};
            mat_mul(C_theta[0], P_CT[0], G_temp1[0], 1, 2, 2, 1);
            float G_temp2 = 1.0f / (G_temp1[0][0] + theta_variance);
            float G[2][1] = {};
            mat_mul_const(P_CT[0], G_temp2, G[0], 2, 1);

            // theta_data estimation: theta = theta'+G(y-Ctheta')
            float C_theta_theta[1][1] = {};
            mat_mul(C_theta[0], theta_data_predict[0], C_theta_theta[0], 1, 2,
                    2, 1);
            float delta_y = theta - C_theta_theta[0][0];
            float delta_theta[2][1] = {};
            mat_mul_const(G[0], delta_y, delta_theta[0], 2, 1);
            mat_add(theta_data_predict[0], delta_theta[0], theta_data[0], 2, 1);

            // calculate covariance matrix: P=(I-GC)P'
            float GC[2][2] = {};
            float I2[2][2] = {{1, 0}, {0, 1}};
            mat_mul(G[0], C_theta[0], GC[0], 2, 1, 1, 2);
            float I2_GC[2][2] = {};
            mat_sub(I2[0], GC[0], I2_GC[0], 2, 2);
            mat_mul(I2_GC[0], P_theta_predict[0], P_theta[0], 2, 2, 2, 2);

            // predict the next step data: theta'=Atheta+Bu
            float A_theta_theta[2][1] = {};
            float B_theta_dot[2][1] = {};
            mat_mul(A_theta[0], theta_data[0], A_theta_theta[0], 2, 2, 2, 1);
            mat_mul_const(B_theta[0], theta_dot_gyro, B_theta_dot[0], 2, 1);
            mat_add(A_theta_theta[0], B_theta_dot[0], theta_data_predict[0], 2,
                    1);

            // predict covariance matrix: P'=APA^T + BUB^T
            float AP[2][2] = {};
            float APAT[2][2] = {};
            float tran_A_theta[2][2] = {};
            mat_tran(A_theta[0], tran_A_theta[0], 2, 2);
            mat_mul(A_theta[0], P_theta[0], AP[0], 2, 2, 2, 2);
            mat_mul(AP[0], tran_A_theta[0], APAT[0], 2, 2, 2, 2);
            float BBT[2][2];
            float tran_B_theta[1][2] = {};
            mat_tran(B_theta[0], tran_B_theta[0], 2, 1);
            mat_mul(B_theta[0], tran_B_theta[0], BBT[0], 2, 1, 1, 2);
            float BUBT[2][2] = {};
            mat_mul_const(BBT[0], theta_dot_variance, BUBT[0], 2, 2);
            mat_add(APAT[0], BUBT[0], P_theta_predict[0], 2, 2);

            enc_syn = 1;
        }
    }
    std::this_thread::sleep_for(std::chrono::microseconds(th1_dura));
}

void kalman_filter_update() {
    // measurement data
    float theta1_dot_temp = get_gyr_data(pi, bus_gyr);
    y[0][0] = theta_data[0][0] * 3.14f / 180;                         // rad
    y[1][0] = (theta1_dot_temp - theta_data[1][0]) * 3.14f / 180;     // rad/s
    y[2][0] = encoder_value * (2 * 3.14f) / (4 * encoder_resolution); // rad
    y[3][0] = (y[2][0] - pre_theta2) / feedback_rate;                 // rad/s

    // calculate Kalman gain: G = P'C^T(W+CP'C^T)^-1
    float tran_C_x[4][4];
    float P_CT[4][4];
    float G_temp1[4][4];
    float G_temp2[4][4];
    float G_temp2_inv[4][4];
    float G[4][4];
    mat_tran(C_x[0], tran_C_x[0], 4, 4);
    mat_mul(P_x_predict[0], tran_C_x[0], P_CT[0], 4, 4, 4, 4);
    mat_mul(C_x[0], P_CT[0], G_temp1[0], 4, 4, 4, 4);
    mat_add(G_temp1[0], measure_variance_mat[0], G_temp2[0], 4, 4);
    mat_inv(G_temp2[0], G_temp2_inv[0], 4, 4);
    mat_mul(P_CT[0], G_temp2_inv[0], G[0], 4, 4, 4, 4);

    // x_data estimation: x = x'+G(y-Cx')
    float C_x_x[4][1];
    float delta_y[4][1];
    float delta_x[4][1];
    mat_mul(C_x[0], x_data_predict[0], C_x_x[0], 4, 4, 4, 1);
    mat_sub(y[0], C_x_x[0], delta_y[0], 4, 1);
    mat_mul(G[0], delta_y[0], delta_x[0], 4, 4, 4, 1);
    mat_add(x_data_predict[0], delta_x[0], x_data[0], 4, 1);

    // calculate covariance matrix: P=(I-GC)P'
    float GC[4][4];
    float I4[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    float I4_GC[4][4];
    mat_mul(G[0], C_x[0], GC[0], 4, 4, 4, 4);
    mat_sub(I4[0], GC[0], I4_GC[0], 4, 4);
    mat_mul(I4_GC[0], P_x_predict[0], P_x[0], 4, 4, 4, 4);

    // predict the next step data: x'=Ax+Bu
    float Vin = motor_value;
    if (motor_value > MAX_VOLTAGE) {
        Vin = MAX_VOLTAGE;
    }
    if (motor_value < -MAX_VOLTAGE) {
        Vin = -MAX_VOLTAGE;
    }
    float A_x_x[4][1];
    float B_x_Vin[4][1];
    mat_mul(A_x[0], x_data[0], A_x_x[0], 4, 4, 4, 1);
    mat_mul_const(B_x[0], Vin, B_x_Vin[0], 4, 1);
    mat_add(A_x_x[0], B_x_Vin[0], x_data_predict[0], 4, 1);

    // predict covariance matrix: P'=APA^T + BUB^T
    float tran_A_x[4][4];
    float AP[4][4];
    float APAT[4][4];
    float BBT[4][4];
    float tran_B_x[1][4];
    float BUBT[4][4];
    mat_tran(A_x[0], tran_A_x[0], 4, 4);
    mat_mul(A_x[0], P_x[0], AP[0], 4, 4, 4, 4);
    mat_mul(AP[0], tran_A_x[0], APAT[0], 4, 4, 4, 4);
    mat_tran(B_x[0], tran_B_x[0], 4, 1);
    mat_mul(B_x[0], tran_B_x[0], BBT[0], 4, 1, 1, 4);
    mat_mul_const(BBT[0], voltage_variance, BUBT[0], 4, 4);
    mat_add(APAT[0], BUBT[0], P_x_predict[0], 4, 4);
}
