#pragma once

void acc_init(int pi, int bus, int sample_num, float meas_interval,
              float &theta_mean, float &theta_variance);
void bmx055_init(int pi, int bus_acc, int bus_gyr);
void gyr_init(int pi, int bus, int sample_num, float meas_interval,
              float &theta_dot_mean, float &theta_dot_variance);
void get_acc_data(int pi, int bus_acc);
void get_gyr_data(int pi, int bus_gyr);
float get_theta_p_deg(int pi, int bus_acc);
float get_theta_p_dot_deg(int pi, int bus_gyro);