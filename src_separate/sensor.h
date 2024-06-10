#pragma once

void acc_init(int pi, int bus, int sample_num, float meas_interval, float &theta_mean, float &theta_variance);
void gyr_init(int pi, int bus, int sample_num, float meas_interval, float &theta_dot_mean, float &theta_dot_variance);
float get_acc_data(int pi, int bus);
float get_gyr_data(int pi, int bus);
