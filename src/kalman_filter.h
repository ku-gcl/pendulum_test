#pragma once

extern float y[4][1];
void kalman_filter_init();
void kalman_filter_update();
void update_theta(int bus_acc, int bus_gyr);
