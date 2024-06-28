#include <cmath>
#include <unistd.h>

#include "config.h"
#include "pigpiod_if2.h"
#include "sensor.h"

// float xAccl = 0.0f, yAccl = 0.0f, zAccl = 0.0f;
// float xGyro = 0.0f, yGyro = 0.0f, zGyro = 0.0f;

void bmx055_init(int pi, int bus_acc, int bus_gyr) {
    // Accelerometer initialization
    // range: register 0x0F, Full scale = +/- 2 G
    i2c_write_byte_data(pi, bus_acc, 0x0F, 0x03);
    // band width: 0x10, Filter bandwidth = 1000 Hz
    i2c_write_byte_data(pi, bus_acc, 0x10, 0x0F);
    // sleep duration: 0x11, 0.5 ms
    i2c_write_byte_data(pi, bus_acc, 0x11, 0x00);

    // Gyroscope initialization
    // range: register 0x0F, Full scale = +/- 1000 deg/s
    i2c_write_byte_data(pi, bus_gyr, 0x0F, 0x01);
    // band width: 0x10, Filter bandwidth = 116 Hz
    i2c_write_byte_data(pi, bus_gyr, 0x10, 0x02);
    // sleep duration: 0x11, 0.5 ms
    i2c_write_byte_data(pi, bus_gyr, 0x11, 0x00);
}

//=========================================================
// Accelerometer (BMX055)
//=========================================================
// get data
void get_acc_data(int pi, int bus_acc) {
    char data[6];
    i2c_read_i2c_block_data(pi, bus_acc, 0x02, data, 6);

    int x = ((data[1] << 8) | (data[0] & 0xF0)) >> 4;
    if (x > 2047)
        x -= 4096;
    int y = ((data[3] << 8) | (data[2] & 0xF0)) >> 4;
    if (y > 2047)
        y -= 4096;
    int z = ((data[5] << 8) | (data[4] & 0xF0)) >> 4;
    if (z > 2047)
        z -= 4096;

    xAccl = x * 0.00098f; // range = +/-2g
    yAccl = y * 0.00098f; // range = +/-2g
    zAccl = z * 0.00098f; // range = +/-2g
}

// statistical data of accelerometer
// By passing references to theta_mean and theta_variance, using "&",
// you can modify the values at their referenced locations.
void acc_init(int pi, int bus, int sample_num, float meas_interval,
              float &theta_mean, float &theta_variance) {
    // get data
    float theta_array[sample_num];
    for (int i = 0; i < sample_num; i++) {
        theta_array[i] = get_theta_p_deg(pi, bus);
        usleep(meas_interval);
    }

    // calculate mean
    theta_mean = 0;
    for (int i = 0; i < sample_num; i++) {
        theta_mean += theta_array[i];
    }
    theta_mean /= sample_num;

    // calculate variance
    float temp;
    theta_variance = 0;
    for (int i = 0; i < sample_num; i++) {
        temp = theta_array[i] - theta_mean;
        theta_variance += temp * temp;
    }
    theta_variance /= sample_num;
    return;
}

//=========================================================
// Gyroscope (BMX055)
//=========================================================
// get data
void get_gyr_data(int pi, int bus_gyr) {
    char data[6];
    i2c_read_i2c_block_data(pi, bus_gyr, 0x02, data, 6);

    int x = (data[1] << 8) | data[0];
    if (x > 32767)
        x -= 65536;
    int y = (data[3] << 8) | data[2];
    if (y > 32767)
        y -= 65536;
    int z = (data[5] << 8) | data[4];
    if (z > 32767)
        z -= 65536;

    // +1000 (deg/sec) / 2^15 = 0.0305176
    xGyro = x * 0.0305176f; //  Full scale = +/- 1000 degree/s
    yGyro = y * 0.0305176f; //  Full scale = +/- 1000 degree/s
    zGyro = z * 0.0305176f; //  Full scale = +/- 1000 degree/s
}

// statistical data of gyro
void gyr_init(int pi, int bus, int sample_num, float meas_interval,
              float &theta_dot_mean, float &theta_dot_variance) {
    // get data
    float theta_dot_array[sample_num];
    for (int i = 0; i < sample_num; i++) {
        theta_dot_array[i] = get_theta_p_dot_deg(pi, bus);
        usleep(meas_interval);
    }

    // calculate mean
    theta_dot_mean = 0;
    for (int i = 0; i < sample_num; i++) {
        theta_dot_mean += theta_dot_array[i];
    }
    theta_dot_mean /= sample_num;

    // calculate variance
    float temp;
    theta_dot_variance = 0;
    for (int i = 0; i < sample_num; i++) {
        temp = theta_dot_array[i] - theta_dot_mean;
        theta_dot_variance += temp * temp;
    }
    theta_dot_variance /= sample_num;
    return;
}

//=========================================================
// Caluculate angle and angvel
//=========================================================
float get_theta_p_deg(int pi, int bus_acc) {
    get_acc_data(pi, bus_acc);
    float theta_p_deg = atan2(float(zAccl), float(yAccl)) * rad2deg;
    return theta_p_deg;
}

float get_theta_p_dot_deg(int pi, int bus_gyr) {
    get_gyr_data(pi, bus_gyr);
    float theta_p_dot_deg = -1 * xGyro; // !caution!
    return theta_p_dot_deg;
}