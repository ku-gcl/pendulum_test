#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <pigpiod_if2.h>
#include <thread>
#include <unistd.h>

#define Addr_Accl 0x19 // BMX055 Accelerometer I2C address
#define Addr_Gyro 0x69 // BMX055 Gyroscope I2C address
#define Addr_Mag 0x13  // BMX055 Magnetometer I2C address

float rad2deg = 57.29578f;

float xAccl = 0.00, yAccl = 0.00, zAccl = 0.00;
float xGyro = 0.00, yGyro = 0.00, zGyro = 0.00;
int xMag = 0, yMag = 0, zMag = 0;

int pi, bus_acc, bus_gyr, bus_mag;

void BMX055_Init();
void BMX055_Accl();
void BMX055_Gyro();
void BMX055_Mag();

int main() {
    pi = pigpio_start(NULL, NULL);
    if (pi < 0) {
        std::cerr << "Failed to connect to pigpiod" << std::endl;
        return 1;
    }

    bus_acc = i2c_open(pi, 1, Addr_Accl, 0);
    bus_gyr = i2c_open(pi, 1, Addr_Gyro, 0);
    bus_mag = i2c_open(pi, 1, Addr_Mag, 0);

    if (bus_acc < 0 || bus_gyr < 0 || bus_mag < 0) {
        std::cerr << "Failed to open I2C bus" << std::endl;
        return 1;
    }

    BMX055_Init();

    while (true) {
        std::cout << "--------------------------------------" << std::endl;

        BMX055_Accl();
        std::cout << "Accl= " << xAccl << ", " << yAccl << ", " << zAccl
                  << std::endl;

        BMX055_Gyro();
        std::cout << "Gyro= " << xGyro << ", " << yGyro << ", " << zGyro
                  << std::endl;

        BMX055_Mag();
        std::cout << "Mag= " << xMag << ", " << yMag << ", " << zMag
                  << std::endl;

        float theta_p_deg = atan2(float(yAccl), float(zAccl)) * rad2deg;
        std::cout << "theta_p " << theta_p_deg << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    i2c_close(pi, bus_acc);
    i2c_close(pi, bus_gyr);
    i2c_close(pi, bus_mag);
    pigpio_stop(pi);

    return 0;
}

void BMX055_Init() {
    // Accelerometer initialization
    i2c_write_byte_data(pi, bus_acc, 0x0F, 0x03);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    i2c_write_byte_data(pi, bus_acc, 0x10, 0x08);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    i2c_write_byte_data(pi, bus_acc, 0x11, 0x00);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Gyroscope initialization
    i2c_write_byte_data(pi, bus_gyr, 0x0F, 0x04);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    i2c_write_byte_data(pi, bus_gyr, 0x10, 0x07);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    i2c_write_byte_data(pi, bus_gyr, 0x11, 0x00);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Magnetometer initialization
    i2c_write_byte_data(pi, bus_mag, 0x4B, 0x83);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    i2c_write_byte_data(pi, bus_mag, 0x4B, 0x01);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    i2c_write_byte_data(pi, bus_mag, 0x4C, 0x00);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    i2c_write_byte_data(pi, bus_mag, 0x4E, 0x84);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    i2c_write_byte_data(pi, bus_mag, 0x51, 0x04);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    i2c_write_byte_data(pi, bus_mag, 0x52, 0x10);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void BMX055_Accl() {
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

void BMX055_Gyro() {
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

    xGyro = x * 0.0038f; //  Full scale = +/- 125 degree/s
    yGyro = y * 0.0038f; //  Full scale = +/- 125 degree/s
    zGyro = z * 0.0038f; //  Full scale = +/- 125 degree/s
}

void BMX055_Mag() {
    char data[8];
    i2c_read_i2c_block_data(pi, bus_mag, 0x42, data, 8);

    int x = ((data[1] << 8) | data[0]) >> 3;
    if (x > 4095)
        x -= 8192;
    int y = ((data[3] << 8) | data[2]) >> 3;
    if (y > 4095)
        y -= 8192;
    int z = ((data[5] << 8) | data[4]) >> 1;
    if (z > 16383)
        z -= 32768;

    xMag = x;
    yMag = y;
    zMag = z;
}
