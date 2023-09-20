#include "pigpiod_if2.h"
#include <cmath>
#include <unistd.h>
#include <iostream>

const int BNO055_ADDR=0x28;

const int PAGE_ID_ADDR=0x07;

int pi;

// page 1 
// G-range: xxxxx00b (+/-2g), Bandwidth: xx111xxb(1000Hz), Operation mode: 000xxxxxb(Normal)
const int ACC_CONFIG_ADDR=0x08;
// deg/s-range: xxx001b (+/-1000deg/s), Bandwidth: 010xxxb(116Hz)
const int GYRO_CONFIG_ADDR_1=0x0A;
// Operation mode: 000b(Normal)
const int GYRO_CONFIG_ADDR_2=0x0B;
// page 0
const int ACC_Y_L=0x0A;  //ay<7:0>
// const int ACC_Y_H=0x0B;  //ay<15:8>
// const int ACC_Z_L=0x0C;  //az<7:0>
// const int ACC_Z_H=0x0D;  //az<15:8>
const int GYR_X_L=0x14;  //GYR_X<7:0>
// const int GYR_X_H=0x15;  //GYR_X<15:8>

float get_accl_data(int bus) {
    float theta1_deg =0;
    unsigned char data[4];
    i2c_write_byte_data(pi,bus, PAGE_ID_ADDR, 0);
    i2c_read_i2c_block_data(pi,bus, ACC_Y_L, (char *)data, 4);
    
    int y_data = (data[0] & 0xFF) + (data[1] * 256);
    if (y_data > 32767) {
        y_data -= 65536;
    }

    int z_data = (data[2] & 0xF0) + (data[3] * 256);
    if (z_data > 32767) {
        z_data -= 65536;
    }

    theta1_deg = atan2( float(z_data),float(y_data)) * 57.29578f;
    return theta1_deg;
}

int main(int argc, char **argv) {
    pi=pigpio_start(NULL,NULL);
    
    //I2C setting
    int bus = i2c_open(pi,0,BNO055_ADDR,0);
    i2c_write_byte_data(pi,bus, PAGE_ID_ADDR, 1);
    // G-range: xxxxx00b (+/-2g), Bandwidth: xx111xxb(1000Hz), Operation mode: 000xxxxxb(Normal)
    i2c_write_byte_data(pi,bus, ACC_CONFIG_ADDR, 0b00011100);
    i2c_write_byte_data(pi,bus, PAGE_ID_ADDR, 0);

    while (true) {
        float theta_deg = get_accl_data(bus);
        std::cout << "theta = " << theta_deg << " degrees" << std::endl;
        sleep(1);  // 0.1 seconds delay
    }

    i2c_close(pi,bus);
    pigpio_stop(pi);
    return 0;
}