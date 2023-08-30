#include <pigpiod_if2.h>
#include <unistd.h>
#include <iostream>

int IN1 = 6; //31
int IN2 = 5; //29
int motor_pwm1 = 12; // Motor driver PWM input 32
int motor_pwm2 = 13; // Motor driver PWM input 32

int pi;

int main() {
    // Initialize pigpio
    pi = pigpio_start(NULL,NULL);

    // Set GPIO pin modes
    set_mode(pi, IN1, PI_OUTPUT);
    set_mode(pi, IN2, PI_OUTPUT);
    set_mode(pi, motor_pwm1, PI_OUTPUT);
    set_mode(pi, motor_pwm2, PI_OUTPUT);

    // Set PWM frequency
    //int pwm_frequency = 1; // 10 kHz
    // int pwm_duty_cycle=255;
    int pwm_duty_cycle=1;
    int pwm_range=255;
    // int pwm_freq =10000;
    int pwm_freq =100;

    gpio_write(pi, IN1, 0);
    gpio_write(pi, IN2, 0);
    // usleep(60000);  
    sleep(2);
    
    set_PWM_frequency(pi, motor_pwm1, pwm_freq);
    set_PWM_range(pi, motor_pwm1, pwm_range);
    int freq = get_PWM_frequency(pi, motor_pwm1);

    set_PWM_frequency(pi, motor_pwm2, pwm_freq);
    set_PWM_range(pi, motor_pwm2, pwm_range);
    // Motor control loop

    // pwm_duty_cycle=0;
    set_PWM_dutycycle(pi, motor_pwm1, pwm_duty_cycle);
    set_PWM_dutycycle(pi, motor_pwm2, pwm_duty_cycle);
        
    gpio_write(pi, IN1, 0);
    gpio_write(pi, IN2, 1);
    // usleep(60000);  
    sleep(60);

    gpio_write(pi, IN1, 0);
    gpio_write(pi, IN2, 0);
    // usleep(60000);  
    sleep(1);

    gpio_write(pi, IN1, 1);
    gpio_write(pi, IN2, 0);
    // usleep(60000);  
    sleep(60);
    
    // Cleanup
    set_PWM_dutycycle(pi, motor_pwm1, 0);
    set_PWM_dutycycle(pi, motor_pwm2, 0);
    gpio_write(pi, IN1, 0);
    gpio_write(pi, IN2, 0);
    pigpio_stop(pi);

    return 0;
}
