#include "motor_control.h"
#include "pigpiod_if2.h"
#include "config.h"

void motor_driver_init(int pi)
{
    gpio_write(pi, IN1, 0);
    gpio_write(pi, IN2, 0);
    set_PWM_frequency(pi, PWM, 10000);
    set_PWM_range(pi, PWM, 100);
    set_PWM_dutycycle(pi, PWM, 0);
}

void motor_control_update()
{
    // モーター制御の更新処理
}
