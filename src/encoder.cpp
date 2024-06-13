
#include "encoder.h"
#include "pigpiod_if2.h"
#include "config.h"
#include <chrono>
#include <thread>

void rotary_encoder()
{
    if (enc_syn == 1)
    {
        static int code;
        code = ((code << 2) + (gpio_read(pi, pin2) << 1) + gpio_read(pi, pin1)) & 0xf;
        int value = -1 * table[code];
        encoder_value += value;
        std::this_thread::sleep_for(std::chrono::microseconds(encoder_update_rate));
        return;
    }
}
