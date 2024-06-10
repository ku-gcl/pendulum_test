
#include "encoder.h"
#include "pigpiod_if2.h"
#include "config.h"

void rotary_encoder()
{
    if (enc_syn == 1)
    {
        static int code;
        code = ((code << 2) + (gpio_read(pi, pin2) << 1) + gpio_read(pi, pin1)) & 0xf;
        int value = -1 * table[code];
        encoder_value += value;
        std::chrono::microseconds dura1(rotary_encoder_update_rate);
        std::this_thread::sleep_for(dura1);
        return;
    }
}
