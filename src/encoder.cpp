
#include "encoder.h"
#include "config.h"
#include "pigpiod_if2.h"
#include <chrono>
#include <thread>

#include <iostream>

void rotary_encoder() {
    while (true) {
        if (enc_syn == 1) {
            // TODO: check code is global or local
            // static int code;
            code = ((code << 2) + (gpio_read(pi, pin2) << 1) +
                    gpio_read(pi, pin1)) &
                   0xf;
            int value = -1 * table[code];
            encoder_value += value;
            // TODO: コメント削除
            // デバッグ出力
            // std::cout << "code: " << code << " value: " << value << "
            // encoder_value: " << encoder_value << std::endl;
        }
        std::this_thread::sleep_for(
            std::chrono::microseconds(encoder_update_rate));
    }
}
