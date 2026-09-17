#pragma once

#include <Arduino.h>

namespace BuzzerService {
    void begin();
    void tone(uint16_t freq, uint16_t durMs);
    void stop();
    void update();
}
