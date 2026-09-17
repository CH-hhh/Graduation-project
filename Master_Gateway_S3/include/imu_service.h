#pragma once

#include <cstdint>

namespace ImuService {
bool begin();
bool readAcceleration(int16_t& x, int16_t& y, int16_t& z);
}
