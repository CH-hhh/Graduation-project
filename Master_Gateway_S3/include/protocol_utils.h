#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

constexpr size_t encodedBase64Size(size_t rawBytes) {
    return 4 * ((rawBytes + 2) / 3);
}

inline void buildPcmWavHeader(uint8_t* header, uint32_t pcmBytes, uint32_t sampleRate) {
    const uint32_t fileSize = pcmBytes + 36;
    const uint32_t byteRate = sampleRate * 2;
    const uint8_t value[44] = {
        'R', 'I', 'F', 'F', static_cast<uint8_t>(fileSize), static_cast<uint8_t>(fileSize >> 8), static_cast<uint8_t>(fileSize >> 16), static_cast<uint8_t>(fileSize >> 24),
        'W', 'A', 'V', 'E', 'f', 'm', 't', ' ', 16, 0, 0, 0, 1, 0, 1, 0,
        static_cast<uint8_t>(sampleRate), static_cast<uint8_t>(sampleRate >> 8), static_cast<uint8_t>(sampleRate >> 16), static_cast<uint8_t>(sampleRate >> 24),
        static_cast<uint8_t>(byteRate), static_cast<uint8_t>(byteRate >> 8), static_cast<uint8_t>(byteRate >> 16), static_cast<uint8_t>(byteRate >> 24),
        2, 0, 16, 0, 'd', 'a', 't', 'a',
        static_cast<uint8_t>(pcmBytes), static_cast<uint8_t>(pcmBytes >> 8), static_cast<uint8_t>(pcmBytes >> 16), static_cast<uint8_t>(pcmBytes >> 24)
    };
    memcpy(header, value, sizeof(value));
}
