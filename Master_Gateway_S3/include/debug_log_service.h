#pragma once

#include <Arduino.h>

class DebugLogPrint final : public Stream {
public:
    void begin(unsigned long baud);
    size_t write(uint8_t value) override;
    size_t write(const uint8_t* buffer, size_t size) override;
    int available() override;
    int read() override;
    int peek() override;
    void flush() override;
};

extern DebugLogPrint DebugLog;

namespace DebugLogService {
uint32_t revision();
size_t lineCount();
size_t capacity();
bool usingPsram();
size_t copyWindow(char* output, size_t outputSize, size_t linesFromEnd);
size_t copyTail(char* output, size_t outputSize);
}
