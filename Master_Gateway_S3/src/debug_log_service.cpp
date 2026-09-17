#include "debug_log_service.h"

#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <time.h>

namespace {
constexpr size_t PSRAM_LOG_BUFFER_BYTES = 32 * 1024;
constexpr size_t FALLBACK_LOG_BUFFER_BYTES = 512;
char fallbackLogBuffer[FALLBACK_LOG_BUFFER_BYTES] = {};
char* logBuffer = fallbackLogBuffer;
size_t logCapacity = FALLBACK_LOG_BUFFER_BYTES;
bool logUsesPsram = false;
size_t logHead = 0;
size_t logCount = 0;
size_t logNewlineCount = 0;
bool logAtLineStart = true;
volatile uint32_t logRevision = 0;
portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;

char logicalByte(size_t logicalIndex) {
    const size_t oldest = (logHead + logCapacity - logCount) % logCapacity;
    return logBuffer[(oldest + logicalIndex) % logCapacity];
}

void pushByteLocked(char value) {
    if (logCount == logCapacity && logBuffer[logHead] == '\n' &&
        logNewlineCount > 0) {
        --logNewlineCount;
    }
    logBuffer[logHead] = value;
    logHead = (logHead + 1) % logCapacity;
    if (logCount < logCapacity) ++logCount;
    if (value == '\n') ++logNewlineCount;
}

void buildTimePrefix(char* output, size_t outputSize) {
    const time_t now = time(nullptr);
    if (now >= 1700000000) {
        struct tm localTime;
        localtime_r(&now, &localTime);
        snprintf(output, outputSize, "[%02d:%02d] ",
                 localTime.tm_hour, localTime.tm_min);
        return;
    }

    const uint32_t uptimeSeconds = millis() / 1000;
    snprintf(output, outputSize, "[%02lu:%02lu] ",
             static_cast<unsigned long>((uptimeSeconds / 3600) % 100),
             static_cast<unsigned long>((uptimeSeconds / 60) % 60));
}

void appendToBuffer(const uint8_t* data, size_t size) {
    if (!data || size == 0) return;

    char timePrefix[12];
    buildTimePrefix(timePrefix, sizeof(timePrefix));

    portENTER_CRITICAL(&logMux);
    for (size_t i = 0; i < size; ++i) {
        const char value = static_cast<char>(data[i]);
        if (value == '\r') continue;

        if (logAtLineStart && value != '\n') {
            for (size_t prefixIndex = 0; timePrefix[prefixIndex] != '\0';
                 ++prefixIndex) {
                pushByteLocked(timePrefix[prefixIndex]);
            }
            logAtLineStart = false;
        }
        pushByteLocked(value);
        if (value == '\n') logAtLineStart = true;
    }
    ++logRevision;
    portEXIT_CRITICAL(&logMux);
}
}

DebugLogPrint DebugLog;

void DebugLogPrint::begin(unsigned long baud) {
    Serial.begin(baud);

    char* psramBuffer = static_cast<char*>(
        heap_caps_malloc(PSRAM_LOG_BUFFER_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));

    portENTER_CRITICAL(&logMux);
    if (psramBuffer) {
        logBuffer = psramBuffer;
        logCapacity = PSRAM_LOG_BUFFER_BYTES;
        logUsesPsram = true;
    } else {
        logBuffer = fallbackLogBuffer;
        logCapacity = FALLBACK_LOG_BUFFER_BYTES;
        logUsesPsram = false;
    }
    logHead = 0;
    logCount = 0;
    logNewlineCount = 0;
    logAtLineStart = true;
    logRevision = 0;
    portEXIT_CRITICAL(&logMux);
}

size_t DebugLogPrint::write(uint8_t value) {
    const size_t written = Serial.write(value);
    appendToBuffer(&value, 1);
    return written;
}

size_t DebugLogPrint::write(const uint8_t* buffer, size_t size) {
    const size_t written = Serial.write(buffer, size);
    appendToBuffer(buffer, size);
    return written;
}

int DebugLogPrint::available() {
    return Serial.available();
}

int DebugLogPrint::read() {
    return Serial.read();
}

int DebugLogPrint::peek() {
    return Serial.peek();
}

void DebugLogPrint::flush() {
    Serial.flush();
}

namespace DebugLogService {

uint32_t revision() {
    return logRevision;
}

size_t lineCount() {
    portENTER_CRITICAL(&logMux);
    size_t lines = logNewlineCount;
    if (logCount > 0 && logicalByte(logCount - 1) != '\n') ++lines;
    portEXIT_CRITICAL(&logMux);
    return lines;
}

size_t capacity() {
    return logCapacity;
}

bool usingPsram() {
    return logUsesPsram;
}

size_t copyWindow(char* output, size_t outputSize, size_t linesFromEnd) {
    if (!output || outputSize == 0) return 0;

    portENTER_CRITICAL(&logMux);
    size_t end = logCount;
    for (size_t line = 0; line < linesFromEnd && end > 0; ++line) {
        while (end > 0 && logicalByte(end - 1) == '\n') --end;
        while (end > 0 && logicalByte(end - 1) != '\n') --end;
    }

    size_t bytesToCopy = end;
    if (bytesToCopy >= outputSize) bytesToCopy = outputSize - 1;
    size_t startLogical = end - bytesToCopy;

    // Avoid beginning the visible tail in the middle of a UTF-8 character.
    while (bytesToCopy > 0 &&
           (static_cast<uint8_t>(logicalByte(startLogical)) & 0xC0) == 0x80) {
        ++startLogical;
        --bytesToCopy;
    }

    for (size_t i = 0; i < bytesToCopy; ++i) {
        output[i] = logicalByte(startLogical + i);
    }
    output[bytesToCopy] = '\0';
    portEXIT_CRITICAL(&logMux);
    return bytesToCopy;
}

size_t copyTail(char* output, size_t outputSize) {
    return copyWindow(output, outputSize, 0);
}

}
