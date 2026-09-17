#include "hardware_validation.h"

#include <Arduino.h>

#include "app_config.h"
#include "debug_log_service.h"
#define Serial DebugLog

namespace HardwareValidation {
bool validateMemoryConfiguration() {
    const size_t flashBytes = ESP.getFlashChipSize();
    const size_t psramBytes = ESP.getPsramSize();
    const bool flashOk = flashBytes >= AppConfig::EXPECTED_FLASH_BYTES;
    const bool psramOk = psramFound() && psramBytes >= AppConfig::MINIMUM_PSRAM_BYTES;

    Serial.printf("[SYS] Flash: %uMB | PSRAM: %uMB\r\n",
                  static_cast<unsigned>(flashBytes / 1024 / 1024),
                  static_cast<unsigned>(psramBytes / 1024 / 1024));
    if (!flashOk) Serial.println("[ERR] Flash容量不足");
    if (!psramOk) Serial.println("[ERR] PSRAM容量不足");
    return flashOk && psramOk;
}
}
