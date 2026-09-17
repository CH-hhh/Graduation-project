#include "buzzer_service.h"
#include "app_config.h"

namespace BuzzerService {
namespace {
constexpr uint8_t LEDC_CH = 1;        // not shared with analogWrite backlight
constexpr uint8_t LEDC_RES = 10;      // 10-bit resolution
constexpr uint16_t DUTY_50 = 512;     // ~50% duty

bool active = false;
uint32_t last_note_ms = 0;
uint32_t note_duration_ms = 600;

void setFreq(uint16_t freq) {
    if (freq == 0) {
        ledcWrite(LEDC_CH, 0);
    } else {
        if (ledcChangeFrequency(LEDC_CH, freq, LEDC_RES) == 0) {
            ledcSetup(LEDC_CH, freq, LEDC_RES);
        }
        ledcWrite(LEDC_CH, DUTY_50);
    }
}
} // namespace

void begin() {
    ledcSetup(LEDC_CH, 440, LEDC_RES);
    ledcAttachPin(static_cast<int>(AppConfig::PASSIVE_BUZZER_PIN), LEDC_CH);
    ledcWrite(LEDC_CH, 0);
    active = false;
}

void tone(uint16_t freq, uint16_t durMs) {
    setFreq(freq);
    last_note_ms = millis();
    note_duration_ms = durMs ? durMs : 600;
    active = true;
}

void stop() {
    setFreq(0);
    active = false;
}

void update() {
    if (active && (millis() - last_note_ms >= note_duration_ms)) {
        setFreq(0);
        active = false;
    }
}
} // namespace BuzzerService
