#include "audio_service.h"

#include <Arduino.h>
#include <driver/i2s.h>

#include "app_config.h"
#include "debug_log_service.h"
#define Serial DebugLog

namespace AudioService {

static bool s_installed = false;
static AudioMode s_current_mode = AudioMode::MODE_NONE;
static uint32_t s_current_rate = 22050;

static bool installDriver(uint32_t sample_rate, size_t dma_buf_count = 8, size_t dma_buf_len = 512) {
    if (s_installed) {
        i2s_stop(I2S_NUM_0);
        i2s_driver_uninstall(I2S_NUM_0);
        s_installed = false;
    }

    i2s_config_t config = {
        .mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX),
        .sample_rate = sample_rate,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // 🌟 标准立体声帧 (64 BCLK/LRCK)，NS4168 功放硬件锁相环 100% 稳定锁定且发声！
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = static_cast<int>(dma_buf_count),
        .dma_buf_len = static_cast<int>(dma_buf_len),
        .use_apll = false, // 🌟 ESP32-S3 硬件无 APLL 外设，必须置 false 杜绝底层 Panic 重启！
        .tx_desc_auto_clear = true
    };
    i2s_pin_config_t pins = {
        .mck_io_num = I2S_PIN_NO_CHANGE,
        .bck_io_num = static_cast<int>(AppConfig::I2S_BCLK),
        .ws_io_num = static_cast<int>(AppConfig::I2S_LRCK),
        .data_out_num = static_cast<int>(AppConfig::I2S_DOUT),
        .data_in_num = static_cast<int>(AppConfig::I2S_DIN)
    };

    esp_err_t error = i2s_driver_install(I2S_NUM_0, &config, 0, nullptr);
    if (error != ESP_OK) {
        Serial.printf("[ERR] I2S 驱动安装失败: %s\r\n", esp_err_to_name(error));
        return false;
    }
    error = i2s_set_pin(I2S_NUM_0, &pins);
    if (error != ESP_OK) {
        Serial.printf("[ERR] I2S 引脚配置失败: %s\r\n", esp_err_to_name(error));
        i2s_driver_uninstall(I2S_NUM_0);
        return false;
    }
    error = i2s_zero_dma_buffer(I2S_NUM_0);
    if (error != ESP_OK) {
        Serial.printf("[ERR] I2S DMA 清零失败: %s\r\n", esp_err_to_name(error));
        i2s_driver_uninstall(I2S_NUM_0);
        return false;
    }
    i2s_start(I2S_NUM_0);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    s_installed = true;
    s_current_rate = sample_rate;
    return true;
}

// 🌟 音乐播放专用初始化 (44.1kHz 音乐模式)
bool initForMusic(uint32_t sample_rate) {
    if (sample_rate < 8000 || sample_rate > 96000) sample_rate = 44100;
    
    if (!s_installed || s_current_rate != sample_rate || s_current_mode != AudioMode::MODE_MUSIC) {
        if (!installDriver(sample_rate, 8, 512)) {
            return false;
        }
    }
    s_current_mode = AudioMode::MODE_MUSIC;
    Serial.printf("[SYS] 🎵 I2S 音乐播放器专用模式已激活 (%u Hz, CD级高保真时钟)\r\n", (unsigned)sample_rate);
    return true;
}

// 🌟 AI 文本转语音 (TTS) / 语音对话 / 离线唤醒专用初始化 (24kHz 全双工模式：播音 + 麦克风采集)
bool initForTTS(uint32_t sample_rate) {
    if (sample_rate < 8000 || sample_rate > 96000) sample_rate = 24000;
    
    if (!s_installed || s_current_rate != sample_rate || s_current_mode != AudioMode::MODE_TTS) {
        if (!installDriver(sample_rate, 8, 512)) {
            return false;
        }
    }
    s_current_mode = AudioMode::MODE_TTS;
    Serial.printf("[SYS] 🗣️ I2S 语音对话/录音/TTS全双工模式已激活 (%u Hz, 麦克风录音使能)\r\n", (unsigned)sample_rate);
    return true;
}

bool begin() {
    return initForTTS(24000);
}

void setSampleRate(uint32_t rate) {
    if (rate < 8000 || rate > 96000) return;
    if (rate == s_current_rate && s_installed) return;
    
    if (s_installed) {
        i2s_set_sample_rates(I2S_NUM_0, rate);
        s_current_rate = rate;
        Serial.printf("[SYS] 🎯 I2S 硬件高精度采样率已平滑锁定为: %u Hz\r\n", (unsigned)rate);
    } else {
        AudioMode currentMode = s_current_mode;
        installDriver(rate, 8, 512);
        s_current_mode = currentMode;
        wake();
        Serial.printf("[SYS] 🎯 I2S 硬件驱动已初始化并锁定为: %u Hz\r\n", (unsigned)rate);
    }
}

uint32_t getSampleRate() {
    return s_current_rate;
}

AudioMode getCurrentMode() {
    return s_current_mode;
}

void wake() {
    if (!s_installed) {
        begin();
    }
    i2s_start(I2S_NUM_0);
    i2s_zero_dma_buffer(I2S_NUM_0);
}

void resetDma() {
    if (s_installed) {
        i2s_zero_dma_buffer(I2S_NUM_0);
    }
}

void sleep() {
    if (s_installed) {
        i2s_stop(I2S_NUM_0);
    }
}

}

