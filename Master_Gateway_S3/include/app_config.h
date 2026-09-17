#pragma once

#include <Arduino.h>

namespace AppConfig {
inline constexpr uint8_t DEFAULT_ROTATION = 0;

inline constexpr gpio_num_t JOY_VRX = GPIO_NUM_2;
inline constexpr gpio_num_t JOY_VRY = GPIO_NUM_1;
inline constexpr gpio_num_t JOY_SW = GPIO_NUM_42;
inline constexpr gpio_num_t RETURN_BUTTON = GPIO_NUM_18;
inline constexpr gpio_num_t BUZZER = GPIO_NUM_15;
inline constexpr gpio_num_t PASSIVE_BUZZER_PIN = GPIO_NUM_15; // 无源蜂鸣器(音乐播放, 与系统蜂鸣器共用15)
inline constexpr gpio_num_t STATUS_LED = GPIO_NUM_NC; // 用户未使用指示灯，已禁用以释放 39 号引脚给相机 SPI MISO
inline constexpr gpio_num_t TFT_BACKLIGHT = GPIO_NUM_9;

inline constexpr gpio_num_t I2S_DIN = GPIO_NUM_4;
inline constexpr gpio_num_t I2S_BCLK = GPIO_NUM_5;
inline constexpr gpio_num_t I2S_LRCK = GPIO_NUM_6;
inline constexpr gpio_num_t I2S_DOUT = GPIO_NUM_7;
inline constexpr gpio_num_t AMP_ENABLE = GPIO_NUM_NC;

inline constexpr gpio_num_t IR_RECV_PIN = GPIO_NUM_16;

inline constexpr gpio_num_t ICM_I2C_SDA = GPIO_NUM_41;
inline constexpr gpio_num_t ICM_I2C_SCL = GPIO_NUM_40;
inline constexpr uint8_t ICM_I2C_ADDRESS = 0x68;

inline constexpr uint32_t TTS_SAMPLE_RATE = 24000; // 文本转语音(TTS)标准 24kHz 采样率 (DashScope 原生)
inline constexpr uint32_t MUSIC_DEFAULT_SAMPLE_RATE = 44100; // 音乐播放原生 44.1kHz 采样率
inline constexpr uint32_t AUDIO_SAMPLE_RATE = 24000;
inline constexpr uint32_t MAX_RECORD_SIZE = 1 * 1024 * 1024; // 24kHz 16bit 单声道约可录 22 秒
inline constexpr size_t MAX_PCM_CACHE_SIZE = 2560 * 1024; // 2.5 MB PSRAM (刚好容纳 150 字 / 45 秒完整 24kHz 音频)
inline constexpr size_t MAX_RAW_CACHE_SIZE = 64 * 1024;

inline constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 8000;
inline constexpr uint32_t WIFI_RETRY_INTERVAL_MS = 30000;
inline constexpr uint32_t ENVIRONMENT_UPDATE_INTERVAL_MS = 5U * 60U * 1000U;
inline constexpr uint32_t ENVIRONMENT_QUERY_TIMEOUT_MS = 2000;
inline constexpr uint32_t HTTP_CONNECT_TIMEOUT_MS = 10000;
inline constexpr uint32_t HTTP_READ_TIMEOUT_MS = 35000;
inline constexpr uint32_t CLOUD_FIRST_BYTE_TIMEOUT_MS = 60000;

inline constexpr size_t EXPECTED_FLASH_BYTES = 16U * 1024U * 1024U;
inline constexpr size_t MINIMUM_PSRAM_BYTES = 4U * 1024U * 1024U;

inline constexpr char DASHSCOPE_HOST[] = "dashscope.aliyuncs.com";
inline constexpr uint16_t DASHSCOPE_PORT = 443;
inline constexpr char AMAP_HOST[] = "restapi.amap.com";

    // SPI Slave pins for STM32 Camera (物理引脚: 21->PB1 HANDSHAKE, 39->PB14 MISO)
    constexpr int STM32_SPI_CS = 47;
    constexpr int STM32_SPI_SCK = 38;
    constexpr int STM32_SPI_MISO = 39;
    constexpr int STM32_SPI_MOSI = 45;
    constexpr int STM32_SPI_HANDSHAKE = 21;

    // STM32 音乐播放器双向控制串口 UART1 (GPIO 0 对应 TX 发给 STM32 RX, GPIO 48 对应 RX 接收 STM32 TX)
    inline constexpr gpio_num_t MUSIC_UART_TX = GPIO_NUM_0;
    inline constexpr gpio_num_t MUSIC_UART_RX = GPIO_NUM_48;
}
