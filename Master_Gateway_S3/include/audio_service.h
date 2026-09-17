#pragma once

#include <stddef.h>
#include <stdint.h>

namespace AudioService {

enum class AudioMode {
    MODE_NONE = 0,
    MODE_MUSIC,
    MODE_TTS
};

// 🌟 音乐播放专用初始化函数 (默认 44.1kHz 高保真音乐模式)
bool initForMusic(uint32_t sample_rate = 44100);

// 🌟 AI 文本转语音(TTS)专用初始化函数 (默认 24kHz 语音合成模式)
bool initForTTS(uint32_t sample_rate = 24000);

// 通用系统启动初始化
bool begin();

// 电源与时钟控制 (功放休眠/唤醒/DMA复位)
void wake();
void sleep();
void resetDma();

// 采样率设置与查询
void setSampleRate(uint32_t rate);
uint32_t getSampleRate();
AudioMode getCurrentMode();

}

