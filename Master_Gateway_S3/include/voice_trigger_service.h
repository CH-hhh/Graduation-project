#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>

namespace VoiceTriggerService {

void init();
void start();
void stop();
bool isEnabled();
void setEnabled(bool enabled);
void setCooldown(uint32_t ms);

// 🌟 纯本地离线声学关键词流式识别接口：
// 传入当前音频帧干净 PCM 采样点，内部实时提取 MFCC 声学特征并执行 DTW 音节共振峰模式匹配。
// 当本地芯片完全离线识别到“你好小鑫” (Ni-Hao-Xiao-Xin) 或“你好小乐”时，立即返回 true！
bool feedAudioFrame(const int16_t* samples, size_t sample_count);

} // namespace VoiceTriggerService
