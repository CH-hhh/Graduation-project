#pragma once

#include <Arduino.h>

namespace DeviceActionService {

struct SystemDeviceState {
    // 💡 智能家居与执行器
    bool light_on;              // 智能灯状态 (true:开, false:关)
    bool flash_on;              // 摄像头补光灯 (true:开, false:关)

    // 🎵 音乐播放器状态
    uint8_t music_state;        // 0:停止, 1:播放中, 2:暂停
    uint8_t music_volume;       // 音量 (0~100)
    int current_track_idx;      // 当前曲目索引 (0-based)
    String current_song_title;  // 当前歌名与歌手
    uint8_t music_mode;         // 0:顺序, 1:单曲, 2:随机

    // 📱 系统设置与 UI
    uint8_t screen_brightness;  // 屏幕背光亮度 (10~255)
    uint8_t current_page;       // 当前所在页面 (0~8)
    bool sound_muted;           // 系统提示音静音状态

    // 🌡️ 传感器实时缓存快照
    float indoor_temp;          // 室内温度 (°C)
    float indoor_hum;           // 室内湿度 (%)
    float indoor_press;         // 大气压 (hPa)
    float indoor_lux;           // 光照强度 (Lux)
};

struct ActionResult {
    bool success;
    bool state_unchanged;       // 幂等标记: 若当前已是目标状态则为 true
    String message;             // 返回给大模型或日志的执行结果描述
};

enum class MusicCmd {
    PLAY = 0,
    PAUSE,
    TOGGLE_PLAY_PAUSE,
    NEXT,
    PREV,
    STOP,
    TOGGLE_MODE
};

// ----------------- 初始化与状态同步 -----------------
void init();
void syncCurrentStates();
const SystemDeviceState& getState();

// ----------------- 统一原子能力 Action 接口 (GUI 与 AI 双端同源调用) -----------------
ActionResult Action_SetLight(bool on, bool force = false);
ActionResult Action_ControlMusic(MusicCmd cmd, int param = 0);
ActionResult Action_PlayTrackByName(const String& songName);
ActionResult Action_SetVolume(int vol);
ActionResult Action_SetBrightness(int level);
ActionResult Action_SwitchPage(int pageIndex);
ActionResult Action_SetFlash(bool on);
ActionResult Action_ScanDevices();
ActionResult Action_ReadEnvironment();
ActionResult Action_ReadTempProbe();
ActionResult Action_ReadAmbientLight();
ActionResult Action_SendIRKey(int keyIndex, int profile = 0);

// ----------------- 传感器与子设备异步状态回写 -----------------
void Action_UpdateEnvironment(float temp, float hum, float press);
void Action_UpdateLightLux(float lux);

// ----------------- 大模型 System Prompt 状态上下文生成 -----------------
String Action_BuildSystemPromptStateContext();

// ----------------- 语音交互焦点与待开播控制 -----------------
void setPendingVoiceTrack(int trackId);
int getPendingVoiceTrack();
void clearPendingVoiceTrack();
bool isVoiceInterrupted();
void setVoiceInterrupted(bool interrupted);

} // namespace DeviceActionService
