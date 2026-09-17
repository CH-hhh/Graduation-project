#pragma once

#include <Arduino.h>
#include <vector>
#include "app_config.h"

namespace MusicPlayerService {

struct SongItem {
    int id;
    String title;
    String artist;
    int durationSec;
};

enum PlayState {
    STATE_STOPPED = 0,
    STATE_PLAYING = 1,
    STATE_PAUSED  = 2
};

enum PlayMode {
    MODE_SEQUENCE      = 0, // 顺序播放
    MODE_SINGLE_REPEAT = 1, // 单曲循环
    MODE_RANDOM        = 2  // 随机播放
};

// 初始化与周期性串口轮询
void begin();
void update();

// 播放器状态读取
PlayState getPlayState();
bool isPlaying();
bool isPaused();
int getCurrentTrackIndex();
const SongItem* getCurrentSong();
int getCurrentSeconds();
int getTotalSeconds();
int getVolume();
PlayMode getPlayMode();
const char* getPlayModeName();
String getCurrentLyric();
void getLyrics3Lines(String& outPrev, String& outCurr, String& outNext);
void getLyrics3LinesForTime(int seconds, String& outPrev, String& outCurr, String& outNext);
void setMuteState(bool mute);
const std::vector<SongItem>& getPlaylist();
int getPlaylistCount();
const SongItem* getSong(int index);

// 频谱动效数据获取 (16 根柱状条 0~100 与 16 根峰值浮顶 0~100)
void getSpectrumBars(uint8_t outBars[16]);
void getSpectrumPeaks(uint8_t outPeaks[16]);

// 连接状态
bool isConnected();
void sendConnect();
void sendDisconnect();

// 前端控制指令下发 (通过 UART1 发送给 STM32)
void sendPlay(int trackId);
void sendTogglePlayPause();
void sendPause();
void sendResume();
void sendStop();
void sendNext();
void sendPrev();
void sendSeek(int seconds);
void sendVolume(int vol);
void sendTogglePlayMode();
void sendRequestPlaylist();
void sendRawCommand(const String& cmd);

// 暂停记录点查询
int getSavedPauseSeconds();

// 🌟 扬声器实时参考信号提取 (供 AEC 声学降噪算法消除回声)
bool getPlayingReferenceSamples(int16_t out_samples[256]);

// 模拟本地默认演示歌单 (在未连上 STM32 SD 卡时提供演示交互)
void loadMockPlaylistIfEmpty();

} // namespace MusicPlayerService
