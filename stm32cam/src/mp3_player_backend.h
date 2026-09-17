#pragma once

#include <Arduino.h>
#include <vector>

namespace Mp3PlayerBackend {

struct SongItem {
    int id;
    String fileName;
    String title;
    String artist;
    int durationSec;
    uint32_t startClust;
    uint32_t fileSize;
    uint32_t id3Size;
};

struct LyricItem {
    uint32_t timeMs;
    String text;
};

enum PlayState {
    STATE_STOPPED = 0,
    STATE_PLAYING = 1,
    STATE_PAUSED  = 2
};

enum PlayMode {
    MODE_SEQUENCE      = 0,
    MODE_SINGLE_REPEAT = 1,
    MODE_RANDOM        = 2
};

// 初始化与周期性轮询
void begin();
void update();

// 播放核心控制
void playTrack(int trackId);
void togglePlayPause();
void pause();
void resume();
void stop();
void next();
void prev();
void seek(int seconds);
void setVolume(int vol);
void setPlayMode(PlayMode mode);
void togglePlayMode();
void refreshPlaylist();

// 状态读取
PlayState getPlayState();
int getCurrentTrack();
int getCurrentSeconds();
int getTotalSeconds();
int getVolume();
PlayMode getPlayMode();
const std::vector<SongItem>& getPlaylist();
int getPlaylistCount();
const SongItem* getCurrentSong();

} // namespace Mp3PlayerBackend
