#include "device_action_service.h"

#include "music_player_service.h"
#include "camera_service.h"
#include "custom_ui_engine.h"
#include "ir_service.h"
#include "debug_log_service.h"
#include "app_config.h"

#define Serial DebugLog

// 外部在 main.cpp 中定义的 RPC 与传感器接口
extern void sendRpcCommand(uint8_t dev_id, uint8_t action, uint8_t val, uint8_t retries = 0);
extern volatile bool ack_received;
extern volatile bool ack_success;
extern volatile float ack_temperature_c;
extern volatile float ack_dht_hum;
extern volatile float ack_pressure_hpa;
extern volatile float ack_light_lux;
extern void skillScanDevices();
extern bool queryEnvironmentSensor(uint32_t timeoutMs = AppConfig::ENVIRONMENT_QUERY_TIMEOUT_MS);
extern String environmentReadingText();
extern bool queryTempProbeSensor(uint32_t timeoutMs = AppConfig::ENVIRONMENT_QUERY_TIMEOUT_MS);
extern String tempProbeReadingText();
extern bool queryAmbientLightSensor(uint32_t timeoutMs = AppConfig::ENVIRONMENT_QUERY_TIMEOUT_MS);
extern String ambientLightReadingText();
extern void beep(int ms, int count);

namespace DeviceActionService {

static SystemDeviceState s_state = {
    .light_on = false,
    .flash_on = false,
    .music_state = 0,
    .music_volume = 80,
    .current_track_idx = 0,
    .current_song_title = "暂无歌曲",
    .music_mode = 0,
    .screen_brightness = 255,
    .current_page = 0,
    .sound_muted = false,
    .indoor_temp = 0.0f,
    .indoor_hum = 0.0f,
    .indoor_press = 0.0f,
    .indoor_lux = 0.0f
};

void init() {
    syncCurrentStates();
    Serial.println("🌐 [ACTION-CENTER] 统一原子能力中台与全局状态孪生表已初始化就绪！");
}

void syncCurrentStates() {
    s_state.flash_on = CameraService::isFlashOn();
    s_state.music_state = static_cast<uint8_t>(MusicPlayerService::getPlayState());
    s_state.music_volume = static_cast<uint8_t>(MusicPlayerService::getVolume());
    s_state.current_track_idx = MusicPlayerService::getCurrentTrackIndex();
    const MusicPlayerService::SongItem* song = MusicPlayerService::getCurrentSong();
    if (song) {
        s_state.current_song_title = song->title + (song->artist.length() ? (" - " + song->artist) : "");
    }
    s_state.music_mode = static_cast<uint8_t>(MusicPlayerService::getPlayMode());
    s_state.screen_brightness = static_cast<uint8_t>(CustomUiEngine::getScreenBrightness());
    s_state.current_page = static_cast<uint8_t>(CustomUiEngine::getCurrentPage());
    s_state.sound_muted = !CustomUiEngine::getSystemSoundEnabled();

    if (ack_temperature_c > 0.0f) s_state.indoor_temp = ack_temperature_c;
    if (ack_dht_hum > 0.0f) s_state.indoor_hum = ack_dht_hum;
    if (ack_pressure_hpa > 0.0f) s_state.indoor_press = ack_pressure_hpa;
    if (ack_light_lux >= 0.0f) s_state.indoor_lux = ack_light_lux;
}

const SystemDeviceState& getState() {
    syncCurrentStates();
    return s_state;
}

// ----------------- 统一原子 Action 接口实现 -----------------

ActionResult Action_SetLight(bool on, bool force) {
    if (!force && s_state.light_on == on) {
        Serial.printf("ℹ️ [ACTION] 智能灯状态已是 %s，幂等拦截重复发射\r\n", on ? "开启" : "关闭");
        return { true, true, on ? "智能灯当前已经是开启状态" : "智能灯当前已经是关闭状态" };
    }

    ack_received = false;
    sendRpcCommand(2, 0x01, on ? 1 : 0);

    uint32_t wait_start = millis();
    bool is_timeout = true;
    while (millis() - wait_start < 1500) {
        if (ack_received) {
            is_timeout = false;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (is_timeout) {
        return { false, false, "控制智能灯超时，从机设备可能离线" };
    }

    s_state.light_on = on;
    CustomUiEngine::notifyUiNeedsUpdate();
    return { true, false, on ? "已为您打开智能灯" : "已为您关闭智能灯" };
}

static int s_pending_voice_track = -1;
static bool s_voice_interrupted_music = false;

void setPendingVoiceTrack(int trackId) { s_pending_voice_track = trackId; }
int getPendingVoiceTrack() { return s_pending_voice_track; }
void clearPendingVoiceTrack() { s_pending_voice_track = -1; }
bool isVoiceInterrupted() { return s_voice_interrupted_music; }
void setVoiceInterrupted(bool interrupted) { s_voice_interrupted_music = interrupted; }

ActionResult Action_ControlMusic(MusicCmd cmd, int param) {
    syncCurrentStates();

    switch (cmd) {
        case MusicCmd::PLAY:
            if (param >= 0 && param < MusicPlayerService::getPlaylistCount()) {
                s_pending_voice_track = param;
                s_voice_interrupted_music = false;
                s_state.music_state = 1;
                CustomUiEngine::setCurrentPage(PAGE_MUSIC);
                CustomUiEngine::notifyUiNeedsUpdate();
                return { true, false, "已锁定指定曲目，即将在语音播报后为您播放" };
            } else {
                if (s_state.music_state == 1) {
                    return { true, true, "音乐当前正在播放中" };
                }
                s_pending_voice_track = -1;
                s_voice_interrupted_music = true; // 标记语音播报完继续播放
                s_state.music_state = 1;
                CustomUiEngine::notifyUiNeedsUpdate();
                return { true, false, "好的，即将在语音播报后为您继续播放音乐" };
            }

        case MusicCmd::PAUSE:
            MusicPlayerService::sendPause();
            s_pending_voice_track = -1;
            s_voice_interrupted_music = false; // 用户明确要求暂停，绝不恢复
            s_state.music_state = 2;
            CustomUiEngine::notifyUiNeedsUpdate();
            return { true, false, "已为您暂停音乐播放" };

        case MusicCmd::TOGGLE_PLAY_PAUSE:
            MusicPlayerService::sendTogglePlayPause();
            s_pending_voice_track = -1;
            s_voice_interrupted_music = false;
            syncCurrentStates();
            CustomUiEngine::notifyUiNeedsUpdate();
            return { true, false, s_state.music_state == 1 ? "已恢复播放" : "已暂停播放" };

        case MusicCmd::NEXT:
            MusicPlayerService::sendNext();
            s_pending_voice_track = -1;
            s_voice_interrupted_music = false;
            syncCurrentStates();
            CustomUiEngine::notifyUiNeedsUpdate();
            return { true, false, "已为您切换至下一首歌曲" };

        case MusicCmd::PREV:
            MusicPlayerService::sendPrev();
            s_pending_voice_track = -1;
            s_voice_interrupted_music = false;
            syncCurrentStates();
            CustomUiEngine::notifyUiNeedsUpdate();
            return { true, false, "已为您切换至上一首歌曲" };

        case MusicCmd::STOP:
            MusicPlayerService::sendStop();
            s_pending_voice_track = -1;
            s_voice_interrupted_music = false;
            s_state.music_state = 0;
            CustomUiEngine::notifyUiNeedsUpdate();
            return { true, false, "已停止音乐播放" };

        case MusicCmd::TOGGLE_MODE:
            MusicPlayerService::sendTogglePlayMode();
            syncCurrentStates();
            CustomUiEngine::notifyUiNeedsUpdate();
            return { true, false, "已切换播放模式为：" + String(MusicPlayerService::getPlayModeName()) };

        default:
            return { false, false, "未知音乐控制指令" };
    }
}

ActionResult Action_PlayTrackByName(const String& songName) {
    if (songName.length() == 0) return { false, false, "歌曲名称不能为空" };

    const auto& list = MusicPlayerService::getPlaylist();
    if (list.empty()) return { false, false, "当前歌单为空，请确认 SD 卡已正确插入且已同步歌单" };

    String target = songName;
    target.toLowerCase();
    target.trim();
    // 过滤可能包含的书名号、引号等标点
    target.replace("《", "");
    target.replace("》", "");
    target.replace("\"", "");
    target.replace("'", "");
    target.replace("“", "");
    target.replace("”", "");
    target.trim();

    int matched_idx = -1;
    String matched_title = "";
    String matched_artist = "";

    // 1. 精确匹配 (歌名完全一致)
    for (size_t i = 0; i < list.size(); i++) {
        String t = list[i].title;
        t.toLowerCase();
        t.trim();
        t.replace("《", "");
        t.replace("》", "");
        t.trim();
        if (t == target) {
            matched_idx = list[i].id;
            matched_title = list[i].title;
            matched_artist = list[i].artist;
            break;
        }
    }

    // 2. 模糊子串双向包含匹配 (支持匹配歌名或歌手)
    if (matched_idx < 0) {
        for (size_t i = 0; i < list.size(); i++) {
            String t = list[i].title;
            t.toLowerCase();
            String a = list[i].artist;
            a.toLowerCase();
            if (t.indexOf(target) != -1 || target.indexOf(t) != -1 || 
                (a.length() > 0 && a != "未知歌手" && (a.indexOf(target) != -1 || target.indexOf(a) != -1))) {
                matched_idx = list[i].id;
                matched_title = list[i].title;
                matched_artist = list[i].artist;
                break;
            }
        }
    }

    if (matched_idx >= 0) {
        // 🌟 核心技能设计：记录待播曲目，待当前轮次语音播报完毕后无缝触发开播
        s_pending_voice_track = matched_idx;
        s_voice_interrupted_music = false;
        s_state.current_track_idx = matched_idx;
        s_state.music_state = 1;
        CustomUiEngine::setCurrentPage(PAGE_MUSIC); // 自动切至音乐播放页
        CustomUiEngine::notifyUiNeedsUpdate();
        String msg = "已为您匹配到歌曲：《" + matched_title + "》";
        if (matched_artist.length() > 0 && matched_artist != "未知歌手") {
            msg += " - " + matched_artist;
        }
        msg += "，即将为您播放";
        return { true, false, msg };
    }

    // 3. 未找到时，列出可用歌单推荐给大模型做自然语言回复
    String availableList = "";
    size_t showCount = min((size_t)5, list.size());
    for (size_t i = 0; i < showCount; i++) {
        availableList += "《" + list[i].title + "》";
        if (i < showCount - 1) availableList += "、";
    }
    if (list.size() > 5) availableList += " 等共 " + String(list.size()) + " 首";

    return { false, false, "歌单中未找到歌曲《" + songName + "》。当前 SD 卡可用歌曲有：" + availableList };
}

ActionResult Action_SetVolume(int vol) {
    if (vol < 0) vol = 0;
    if (vol > 100) vol = 100;

    if (s_state.music_volume == vol) {
        return { true, true, "音量当前已经是 " + String(vol) + "%" };
    }

    MusicPlayerService::sendVolume(vol);
    s_state.music_volume = vol;
    CustomUiEngine::notifyUiNeedsUpdate();
    return { true, false, "已将音乐音量设置为 " + String(vol) + "%" };
}

ActionResult Action_SetBrightness(int level) {
    if (level < 10) level = 10;
    if (level > 255) level = 255;

    // 若传入的是百分比 (10~100) 且 <= 100，自动映射为 PWM 范围 (10~255)
    int pwm_val = (level <= 100 && level >= 10) ? (level * 255 / 100) : level;
    if (pwm_val < 10) pwm_val = 10;
    if (pwm_val > 255) pwm_val = 255;

    if (s_state.screen_brightness == pwm_val) {
        return { true, true, "屏幕亮度当前已经是 " + String((pwm_val * 100) / 255) + "%" };
    }

    CustomUiEngine::setScreenBrightness(pwm_val);
    s_state.screen_brightness = pwm_val;
    CustomUiEngine::notifyUiNeedsUpdate();
    return { true, false, "已将屏幕亮度设置为 " + String((pwm_val * 100) / 255) + "%" };
}

ActionResult Action_SwitchPage(int pageIndex) {
    if (pageIndex < 0 || pageIndex >= PAGE_COUNT) {
        return { false, false, "目标页面编号不存在" };
    }

    if (CustomUiEngine::getCurrentPage() == pageIndex) {
        return { true, true, "当前已经在该页面" };
    }

    CustomUiEngine::setCurrentPage(pageIndex);
    s_state.current_page = pageIndex;
    CustomUiEngine::notifyUiNeedsUpdate();
    return { true, false, "已为您切换至目标界面" };
}

ActionResult Action_SetFlash(bool on) {
    if (CameraService::isFlashOn() == on) {
        return { true, true, on ? "补光灯当前已经是开启状态" : "补光灯当前已经是关闭状态" };
    }

    CameraService::toggleFlash();
    s_state.flash_on = on;
    CustomUiEngine::notifyUiNeedsUpdate();
    return { true, false, on ? "已开启摄像头补光灯" : "已关闭摄像头补光灯" };
}

ActionResult Action_ScanDevices() {
    skillScanDevices();
    return { true, false, "局域网在线设备列表：" + CustomUiEngine::online_devices_list };
}

ActionResult Action_ReadEnvironment() {
    bool ok = queryEnvironmentSensor();
    if (ok) {
        syncCurrentStates();
        return { true, false, environmentReadingText() };
    }
    return { false, false, "读取环境传感器失败或被控端无响应" };
}

ActionResult Action_ReadTempProbe() {
    bool ok = queryTempProbeSensor();
    if (ok) {
        syncCurrentStates();
        return { true, false, "外置温湿度探针数据：" + tempProbeReadingText() };
    }
    return { false, false, "读取外置温湿度探针失败：从机设备未响应或探针离线" };
}

ActionResult Action_ReadAmbientLight() {
    bool ok = queryAmbientLightSensor();
    if (ok) {
        syncCurrentStates();
        return { true, false, "实时环境光照：" + ambientLightReadingText() };
    }
    return { false, false, "读取环境光照失败：从机设备未响应或光照传感器离线" };
}

ActionResult Action_SendIRKey(int keyIndex, int profile) {
    if (profile >= 0 && profile <= 2) {
        IRService::setActiveProfile(profile);
    }
    return { true, false, "已为您切换至对应红外遥控预设并就绪" };
}

void Action_UpdateEnvironment(float temp, float hum, float press) {
    if (temp > 0.0f) s_state.indoor_temp = temp;
    if (hum > 0.0f) s_state.indoor_hum = hum;
    if (press > 0.0f) s_state.indoor_press = press;
}

void Action_UpdateLightLux(float lux) {
    if (lux >= 0.0f) s_state.indoor_lux = lux;
}

String Action_BuildSystemPromptStateContext() {
    syncCurrentStates();

    String ctx = "【当前设备实时状态表】\n";
    ctx += "- 智能灯: " + String(s_state.light_on ? "【开启】" : "【关闭】") + "\n";
    ctx += "- 补光灯: " + String(s_state.flash_on ? "【开启】" : "【关闭】") + "\n";
    
    String mStat = "【停止】";
    if (s_state.music_state == 1) mStat = "【播放中】";
    else if (s_state.music_state == 2) mStat = "【暂停】";
    ctx += "- 音乐播放器: " + mStat + " 当前曲目: 《" + s_state.current_song_title + "》, 音量: " + String(s_state.music_volume) + "%\n";

    const auto& playlist = MusicPlayerService::getPlaylist();
    if (!playlist.empty()) {
        ctx += "- SD卡可用歌单 (" + String(playlist.size()) + "首): ";
        size_t showN = min((size_t)10, playlist.size());
        for (size_t i = 0; i < showN; i++) {
            ctx += "《" + playlist[i].title + "》";
            if (i < showN - 1) ctx += ", ";
        }
        if (playlist.size() > 10) ctx += " 等";
        ctx += "\n";
    }

    ctx += "- 屏幕亮度: " + String((s_state.screen_brightness * 100) / 255) + "%, 提示音: " + String(s_state.sound_muted ? "静音" : "正常") + "\n";

    if (s_state.indoor_temp > 0.0f || s_state.indoor_hum > 0.0f) {
        char envBuf[64];
        snprintf(envBuf, sizeof(envBuf), "- 室内环境: 温度 %.1f°C, 湿度 %.0f%%, 气压 %.0fhPa, 光照 %.0fLux\n",
                 s_state.indoor_temp, s_state.indoor_hum, s_state.indoor_press, s_state.indoor_lux);
        ctx += envBuf;
    }

    ctx += "【点歌规则】若用户想听歌或点播某首歌，必须调用 controlMusic 工具：若用户指定了歌名，将提取的歌名填入 song 参数（如 song=\"七里香\"）；若用户未指定歌名只说播放音乐，则 song 留空。\n";
    ctx += "【核心规则】若用户指令的目标状态与上述状态已完全一致（如灯已开时用户说开灯、已暂停时说暂停），严禁调用工具，直接自然语言告知用户已是该状态。";
    return ctx;
}

} // namespace DeviceActionService
