#ifndef CUSTOM_UI_ENGINE_H
#define CUSTOM_UI_ENGINE_H

#include <Arduino.h>
#include <USB.h>
extern USBCDC USBSerial;
#include <TFT_eSPI.h>
#include <U8g2_for_TFT_eSPI.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <time.h>
#include "app_config.h"
#include "app_events.h"
#include "debug_log_service.h"
#include <Preferences.h>
#include <vector>

struct WeatherForecast {
    String date;
    String week;
    String weather;
    int high_temp;
    int low_temp;
};

extern WeatherForecast forecasts[4];
extern volatile bool weather_ready;
extern void beep(int ms, int count = 1);

namespace CustomUiTheme {
constexpr uint16_t BG_TOP       = 0x08A4;
constexpr uint16_t SURFACE      = 0x1107;
constexpr uint16_t SURFACE_ALT  = 0x1568;
constexpr uint16_t BORDER       = 0x224D;
constexpr uint16_t PRIMARY      = 0x361F;
constexpr uint16_t PRIMARY_DARK = 0x0BCB;
constexpr uint16_t PURPLE       = 0x8BFD;
constexpr uint16_t SUCCESS      = 0x3ED1;
constexpr uint16_t WARNING      = 0xFDA8;
constexpr uint16_t DANGER       = 0xFAD0;
constexpr uint16_t TEXT         = 0xF7BE;
constexpr uint16_t MUTED        = 0x8D57;
constexpr uint16_t FOCUS_GOLD   = 0xFEA0;
}

enum CustomUiKey {
    CUSTOM_KEY_NONE = 0,
    CUSTOM_KEY_UP = 17,
    CUSTOM_KEY_DOWN = 18,
    CUSTOM_KEY_LEFT = 20,
    CUSTOM_KEY_RIGHT = 19,
    CUSTOM_KEY_ENTER = 10,
    CUSTOM_KEY_ESC = 27
};

enum CustomUiPage {
    PAGE_HOME = 0,
    PAGE_WEATHER,
    PAGE_AI,
    PAGE_SMART_HOME,
    PAGE_DEVICES,
    PAGE_CAMERA,
    PAGE_MUSIC,
    PAGE_SYSTEM,
    PAGE_SETTINGS,
    PAGE_LOGS,
    PAGE_COUNT
};

class CustomUiEngine {
public:
    static void init(TFT_eSPI* tft_ptr);
    static void update();
    
    // 当前屏幕逻辑尺寸与方向 (跟随 tft 旋转状态)
    static int screenWidth();
    static int screenHeight();
    static bool isLandscape();
    static void handleKeyRelease(uint32_t key);
    
    // 按键与手势事件分发
    static void handleKeyInput(uint32_t key);
    
    // 状态切换与接口兼容
    static int getCurrentPage() { return current_page; }
    static void setCurrentPage(int page) { if (page >= 0 && page < PAGE_COUNT) { current_page = page; page_focused = false; ui_needs_redraw = true; } }
    static bool isPageFocused() { return page_focused; }
    static void setPageFocused(bool focused) { page_focused = focused; }
    static bool isPlaylistModalOpen() { return music_playlist_modal; }
    static U8g2_for_TFT_eSPI u8f;
    static TFT_eSprite& getSprite() { return spr; }

    // 辅助数据更新触发器
    static void notifyUiNeedsUpdate();
    static void sleepDisplay();
    static void notifyAiContentUpdate() { ++ai_content_revision; ai_scroll_lines = 0; ui_needs_redraw = true; }
    static void notifyDeviceContentUpdate() { ++device_content_revision; device_scroll_lines = 0; ui_needs_redraw = true; }

    // 🖥️ USB 串口投屏控制
    static void setScreenMirror(bool enabled);
    static bool getScreenMirror() { return screen_mirror_enabled; }

    // 🔕 系统提示音控制
    static void setSystemSoundEnabled(bool enabled);
    static bool getSystemSoundEnabled() { return system_sound_enabled; }

    // 📱 屏幕自动旋转控制
    static void setAutoRotateEnabled(bool enabled);
    static bool getAutoRotateEnabled() { return auto_rotate_enabled; }
    // 指南针校准期间锁定自动旋转, 防止八字转动时屏幕被 IMU 切成横屏
    static void setRotationLocked(bool locked) { rotation_locked = locked; ui_needs_redraw = true; }
    static bool isRotationLocked() { return rotation_locked; }

    // 💡 屏幕背光亮度控制 (10-255)
    static void setScreenBrightness(int level);
    static int getScreenBrightness() { return screen_brightness; }

    // 🌙 自动息屏控制 (挡位: 0, 1, 5, 10, 15, 20, 25, 30)
    static void setAutoSleepTime(int minutes);
    static int getAutoSleepTime() { return auto_sleep_time; }
    static uint32_t getLastInteractionTime() { return last_interaction_time; }

    // 🔊 TTS语音音量控制 (0~100)
    static void setTtsVolume(int vol);
    static int getTtsVolume() { return tts_volume; }

    // 唤醒屏幕与寄存器自愈保活
    static void wakeUpScreen();
    static void ensureScreenState();

    // 全局数据句柄与兼容导出
    static String online_devices_list;
    static String ai_vision_result;
    static void setAiVisionResult(const String& result) { ai_vision_result = result; ui_needs_redraw = true; }
    static String getAiVisionResult() { return ai_vision_result; }
    static size_t log_scroll_lines;
    static uint32_t log_scroll_revision;
    static size_t ai_scroll_lines;
    static uint32_t ai_scroll_revision;
    static uint32_t ai_content_revision;
    static volatile size_t device_scroll_lines;
    static uint32_t device_scroll_revision;
    static volatile uint32_t device_content_revision;

private:
    static TFT_eSPI* tft;
    static TFT_eSprite spr;
    
    static int current_page;
    static bool page_focused;
    static int focus_index; // 智能家居/附近设备页面的选定索引
    static int settings_focus_index; // 设置页面的选定索引
    static bool settings_adjust_mode; // 设置页面的调节模式
    static volatile bool ui_needs_redraw;
    static uint32_t last_redraw_time;
    static volatile bool screen_mirror_enabled;
    static bool system_sound_enabled;
    static bool auto_rotate_enabled;
    static bool rotation_locked;
    static int screen_brightness;
    static int tts_volume;
    static int auto_sleep_time;
    static bool settings_ir_menu_mode;
    static int settings_ir_focus_index;
    static bool settings_ir_learning_mode;

    // 📶 WiFi 设置与手机式虚拟全键盘交互状态
    enum SettingsWifiSubMode {
        WIFI_SUB_NONE = 0,
        WIFI_SUB_SCAN_LIST,
        WIFI_SUB_KEYBOARD,
        WIFI_SUB_CONNECTING
    };
    static SettingsWifiSubMode settings_wifi_mode;

    struct ScannedWifiItem {
        String ssid;
        int32_t rssi;
        bool is_open;
    };
    static std::vector<ScannedWifiItem> scanned_wifis;
    static int wifi_list_focus_index;
    static int wifi_list_scroll_top;
    static bool wifi_scanning_in_progress;
    enum WifiScanState {
        WIFI_SCAN_IDLE = 0,
        WIFI_SCAN_STARTING,
        WIFI_SCAN_SCANNING,
        WIFI_SCAN_DONE,
        WIFI_SCAN_FAILED_STATE
    };
    static WifiScanState wifi_scan_state;
    static uint32_t wifi_scan_start_time;
    static uint8_t wifi_scan_retry_count;
    static String wifi_scan_status_msg;
    static void pollWifiScan();
    static void stopWifiScan();
    static bool isWifiScanning() { return wifi_scanning_in_progress; }
    static String selected_wifi_ssid;
    static String wifi_password_input;

    // 虚拟键盘状态 (4排10列网格)
    static int kb_row;
    static int kb_col;
    static bool kb_symbol_mode;
    static bool kb_upper_mode;

    // 连接状态控制
    static bool wifi_connect_pending;
    static int wifi_connect_result; // 0: 正在连, 1: 成功, 2: 失败
    static uint32_t wifi_connect_finish_time;
    static bool screen_sleeping;
    static uint32_t last_interaction_time;
    static bool light_brightness_mode;
    static Preferences prefs;
    
    // 🖥️ USB 串口帧流式传输
    static void streamFrameToSerial();
    
    // 页面分发渲染函数
    static void renderPage();
    static void renderHeader(const char* title);
    static void renderPageIndicator(int current, int total);
    static void drawWeatherIcon(const uint8_t* map, int x, int y, float scale = 1.3f);
    
    // 🎵 本地音乐播放器三层级导航交互状态
    enum MusicUiTier {
        MUSIC_TIER_PROGRESS = 0, // 上层: 进度条
        MUSIC_TIER_BUTTONS  = 1, // 中层: 5个功能按键
        MUSIC_TIER_VOLUME   = 2  // 下层: 音量条
    };
    static MusicUiTier music_tier;
    static int music_btn_idx;              // 0:模式, 1:上曲, 2:播/暂, 3:下曲, 4:歌单
    static bool music_adjusting_progress;  // 是否处于进度条调节交互中
    static bool music_adjusting_volume;    // 是否处于音量调节交互中
    static bool music_playlist_modal;      // 歌单抽屉是否展开
    static int music_playlist_scroll;      // 歌单滚动偏移
    static int music_playlist_focus;       // 歌单高亮索引
    
    // 独立页面向量渲染
    static void renderHome();
    static void renderWeather();
    static void renderAi();
    static void renderSmartHome();
    static void renderSystem();
    static void renderLogs();
    static void renderDevices();
    static void renderSettings();
    static void renderWifiScanList();
    static void renderWifiKeyboard();
    static void renderWifiConnecting();
    static void startWifiScan();
    static void renderCamera();
    static void renderMusic();

    // 绘图矢量辅助工具
    static void drawCard(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t bg_color = CustomUiTheme::SURFACE, uint16_t border_color = CustomUiTheme::BORDER);
    static void drawProgressBar(int16_t x, int16_t y, int16_t w, int16_t h, uint32_t pct, uint16_t bar_color);
    static void drawActionButton(int16_t x, int16_t y, int16_t w, int16_t h, const char* label, uint16_t bg_color, bool focused, int font_level = 1);
};

#endif // CUSTOM_UI_ENGINE_H
