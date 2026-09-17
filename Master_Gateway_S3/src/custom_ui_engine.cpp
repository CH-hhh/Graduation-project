#include "tusb.h"
#include "custom_ui_engine.h"
#include "camera_service.h"
#include "music_player_service.h"
#include "pinyin_table.h"
#include "ir_service.h"
#include "app_events.h"
#include <freertos/semphr.h>
#include <vector>
#include <algorithm>
#include <WiFi.h>
#include <esp_wifi.h>
#include "network_runtime.h"
#include "weather_icons.h"
#include "chinese_full_font.h"
#include "HardwareSerial.h"
#include "USB.h"
#include <math.h>

extern String skillIPLocation();
extern void skillWeather();

#define Serial DebugLog

static SemaphoreHandle_t s_usb_cdc_mutex = nullptr;

TFT_eSPI* CustomUiEngine::tft = nullptr;
static uint32_t fps_last_time = 0;
static int fps_frames = 0;
static int current_fps = 0;
TFT_eSprite CustomUiEngine::spr = TFT_eSprite(nullptr);
U8g2_for_TFT_eSPI CustomUiEngine::u8f;

int CustomUiEngine::current_page = PAGE_HOME;
bool CustomUiEngine::page_focused = false;
int CustomUiEngine::focus_index = 0;
int CustomUiEngine::settings_focus_index = 0;
bool CustomUiEngine::settings_adjust_mode = false;

extern TaskHandle_t gui_task_handle;

void CustomUiEngine::notifyUiNeedsUpdate() {
    ui_needs_redraw = true;
    if (gui_task_handle) {
        xTaskNotifyGive(gui_task_handle);
    }
}
bool CustomUiEngine::settings_ir_menu_mode = false;
int CustomUiEngine::settings_ir_focus_index = 0;
bool CustomUiEngine::settings_ir_learning_mode = false;
volatile bool CustomUiEngine::ui_needs_redraw = true;
uint32_t CustomUiEngine::last_redraw_time = 0;
volatile bool CustomUiEngine::screen_mirror_enabled = false; // 投屏绝不开机自启
bool CustomUiEngine::system_sound_enabled = true;
bool CustomUiEngine::auto_rotate_enabled = true;
bool CustomUiEngine::rotation_locked = false;
int CustomUiEngine::screen_brightness = 255;
int CustomUiEngine::tts_volume = 100;
int CustomUiEngine::auto_sleep_time = 1;
bool CustomUiEngine::screen_sleeping = false;
uint32_t CustomUiEngine::last_interaction_time = 0;
bool CustomUiEngine::light_brightness_mode = false;
Preferences CustomUiEngine::prefs;

CustomUiEngine::SettingsWifiSubMode CustomUiEngine::settings_wifi_mode = CustomUiEngine::WIFI_SUB_NONE;
std::vector<CustomUiEngine::ScannedWifiItem> CustomUiEngine::scanned_wifis;
int CustomUiEngine::wifi_list_focus_index = 0;
int CustomUiEngine::wifi_list_scroll_top = 0;
bool CustomUiEngine::wifi_scanning_in_progress = false;
CustomUiEngine::WifiScanState CustomUiEngine::wifi_scan_state = CustomUiEngine::WIFI_SCAN_IDLE;
uint32_t CustomUiEngine::wifi_scan_start_time = 0;
uint8_t CustomUiEngine::wifi_scan_retry_count = 0;
String CustomUiEngine::wifi_scan_status_msg = "";
String CustomUiEngine::selected_wifi_ssid = "";
String CustomUiEngine::wifi_password_input = "";
int CustomUiEngine::kb_row = 0;
int CustomUiEngine::kb_col = 0;
bool CustomUiEngine::kb_symbol_mode = false;
bool CustomUiEngine::kb_upper_mode = false;
bool CustomUiEngine::wifi_connect_pending = false;
int CustomUiEngine::wifi_connect_result = 0;
uint32_t CustomUiEngine::wifi_connect_finish_time = 0;


CustomUiEngine::MusicUiTier CustomUiEngine::music_tier = CustomUiEngine::MUSIC_TIER_PROGRESS;
int CustomUiEngine::music_btn_idx = 2; // 默认居中在 [播放/暂停] 按钮
bool CustomUiEngine::music_adjusting_progress = false;
bool CustomUiEngine::music_adjusting_volume = false;
bool CustomUiEngine::music_playlist_modal = false;
int CustomUiEngine::music_playlist_scroll = 0;
int CustomUiEngine::music_playlist_focus = 0;
static int s_preview_seek_sec = 0;
static uint32_t s_seek_debounce_deadline = 0;
static bool s_seek_pending = false;

String CustomUiEngine::online_devices_list = "";
String CustomUiEngine::ai_vision_result = "等待拍摄识图...";
size_t CustomUiEngine::log_scroll_lines = 0;
uint32_t CustomUiEngine::log_scroll_revision = 0;

int CustomUiEngine::screenWidth() { return tft->width(); }
int CustomUiEngine::screenHeight() { return tft->height(); }
bool CustomUiEngine::isLandscape() { return tft->width() > tft->height(); }
size_t CustomUiEngine::ai_scroll_lines = 0;
uint32_t CustomUiEngine::ai_scroll_revision = 0;
uint32_t CustomUiEngine::ai_content_revision = 0;
volatile size_t CustomUiEngine::device_scroll_lines = 0;
uint32_t CustomUiEngine::device_scroll_revision = 0;
volatile uint32_t CustomUiEngine::device_content_revision = 0;

extern String currentText;
extern bool isRecording;
extern bool isCommunicating;
extern bool isSpeaking;
extern WeatherForecast forecasts[4];
extern String current_city;
extern volatile float ack_temperature_c;
extern volatile float ack_pressure_hpa;
extern volatile float ack_dht_temp;
extern volatile float ack_dht_hum;
extern volatile int global_light_state;
extern volatile int smart_light_ack_state;
extern String environmentReadingText();
extern SemaphoreHandle_t state_mutex;
extern int current_rotation;
extern int light_brightness;
extern volatile uint32_t last_light_ack_time;
extern void sendLightBrightness(int v);
extern void sendLightBrightnessFast(int v);

bool smart_home_show_result[5] = {false, false, false, false, false};

// 🌟 绘制天气图标 (支持 1.25x 等比例动态放大，主页与天气预报页 100% 绝对一致)
void CustomUiEngine::drawWeatherIcon(const uint8_t* map, int x, int y, float scale) {
    if (!map) return;
    int src_w = 32;
    int src_h = 32;
    int dst_w = (int)(32 * scale);
    int dst_h = (int)(32 * scale);

    if (map == weather_rain_map) {
        // 1. 上方云朵部分 (src_r < 17) 保持等比例高保真渲染
        int cloud_bottom_r = (int)(17 * scale);
        for (int r = 0; r < cloud_bottom_r; r++) {
            int src_r = (int)(r / scale);
            if (src_r >= 17) break;
            
            for (int c = 0; c < dst_w; c++) {
                int src_c = (int)(c / scale);
                if (src_c >= src_w) src_c = src_w - 1;

                int idx = (src_r * src_w + src_c) * 3;
                uint8_t low = map[idx];
                uint8_t high = map[idx + 1];
                uint8_t alpha = map[idx + 2];

                if (alpha > 30) {
                    uint16_t color = (uint16_t)low | ((uint16_t)high << 8);
                    spr.drawPixel(x + c, y + r, color);
                }
            }
        }

        // 2. 下方 5 颗极简垂直雨滴 (基于 scale 动态比例布局，主页与预报页 100% 绝对一致)
        uint16_t c_cyan = 0x3EBF;  // 高亮天蓝
        uint16_t c_white = 0x7F5F; // 冰晶纯白

        int top_y = cloud_bottom_r + (int)(3 * scale); // 留出 3px * scale 的云下空隙
        int bot_y = top_y + (int)(4 * scale);          // 上下两排雨滴间距

        // 上排 3 颗 2px 宽垂直雨滴
        int x1 = (int)(10 * scale), x2 = (int)(16 * scale), x3 = (int)(22 * scale);
        spr.drawFastVLine(x + x1, y + top_y, 2, c_cyan);
        spr.drawFastVLine(x + x1 + 1, y + top_y, 2, c_cyan);

        spr.drawFastVLine(x + x2, y + top_y, 2, c_white);
        spr.drawFastVLine(x + x2 + 1, y + top_y, 2, c_white);

        spr.drawFastVLine(x + x3, y + top_y, 2, c_cyan);
        spr.drawFastVLine(x + x3 + 1, y + top_y, 2, c_cyan);

        // 下排 2 颗 2px 宽垂直雨滴 (精准错开在 1&2, 2&3 几何正中间)
        int bx1 = (x1 + x2) / 2;
        int bx2 = (x2 + x3) / 2;

        spr.drawFastVLine(x + bx1, y + bot_y, 2, c_white);
        spr.drawFastVLine(x + bx1 + 1, y + bot_y, 2, c_white);

        spr.drawFastVLine(x + bx2, y + bot_y, 2, c_cyan);
        spr.drawFastVLine(x + bx2 + 1, y + bot_y, 2, c_cyan);
    } else {
        // 晴/云/阴图标完全保留原版 Alpha 平滑轮廓
        for (int r = 0; r < dst_h; r++) {
            int src_r = (int)(r / scale);
            if (src_r >= src_h) src_r = src_h - 1;
            
            for (int c = 0; c < dst_w; c++) {
                int src_c = (int)(c / scale);
                if (src_c >= src_w) src_c = src_w - 1;

                int idx = (src_r * src_w + src_c) * 3;
                uint8_t low = map[idx];
                uint8_t high = map[idx + 1];
                uint8_t alpha = map[idx + 2];

                if (alpha > 30) {
                    uint16_t color = (uint16_t)low | ((uint16_t)high << 8);
                    spr.drawPixel(x + c, y + r, color);
                }
            }
        }
    }
}

static const uint8_t* getWeatherIconMap(const String& weatherText) {
    if (weatherText.indexOf("晴") >= 0) return weather_sun_map;
    if (weatherText.indexOf("雨") >= 0) return weather_rain_map;
    if (weatherText.indexOf("阴") >= 0) return weather_overcast_map;
    return weather_cloud_map;
}

// 🌟 针对 U8g2 字库遗漏的 GBK 姓名/生僻字的高精度 16x16 / 12x12 点阵字模扩展引擎 (如 奕、喆 等)
struct CustomGlyphItem {
    uint16_t unicode;
    uint8_t bitmap16[32];
    uint8_t bitmap12[24];
};

static const CustomGlyphItem s_custom_glyphs[] = {
    // '♪' (U+266A) - 音乐音符符号
    { 0x266A,
      { 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x0F, 0x00, 0x0D, 0x80, 0x0C, 0xC0, 0x0C, 0xC0, 0x0C, 0x80, 0x0C, 0x00, 0x0C, 0x00, 0x1C, 0x00, 0x3C, 0x00, 0x7E, 0x00, 0x7C, 0x00, 0x38, 0x00, 0x00, 0x00 },
      { 0x00, 0x00, 0x18, 0x00, 0x1E, 0x00, 0x16, 0x00, 0x12, 0x00, 0x10, 0x00, 0x10, 0x00, 0x30, 0x00, 0x78, 0x00, 0xF8, 0x00, 0x70, 0x00, 0x00, 0x00 } },
    // '奕' (U+5955) - 陈奕迅 / 奕
    { 0x5955,
      { 0x02, 0x00, 0x01, 0x00, 0x7F, 0xFC, 0x04, 0x40, 0x24, 0x50, 0x44, 0x48, 0x89, 0x44, 0x10, 0x80, 0x01, 0x00, 0x01, 0x00, 0xFF, 0xFE, 0x02, 0x80, 0x04, 0x40, 0x08, 0x20, 0x30, 0x18, 0xC0, 0x06 },
      { 0x08, 0x00, 0x04, 0x00, 0xFF, 0xE0, 0x11, 0x00, 0x51, 0x40, 0x91, 0x20, 0x23, 0x00, 0x04, 0x00, 0xFF, 0xE0, 0x0A, 0x00, 0x31, 0x80, 0xC0, 0x60 } },
    // '喆' (U+5586) - 陶喆 / 喆
    { 0x5586,
      { 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0xFE, 0xFE, 0x10, 0x10, 0x10, 0x10, 0x7C, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x7C, 0x7C, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x7C, 0x7C, 0x44, 0x44 },
      { 0x20, 0x80, 0x20, 0x80, 0xFB, 0xE0, 0x20, 0x80, 0x20, 0x80, 0xFB, 0xE0, 0x00, 0x00, 0xFB, 0xE0, 0x8A, 0x20, 0x8A, 0x20, 0xFB, 0xE0, 0x8A, 0x20 } },
    // '赟' (U+8D5F)
    { 0x8D5F,
      { 0x41, 0xD4, 0x20, 0x10, 0xFB, 0xFE, 0x10, 0x90, 0x52, 0xC8, 0x22, 0x8A, 0x57, 0xE6, 0x80, 0x02, 0x1F, 0xF0, 0x10, 0x10, 0x11, 0x10, 0x11, 0x10, 0x11, 0x10, 0x02, 0x60, 0x0C, 0x18, 0x70, 0x04 },
      { 0x46, 0xC0, 0xF0, 0xA0, 0x2F, 0xE0, 0xA2, 0x80, 0x4B, 0xA0, 0xAA, 0x60, 0x1F, 0x20, 0x7F, 0xC0, 0x44, 0x40, 0x44, 0x40, 0x1B, 0x80, 0xE0, 0x60 } },
    // '淼' (U+6DFC)
    { 0x6DFC,
      { 0x01, 0x00, 0x01, 0x08, 0x7D, 0x10, 0x05, 0xA0, 0x09, 0x60, 0x31, 0x18, 0xC5, 0x06, 0x02, 0x00, 0x08, 0x10, 0x0A, 0x14, 0x6A, 0xD4, 0x2C, 0x58, 0x2A, 0x54, 0x4A, 0x94, 0xA9, 0x52, 0x10, 0x20 },
      { 0x04, 0x40, 0xF6, 0x80, 0x15, 0x00, 0x24, 0x80, 0xCC, 0x60, 0x11, 0x00, 0x11, 0x00, 0xD7, 0xA0, 0x5B, 0x40, 0x55, 0x40, 0x91, 0x20, 0x33, 0x00 } },
    // '燊' (U+71CA)
    { 0x71CA,
      { 0x11, 0x08, 0x21, 0x10, 0x02, 0xE0, 0x0C, 0x18, 0x70, 0x04, 0x00, 0x10, 0x52, 0x52, 0x94, 0x94, 0x28, 0x28, 0xC4, 0xC4, 0x01, 0x00, 0x7F, 0xFC, 0x05, 0x40, 0x19, 0x30, 0xE1, 0x0E, 0x01, 0x00 },
      { 0x24, 0x40, 0x44, 0x80, 0x1B, 0x00, 0xE0, 0xE0, 0x20, 0x80, 0xAA, 0xA0, 0x51, 0x40, 0x8A, 0x20, 0x04, 0x00, 0xFF, 0xE0, 0x24, 0x80, 0xC4, 0x60 } },
    // '焜' (U+711C)
    { 0x711C,
      { 0x10, 0x00, 0x11, 0xFC, 0x11, 0x04, 0x11, 0x04, 0x55, 0xFC, 0x59, 0x04, 0x51, 0x04, 0x91, 0xFC, 0x10, 0x00, 0x11, 0x12, 0x11, 0xD4, 0x29, 0x18, 0x25, 0x10, 0x45, 0x52, 0x41, 0x92, 0x81, 0x0E },
      { 0x27, 0xE0, 0x24, 0x20, 0x2F, 0xE0, 0xB4, 0x20, 0xA7, 0xE0, 0xA0, 0x00, 0xA4, 0x80, 0x26, 0xA0, 0x24, 0xC0, 0x54, 0x80, 0x56, 0xA0, 0x84, 0x60 } },
    // '煊' (U+714A)
    { 0x714A,
      { 0x10, 0x40, 0x10, 0x20, 0x13, 0xFE, 0x12, 0x02, 0x54, 0x04, 0x59, 0xFC, 0x50, 0x00, 0x91, 0xFC, 0x11, 0x04, 0x11, 0xFC, 0x11, 0x04, 0x29, 0xFC, 0x25, 0x04, 0x44, 0x00, 0x43, 0xFE, 0x80, 0x00 },
      { 0x21, 0x00, 0x2F, 0xE0, 0x28, 0x20, 0xB7, 0xC0, 0xA0, 0x00, 0xA7, 0xC0, 0xA4, 0x40, 0x27, 0xC0, 0x24, 0x40, 0x57, 0xC0, 0x50, 0x00, 0x8F, 0xE0 } },
    // '玥' (U+73A5)
    { 0x73A5,
      { 0x00, 0x00, 0x00, 0xFC, 0xFE, 0x84, 0x10, 0x84, 0x10, 0x84, 0x10, 0xFC, 0x7C, 0x84, 0x10, 0x84, 0x10, 0x84, 0x10, 0xFC, 0x10, 0x84, 0x1E, 0x84, 0xF1, 0x04, 0x41, 0x04, 0x02, 0x14, 0x04, 0x08 },
      { 0x00, 0x00, 0xFB, 0xE0, 0x22, 0x20, 0x22, 0x20, 0x23, 0xE0, 0xFA, 0x20, 0x22, 0x20, 0x23, 0xE0, 0x22, 0x20, 0x3A, 0x20, 0xE4, 0x20, 0x08, 0x60 } },
    // '璟' (U+749F)
    { 0x749F,
      { 0x00, 0x00, 0x01, 0xFC, 0xFD, 0x04, 0x11, 0xFC, 0x11, 0x04, 0x11, 0xFC, 0x7C, 0x20, 0x13, 0xFE, 0x10, 0x00, 0x11, 0xFC, 0x11, 0x04, 0x1D, 0xFC, 0xE0, 0x20, 0x41, 0x24, 0x02, 0x22, 0x00, 0x60 },
      { 0x07, 0xC0, 0xF4, 0x40, 0x27, 0xC0, 0x24, 0x40, 0x27, 0xC0, 0xF1, 0x00, 0x2F, 0xE0, 0x24, 0x40, 0x27, 0xC0, 0x31, 0x00, 0xC5, 0x40, 0x0B, 0x20 } },
    // '晖' (U+6656)
    { 0x6656,
      { 0x00, 0x00, 0x03, 0xFE, 0x7A, 0x02, 0x4C, 0x44, 0x48, 0x40, 0x4B, 0xFC, 0x48, 0x80, 0x78, 0xA0, 0x49, 0x20, 0x49, 0xFC, 0x48, 0x20, 0x48, 0x20, 0x7B, 0xFE, 0x48, 0x20, 0x00, 0x20, 0x00, 0x20 },
      { 0x00, 0x00, 0xEF, 0xE0, 0xA8, 0x20, 0xA1, 0x00, 0xAF, 0xC0, 0xE2, 0x00, 0xA5, 0x00, 0xA7, 0xC0, 0xA1, 0x00, 0xEF, 0xE0, 0xA1, 0x00, 0x01, 0x00 } },
    // '斐' (U+6590)
    { 0x6590,
      { 0x04, 0x40, 0x7C, 0x7C, 0x04, 0x40, 0x04, 0x40, 0x3C, 0x78, 0x04, 0x40, 0x04, 0x40, 0x7C, 0x7C, 0x04, 0x40, 0x01, 0x00, 0xFF, 0xFE, 0x08, 0x20, 0x04, 0x40, 0x03, 0x80, 0x1C, 0x70, 0xE0, 0x0E },
      { 0x0A, 0x00, 0xFB, 0xE0, 0x0A, 0x00, 0x7B, 0xC0, 0x0A, 0x00, 0xFB, 0xE0, 0x0A, 0x00, 0x04, 0x00, 0xFF, 0xE0, 0x20, 0x80, 0x1F, 0x00, 0xE0, 0xE0 } },
    // '恺' (U+607A)
    { 0x607A,
      { 0x10, 0x20, 0x11, 0x24, 0x11, 0x24, 0x11, 0x24, 0x19, 0xFC, 0x54, 0x00, 0x51, 0xFC, 0x50, 0x04, 0x90, 0x04, 0x11, 0xFC, 0x11, 0x00, 0x11, 0x00, 0x11, 0x02, 0x11, 0x02, 0x10, 0xFE, 0x10, 0x00 },
      { 0x21, 0x00, 0x29, 0x20, 0x39, 0x20, 0xAF, 0xE0, 0xA0, 0x00, 0xAF, 0xC0, 0xA0, 0x40, 0x20, 0x40, 0x2F, 0xC0, 0x28, 0x00, 0x28, 0x20, 0x27, 0xE0 } },
    // '霆' (U+9706)
    { 0x9706,
      { 0x3F, 0xF8, 0x01, 0x00, 0x7F, 0xFE, 0x41, 0x02, 0x9D, 0x74, 0x01, 0x00, 0x1D, 0x70, 0x00, 0x08, 0x7C, 0x3C, 0x09, 0xE0, 0x10, 0x20, 0x3D, 0xFC, 0x44, 0x20, 0x2B, 0xFE, 0x10, 0x00, 0x6F, 0xFE },
      { 0x7F, 0xC0, 0x04, 0x00, 0xFF, 0xE0, 0x95, 0x20, 0x00, 0x40, 0x77, 0x80, 0x11, 0x00, 0x27, 0xC0, 0x91, 0x00, 0x57, 0xC0, 0x20, 0x00, 0xDF, 0xE0 } },
    // '瀚' (U+701A)
    { 0x701A,
      { 0x04, 0x10, 0x84, 0x10, 0x44, 0x28, 0x1F, 0x44, 0x84, 0x82, 0x5F, 0x00, 0x51, 0xEE, 0x1F, 0x22, 0x31, 0xAA, 0x5F, 0x66, 0xC4, 0x22, 0x5F, 0x66, 0x44, 0xAA, 0x44, 0x22, 0x44, 0xAA, 0x04, 0x44 },
      { 0x10, 0x80, 0xB9, 0x40, 0x52, 0x20, 0x38, 0x00, 0x2B, 0x60, 0xB9, 0x20, 0x6D, 0xA0, 0x3B, 0x60, 0x15, 0xA0, 0x79, 0x20, 0x91, 0x20, 0x13, 0x60 } },
    // '睿' (U+777F)
    { 0x777F,
      { 0x01, 0x00, 0x01, 0xF8, 0x01, 0x00, 0x7F, 0xFE, 0x40, 0x02, 0x9F, 0xF4, 0x09, 0x20, 0x32, 0x90, 0x0C, 0x60, 0x3F, 0xF8, 0xD0, 0x16, 0x1F, 0xF0, 0x10, 0x10, 0x1F, 0xF0, 0x10, 0x10, 0x1F, 0xF0 },
      { 0x07, 0xC0, 0x04, 0x00, 0xFF, 0xE0, 0xA4, 0xA0, 0x4A, 0x40, 0x3F, 0x80, 0xE0, 0xE0, 0x3F, 0x80, 0x20, 0x80, 0x3F, 0x80, 0x20, 0x80, 0x3F, 0x80 } },
    // '昊' (U+660A)
    { 0x660A,
      { 0x00, 0x00, 0x1F, 0xF0, 0x10, 0x10, 0x1F, 0xF0, 0x10, 0x10, 0x1F, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF8, 0x01, 0x00, 0x01, 0x00, 0xFF, 0xFE, 0x02, 0x80, 0x04, 0x40, 0x18, 0x30, 0xE0, 0x0E },
      { 0x7F, 0xC0, 0x40, 0x40, 0x7F, 0xC0, 0x40, 0x40, 0x7F, 0xC0, 0x00, 0x00, 0x7F, 0xC0, 0x04, 0x00, 0xFF, 0xE0, 0x0A, 0x00, 0x31, 0x80, 0xC0, 0x60 } },
    // '煜' (U+715C)
    { 0x715C,
      { 0x10, 0x00, 0x11, 0xFC, 0x11, 0x04, 0x15, 0xFC, 0x59, 0x04, 0x51, 0xFC, 0x51, 0x04, 0x90, 0x40, 0x10, 0x20, 0x13, 0xFE, 0x10, 0x00, 0x29, 0x04, 0x24, 0x88, 0x40, 0x00, 0x47, 0xFE, 0x80, 0x00 },
      { 0x27, 0xE0, 0x24, 0x20, 0x2F, 0xE0, 0xB4, 0x20, 0xA7, 0xE0, 0xA1, 0x00, 0xA0, 0x80, 0x27, 0xE0, 0x20, 0x00, 0x54, 0x20, 0x52, 0x40, 0x8F, 0xE0 } },
    // '涵' (U+6DB5)
    { 0x6DB5,
      { 0x00, 0x00, 0x23, 0xF8, 0x10, 0x10, 0x10, 0x20, 0x84, 0x44, 0x45, 0x54, 0x44, 0xE4, 0x14, 0x44, 0x14, 0xE4, 0x25, 0x54, 0xE6, 0x4C, 0x25, 0x44, 0x24, 0x84, 0x24, 0x04, 0x27, 0xFC, 0x00, 0x04 },
      { 0x9F, 0x80, 0x40, 0x80, 0x01, 0x00, 0x22, 0x20, 0xAA, 0xA0, 0x67, 0x20, 0x22, 0x20, 0x27, 0x20, 0x2A, 0xA0, 0x62, 0x20, 0xA6, 0x20, 0x3F, 0xE0 } },
    // '霖' (U+9716)
    { 0x9716,
      { 0x3F, 0xF8, 0x01, 0x00, 0x7F, 0xFE, 0x41, 0x02, 0x9D, 0x74, 0x01, 0x00, 0x1D, 0x70, 0x08, 0x20, 0x08, 0x20, 0x7E, 0xFC, 0x08, 0x30, 0x1C, 0x68, 0x2A, 0xA4, 0xC9, 0x22, 0x08, 0x20, 0x08, 0x20 },
      { 0x7F, 0xC0, 0x04, 0x00, 0xFF, 0xE0, 0xB5, 0xA0, 0x04, 0x00, 0x75, 0xC0, 0x20, 0x80, 0xFB, 0xE0, 0x20, 0x80, 0x71, 0xC0, 0xAA, 0xA0, 0x20, 0x80 } },
    // '萱' (U+8431)
    { 0x8431,
      { 0x08, 0x20, 0x08, 0x20, 0xFF, 0xFE, 0x0A, 0x20, 0x01, 0x00, 0x7F, 0xFE, 0x40, 0x02, 0x9F, 0xF4, 0x00, 0x00, 0x1F, 0xF0, 0x10, 0x10, 0x1F, 0xF0, 0x10, 0x10, 0x1F, 0xF0, 0x00, 0x00, 0xFF, 0xFE },
      { 0x20, 0x80, 0xFF, 0xE0, 0x24, 0x80, 0xFF, 0xE0, 0x80, 0x20, 0x7F, 0xC0, 0x40, 0x40, 0x7F, 0xC0, 0x40, 0x40, 0x7F, 0xC0, 0x00, 0x00, 0xFF, 0xE0 } },
    // '琪' (U+742A)
    { 0x742A,
      { 0x01, 0x08, 0x01, 0x08, 0xFB, 0xFC, 0x21, 0x08, 0x21, 0x08, 0x21, 0xF8, 0x21, 0x08, 0xF9, 0x08, 0x21, 0xF8, 0x21, 0x08, 0x21, 0x08, 0x3B, 0xFE, 0xE0, 0x00, 0x40, 0x90, 0x01, 0x08, 0x02, 0x04 },
      { 0x04, 0x40, 0xF4, 0x40, 0x2F, 0xE0, 0x24, 0x40, 0x27, 0xC0, 0xF4, 0x40, 0x27, 0xC0, 0x24, 0x40, 0x2F, 0xE0, 0x30, 0x00, 0xC4, 0x40, 0x08, 0x20 } },
    // '琦' (U+7426)
    { 0x7426,
      { 0x00, 0x40, 0x00, 0x40, 0xFB, 0xFC, 0x20, 0xA0, 0x21, 0x10, 0x22, 0x08, 0x27, 0xFE, 0xF8, 0x08, 0x23, 0xC8, 0x22, 0x48, 0x22, 0x48, 0x3A, 0x48, 0xE3, 0xC8, 0x40, 0x08, 0x00, 0x28, 0x00, 0x10 },
      { 0x01, 0x00, 0xF7, 0xC0, 0x22, 0x80, 0x24, 0x40, 0x2F, 0xE0, 0xF0, 0x40, 0x27, 0x40, 0x25, 0x40, 0x25, 0x40, 0x37, 0x40, 0xC4, 0x40, 0x00, 0xC0 } },
    // '雯' (U+96EF)
    { 0x96EF,
      { 0x3F, 0xF8, 0x01, 0x00, 0x7F, 0xFE, 0x41, 0x02, 0x9D, 0x74, 0x01, 0x00, 0x1D, 0x70, 0x02, 0x00, 0x01, 0x00, 0x7F, 0xFC, 0x08, 0x20, 0x04, 0x40, 0x03, 0x80, 0x04, 0x40, 0x18, 0x30, 0xE0, 0x0E },
      { 0x7F, 0xC0, 0x04, 0x00, 0xFF, 0xE0, 0xB5, 0xA0, 0x04, 0x00, 0x71, 0xC0, 0x04, 0x00, 0xFF, 0xE0, 0x20, 0x80, 0x11, 0x00, 0x0E, 0x00, 0xF1, 0xE0 } },
    // '婧' (U+5A67)
    { 0x5A67,
      { 0x10, 0x20, 0x10, 0x20, 0x13, 0xFE, 0x10, 0x20, 0xFD, 0xFC, 0x24, 0x20, 0x27, 0xFE, 0x24, 0x00, 0x25, 0xFC, 0x49, 0x04, 0x29, 0xFC, 0x11, 0x04, 0x29, 0xFC, 0x45, 0x04, 0x85, 0x14, 0x01, 0x08 },
      { 0x21, 0x00, 0x2F, 0xE0, 0x21, 0x00, 0xF7, 0xC0, 0x51, 0x00, 0x5F, 0xE0, 0x54, 0x40, 0x97, 0xC0, 0x54, 0x40, 0x27, 0xC0, 0x54, 0x40, 0x94, 0xC0 } },
    // '婷' (U+5A77)
    { 0x5A77,
      { 0x10, 0x40, 0x10, 0x20, 0x13, 0xFE, 0x10, 0x00, 0xFD, 0xFC, 0x25, 0x04, 0x25, 0xFC, 0x24, 0x00, 0x27, 0xFE, 0x4A, 0x02, 0x29, 0xFC, 0x10, 0x20, 0x28, 0x20, 0x44, 0x20, 0x84, 0xA0, 0x00, 0x40 },
      { 0x21, 0x00, 0x2F, 0xE0, 0x20, 0x00, 0xF7, 0xC0, 0x54, 0x40, 0x57, 0xC0, 0x50, 0x00, 0x9F, 0xE0, 0x58, 0x20, 0x27, 0xC0, 0x51, 0x00, 0x93, 0x00 } },
    // '嫣' (U+5AE3)
    { 0x5AE3,
      { 0x20, 0x00, 0x27, 0xFC, 0x20, 0x40, 0x22, 0x78, 0xFA, 0x40, 0x4F, 0xFE, 0x49, 0x00, 0x49, 0xFC, 0x4A, 0x00, 0x93, 0xFC, 0x50, 0x04, 0x25, 0x54, 0x35, 0x54, 0x48, 0x04, 0x48, 0x28, 0x80, 0x10 },
      { 0x4F, 0xE0, 0x41, 0x00, 0x45, 0xC0, 0xE5, 0x00, 0xBF, 0xE0, 0xA4, 0x00, 0xA7, 0xC0, 0xA8, 0x00, 0xAF, 0xE0, 0x40, 0x20, 0xAA, 0xA0, 0x95, 0x60 } },
    // '瑶' (U+7476)
    { 0x7476,
      { 0x00, 0x08, 0x00, 0x3C, 0xFB, 0xC0, 0x20, 0x04, 0x22, 0x44, 0x21, 0x28, 0xF9, 0xFC, 0x22, 0x20, 0x20, 0x20, 0x23, 0xFE, 0x20, 0x20, 0x39, 0x24, 0xE1, 0x24, 0x41, 0x24, 0x01, 0xFC, 0x00, 0x04 },
      { 0x00, 0xE0, 0xEF, 0x00, 0x42, 0x20, 0x49, 0x40, 0x44, 0x00, 0xE7, 0xC0, 0x49, 0x00, 0x41, 0x00, 0x4F, 0xE0, 0x61, 0x00, 0xC9, 0x20, 0x0F, 0xE0 } },
    // '璨' (U+74A8)
    { 0x74A8,
      { 0x00, 0x80, 0x00, 0xDC, 0xFA, 0x84, 0x23, 0xD4, 0x24, 0x48, 0x2A, 0x94, 0x23, 0x24, 0xFC, 0x40, 0x22, 0x48, 0x21, 0x50, 0x27, 0xFC, 0x38, 0xE0, 0xE1, 0x50, 0x42, 0x48, 0x0C, 0x46, 0x00, 0x40 },
      { 0x04, 0x00, 0xE6, 0xE0, 0x44, 0x20, 0x4E, 0xA0, 0x5A, 0x40, 0xE4, 0xA0, 0x49, 0x00, 0x45, 0x40, 0x5F, 0xE0, 0x63, 0x80, 0xC5, 0x40, 0x09, 0x20 } },
    // '璞' (U+749E)
    { 0x749E,
      { 0x00, 0x50, 0x02, 0x52, 0xF9, 0x54, 0x20, 0x50, 0x23, 0xFE, 0x20, 0x88, 0x20, 0x50, 0xFB, 0xFE, 0x20, 0x20, 0x21, 0xFC, 0x20, 0x20, 0x3B, 0xFE, 0xE0, 0x50, 0x40, 0x88, 0x01, 0x04, 0x06, 0x02 },
      { 0x0A, 0xA0, 0xE2, 0x80, 0x4F, 0xE0, 0x44, 0x40, 0x42, 0x80, 0xEF, 0xE0, 0x41, 0x00, 0x47, 0xC0, 0x41, 0x00, 0x6F, 0xE0, 0xC2, 0x80, 0x0C, 0x60 } },
    // '琛' (U+741B)
    { 0x741B,
      { 0x00, 0x00, 0x07, 0xFC, 0xFC, 0x04, 0x24, 0xA4, 0x21, 0x10, 0x22, 0x08, 0x20, 0x40, 0xF8, 0x40, 0x27, 0xFC, 0x20, 0x40, 0x20, 0xE0, 0x39, 0x50, 0xE2, 0x48, 0x4C, 0x46, 0x00, 0x40, 0x00, 0x40 },
      { 0x00, 0x00, 0xEF, 0xE0, 0x48, 0x20, 0x42, 0x80, 0x44, 0x40, 0xE1, 0x00, 0x4F, 0xE0, 0x41, 0x00, 0x43, 0x80, 0x65, 0x40, 0xC9, 0x20, 0x01, 0x00 } },
    // '珏' (U+73CF)
    { 0x73CF,
      { 0x00, 0x00, 0x00, 0x00, 0xFD, 0xFC, 0x10, 0x20, 0x10, 0x20, 0x10, 0x20, 0x10, 0x20, 0x7C, 0x20, 0x11, 0xFC, 0x10, 0x20, 0x10, 0x28, 0x10, 0x24, 0x1C, 0x24, 0xE0, 0x20, 0x43, 0xFE, 0x00, 0x00 },
      { 0x00, 0x00, 0xF7, 0xC0, 0x21, 0x00, 0x21, 0x00, 0x21, 0x00, 0xF1, 0x00, 0x27, 0xC0, 0x21, 0x00, 0x21, 0x40, 0x31, 0x20, 0xC1, 0x00, 0x0F, 0xE0 } },
    // '琰' (U+7430)
    { 0x7430,
      { 0x00, 0x20, 0x01, 0x22, 0xF9, 0x22, 0x22, 0x24, 0x20, 0x50, 0x20, 0x88, 0x23, 0x04, 0xF8, 0x22, 0x20, 0x20, 0x21, 0x24, 0x21, 0x24, 0x22, 0x28, 0x38, 0x50, 0xE0, 0x88, 0x01, 0x04, 0x06, 0x02 },
      { 0x01, 0x00, 0xE5, 0x20, 0x45, 0x40, 0x49, 0x00, 0x42, 0xC0, 0xEC, 0x20, 0x41, 0x00, 0x45, 0x20, 0x49, 0x40, 0x62, 0x80, 0xC4, 0x40, 0x18, 0x20 } }
};

static int16_t drawCustomChineseGlyph(int cur_x, int baseline_y, uint16_t unicode, uint16_t color, int font_level) {
    for (size_t i = 0; i < sizeof(s_custom_glyphs) / sizeof(s_custom_glyphs[0]); i++) {
        if (s_custom_glyphs[i].unicode == unicode) {
            if (font_level == 4) {
                int top_y = baseline_y - 9;
                const uint8_t* b = s_custom_glyphs[i].bitmap12;
                for (int row = 0; row < 12; row++) {
                    uint16_t row_bits = ((uint16_t)b[row * 2] << 8) | b[row * 2 + 1];
                    for (int col = 0; col < 12; col++) {
                        if (row_bits & (1 << (15 - col))) {
                            CustomUiEngine::getSprite().drawPixel(cur_x + col, top_y + row, color);
                        }
                    }
                }
                return 12;
            } else {
                int top_y = baseline_y - ((font_level == 1) ? 13 : ((font_level == 3) ? 12 : 11));
                const uint8_t* b = s_custom_glyphs[i].bitmap16;
                for (int row = 0; row < 16; row++) {
                    uint16_t row_bits = ((uint16_t)b[row * 2] << 8) | b[row * 2 + 1];
                    for (int col = 0; col < 16; col++) {
                        if (row_bits & (1 << (15 - col))) {
                            CustomUiEngine::getSprite().drawPixel(cur_x + col, top_y + row, color);
                        }
                    }
                }
                return (font_level == 1) ? 16 : ((font_level == 3) ? 15 : ((font_level == 2) ? 14 : 13));
            }
        }
    }
    return 0;
}

// 🌟 音乐播放器专用：全量 7445 汉字字模直读引擎 (包含原生 12x12 及 16x16 双规格字库，笔画100%饱满，绝不残缺)
static int16_t drawFullChineseGlyph(int cur_x, int baseline_y, uint16_t unicode, uint16_t color, int font_level) {
    // 1. 检查生僻字点阵表 (如 喆, 赟, 淼, 奕 等)
    for (size_t i = 0; i < sizeof(s_custom_glyphs) / sizeof(s_custom_glyphs[0]); i++) {
        if (s_custom_glyphs[i].unicode == unicode) {
            if (font_level == 4) { // 12px
                int top_y = baseline_y - 10;
                const uint8_t* b = s_custom_glyphs[i].bitmap12;
                for (int row = 0; row < 12; row++) {
                    uint16_t row_bits = ((uint16_t)b[row * 2] << 8) | b[row * 2 + 1];
                    if (row_bits == 0) continue;
                    for (int col = 0; col < 12; col++) {
                        if (row_bits & (1 << (15 - col))) {
                            CustomUiEngine::getSprite().drawPixel(cur_x + col, top_y + row, color);
                        }
                    }
                }
                return 13;
            } else { // 16px
                int top_y = baseline_y - 13;
                const uint8_t* b = s_custom_glyphs[i].bitmap16;
                for (int row = 0; row < 16; row++) {
                    uint16_t row_bits = ((uint16_t)b[row * 2] << 8) | b[row * 2 + 1];
                    if (row_bits == 0) continue;
                    for (int col = 0; col < 16; col++) {
                        if (row_bits & (1 << (15 - col))) {
                            CustomUiEngine::getSprite().drawPixel(cur_x + col, top_y + row, color);
                        }
                    }
                }
                return 16;
            }
        }
    }

    // 2. 查阅全量 GB2312 汉字字模库 (覆盖 7445 个全部一级/二级汉字)
    uint16_t gb = lookupGb2312Code(unicode);
    if (gb != 0) {
        uint8_t zone = (gb >> 8) - 0xA0;
        uint8_t pos = (gb & 0xFF) - 0xA0;
        if (zone >= 1 && zone <= 87 && pos >= 1 && pos <= 94) {
            uint32_t slot = (94 * (zone - 1) + (pos - 1));
            if (font_level == 4) {
                // 🌟 12px 原生点阵直读，笔画100%完整饱满，绝不残缺！
                uint32_t offset = slot * 24;
                int top_y = baseline_y - 10;
                for (int row = 0; row < 12; row++) {
                    uint8_t b0 = pgm_read_byte(&s_hzk12_bitmaps[offset + row * 2]);
                    uint8_t b1 = pgm_read_byte(&s_hzk12_bitmaps[offset + row * 2 + 1]);
                    uint16_t row_bits = ((uint16_t)b0 << 8) | b1;
                    if (row_bits == 0) continue;
                    for (int col = 0; col < 12; col++) {
                        if (row_bits & (1 << (15 - col))) {
                            CustomUiEngine::getSprite().drawPixel(cur_x + col, top_y + row, color);
                        }
                    }
                }
                return 13;
            } else {
                // 🌟 16px 原生点阵直读，宽度严格为 16px，绝不重叠！
                uint32_t offset = slot * 32;
                int top_y = baseline_y - 13;
                for (int row = 0; row < 16; row++) {
                    uint8_t b0 = pgm_read_byte(&s_hzk16_bitmaps[offset + row * 2]);
                    uint8_t b1 = pgm_read_byte(&s_hzk16_bitmaps[offset + row * 2 + 1]);
                    uint16_t row_bits = ((uint16_t)b0 << 8) | b1;
                    if (row_bits == 0) continue;
                    for (int col = 0; col < 16; col++) {
                        if (row_bits & (1 << (15 - col))) {
                            CustomUiEngine::getSprite().drawPixel(cur_x + col, top_y + row, color);
                        }
                    }
                }
                return 16;
            }
        }
    }
    return 0;
}

// 🌟 全局标准页面文本渲染引擎 (其他页面继续使用经典文泉驿字体库)
static void drawChineseText(const char* text, int x, int y, uint16_t color, uint16_t bg_color, int font_level = 0) {
    if (!text || !*text) return;
    CustomUiEngine::u8f.setFontMode(1); // 强制透明模式
    CustomUiEngine::u8f.setForegroundColor(color);
    CustomUiEngine::u8f.setBackgroundColor(bg_color);

    const uint8_t* font_main_a = u8g2_font_wqy13_t_gb2312a;
    const uint8_t* font_main_b = u8g2_font_wqy13_t_gb2312b;
    const uint8_t* font_jap1   = u8g2_font_b12_t_japanese1;
    const uint8_t* font_jap2   = u8g2_font_b12_t_japanese2;
    const uint8_t* font_kor    = u8g2_font_unifont_t_korean2;
    int baseline_y = y + 11;

    if (font_level == 1) { // 16px
        font_main_a = u8g2_font_wqy16_t_gb2312a;
        font_main_b = u8g2_font_wqy16_t_gb2312b;
        font_jap1   = u8g2_font_b16_t_japanese1;
        font_jap2   = u8g2_font_b16_t_japanese2;
        font_kor    = u8g2_font_unifont_t_korean2;
        baseline_y  = y + 14;
    } else if (font_level == 2) { // 14px
        font_main_a = u8g2_font_wqy14_t_gb2312a;
        font_main_b = u8g2_font_wqy14_t_gb2312b;
        font_jap1   = u8g2_font_b12_t_japanese1;
        font_jap2   = u8g2_font_b12_t_japanese2;
        font_kor    = u8g2_font_unifont_t_korean2;
        baseline_y  = y + 12;
    } else if (font_level == 3) { // 15px
        font_main_a = u8g2_font_wqy15_t_gb2312a;
        font_main_b = u8g2_font_wqy15_t_gb2312b;
        font_jap1   = u8g2_font_b16_t_japanese1;
        font_jap2   = u8g2_font_b16_t_japanese2;
        font_kor    = u8g2_font_unifont_t_korean2;
        baseline_y  = y + 13;
    } else if (font_level == 4) { // 12px
        font_main_a = u8g2_font_wqy12_t_gb2312a;
        font_main_b = u8g2_font_wqy12_t_gb2312b;
        font_jap1   = u8g2_font_b12_t_japanese1;
        font_jap2   = u8g2_font_b12_t_japanese2;
        font_kor    = u8g2_font_unifont_t_korean2;
        baseline_y  = y + 10;
    }

    CustomUiEngine::u8f.utf8_state = 0;
    int cur_x = x;
    const char* p = text;
    while (*p) {
        uint16_t e = CustomUiEngine::u8f.utf8_next((uint8_t)*p);
        p++;
        if (e == 0x0ffff) break;
        if (e != 0x0fffe) {
            int16_t delta = 0;
            // 1. 韩文谚文音节区 (Hangul Syllables / Jamo)
            if ((e >= 0xAC00 && e <= 0xD7A3) || (e >= 0x1100 && e <= 0x11FF) || (e >= 0x3130 && e <= 0x318F)) {
                CustomUiEngine::u8f.setFont(font_kor);
                CustomUiEngine::u8f.setFontMode(1);
                CustomUiEngine::u8f.setBackgroundColor(bg_color);
                delta = CustomUiEngine::u8f.drawGlyph(cur_x, baseline_y, e);
                if (delta == 0) {
                    CustomUiEngine::u8f.setFont(u8g2_font_unifont_t_korean1);
                    CustomUiEngine::u8f.setFontMode(1);
                    CustomUiEngine::u8f.setBackgroundColor(bg_color);
                    delta = CustomUiEngine::u8f.drawGlyph(cur_x, baseline_y, e);
                }
            } else {
                // 2. 默认优先中英文文泉驿 A 库 (GB2312 Level 1 常用字 + ASCII)
                CustomUiEngine::u8f.setFont(font_main_a);
                CustomUiEngine::u8f.setFontMode(1);
                CustomUiEngine::u8f.setBackgroundColor(bg_color);
                delta = CustomUiEngine::u8f.drawGlyph(cur_x, baseline_y, e);

                // 3. 中文文泉驿 B 库 (GB2312 Level 2 二级汉字: 如 烟、陈、迅 等)
                if (delta == 0) {
                    CustomUiEngine::u8f.setFont(font_main_b);
                    CustomUiEngine::u8f.setFontMode(1);
                    CustomUiEngine::u8f.setBackgroundColor(bg_color);
                    delta = CustomUiEngine::u8f.drawGlyph(cur_x, baseline_y, e);
                }

                // 4. 专属生僻字/GBK姓名汉字拓展引擎 (如 奕、喆 等)
                if (delta == 0) {
                    delta = drawCustomChineseGlyph(cur_x, baseline_y, e, color, font_level);
                }

                // 5. 若文泉驿未命中（如日文假名与独有扩展汉字），回退至日文字库
                if (delta == 0) {
                    CustomUiEngine::u8f.setFont(font_jap1);
                    CustomUiEngine::u8f.setFontMode(1);
                    CustomUiEngine::u8f.setBackgroundColor(bg_color);
                    delta = CustomUiEngine::u8f.drawGlyph(cur_x, baseline_y, e);
                }
                if (delta == 0) {
                    CustomUiEngine::u8f.setFont(font_jap2);
                    CustomUiEngine::u8f.setFontMode(1);
                    CustomUiEngine::u8f.setBackgroundColor(bg_color);
                    delta = CustomUiEngine::u8f.drawGlyph(cur_x, baseline_y, e);
                }
                if (delta == 0) {
                    CustomUiEngine::u8f.setFont(font_kor);
                    CustomUiEngine::u8f.setFontMode(1);
                    CustomUiEngine::u8f.setBackgroundColor(bg_color);
                    delta = CustomUiEngine::u8f.drawGlyph(cur_x, baseline_y, e);
                }
            }
            if (delta <= 0) {
                if (e == ' ') delta = (font_level == 4) ? 4 : ((font_level == 1) ? 6 : 5);
                else delta = (font_level == 4 || font_level == 0) ? 6 : 8;
            }
            cur_x += delta;
        }
    }
}

// 🎵 音乐播放界面专用：全量中英日韩全字符字库引擎 (原生双规格真点阵，0 笔画残缺，0 水平重叠)
static void drawMusicChineseText(const char* text, int x, int y, uint16_t color, uint16_t bg_color, int font_level = 0) {
    if (!text || !*text) return;
    CustomUiEngine::u8f.setFontMode(1); // 强制透明模式
    CustomUiEngine::u8f.setForegroundColor(color);
    CustomUiEngine::u8f.setBackgroundColor(bg_color);

    const uint8_t* font_ascii = (font_level == 4) ? u8g2_font_wqy12_t_gb2312a : u8g2_font_wqy15_t_gb2312a;
    const uint8_t* font_jap1  = (font_level == 4) ? u8g2_font_b12_t_japanese1 : u8g2_font_b16_t_japanese1;
    const uint8_t* font_jap2  = (font_level == 4) ? u8g2_font_b12_t_japanese2 : u8g2_font_b16_t_japanese2;
    const uint8_t* font_kor   = u8g2_font_unifont_t_korean2;
    int baseline_y = (font_level == 4) ? (y + 10) : (y + 13);

    CustomUiEngine::u8f.utf8_state = 0;
    int cur_x = x;
    const char* p = text;
    while (*p) {
        uint16_t e = CustomUiEngine::u8f.utf8_next((uint8_t)*p);
        p++;
        if (e == 0x0ffff) break;
        if (e != 0x0fffe) {
            int16_t delta = 0;
            // 1. 韩文谚文音节区
            if ((e >= 0xAC00 && e <= 0xD7A3) || (e >= 0x1100 && e <= 0x11FF) || (e >= 0x3130 && e <= 0x318F)) {
                CustomUiEngine::u8f.setFont(font_kor);
                CustomUiEngine::u8f.setFontMode(1);
                CustomUiEngine::u8f.setBackgroundColor(bg_color);
                delta = CustomUiEngine::u8f.drawGlyph(cur_x, baseline_y, e);
                if (delta == 0) {
                    CustomUiEngine::u8f.setFont(u8g2_font_unifont_t_korean1);
                    CustomUiEngine::u8f.setFontMode(1);
                    CustomUiEngine::u8f.setBackgroundColor(bg_color);
                    delta = CustomUiEngine::u8f.drawGlyph(cur_x, baseline_y, e);
                }
            } else if (e < 128) {
                // 2. ASCII 英文数字标点
                CustomUiEngine::u8f.setFont(font_ascii);
                CustomUiEngine::u8f.setFontMode(1);
                CustomUiEngine::u8f.setBackgroundColor(bg_color);
                delta = CustomUiEngine::u8f.drawGlyph(cur_x, baseline_y, e);
            } else {
                // 3. 🌟 音乐专属全量汉字库 (12px/16px 原生真字模直读，绝无残缺或字形重叠)
                delta = drawFullChineseGlyph(cur_x, baseline_y, e, color, font_level);

                // 4. 若未命中，查阅日文假名与字符
                if (delta == 0) {
                    CustomUiEngine::u8f.setFont(font_jap1);
                    CustomUiEngine::u8f.setFontMode(1);
                    CustomUiEngine::u8f.setBackgroundColor(bg_color);
                    delta = CustomUiEngine::u8f.drawGlyph(cur_x, baseline_y, e);
                }
                if (delta == 0) {
                    CustomUiEngine::u8f.setFont(font_jap2);
                    CustomUiEngine::u8f.setFontMode(1);
                    CustomUiEngine::u8f.setBackgroundColor(bg_color);
                    delta = CustomUiEngine::u8f.drawGlyph(cur_x, baseline_y, e);
                }
                if (delta == 0) {
                    CustomUiEngine::u8f.setFont(font_kor);
                    CustomUiEngine::u8f.setFontMode(1);
                    CustomUiEngine::u8f.setBackgroundColor(bg_color);
                    delta = CustomUiEngine::u8f.drawGlyph(cur_x, baseline_y, e);
                }
            }
            if (delta <= 0) {
                if (e == ' ') delta = (font_level == 4) ? 4 : 6;
                else delta = (font_level == 4) ? 12 : 16;
            }
            cur_x += delta;
        }
    }
}

static int getMusicTextWidth(const char* text, int font_level = 0) {
    if (!text || !*text) return 0;
    const uint8_t* font_ascii = (font_level == 4) ? u8g2_font_wqy12_t_gb2312a : u8g2_font_wqy15_t_gb2312a;
    CustomUiEngine::u8f.setFont(font_ascii);
    CustomUiEngine::u8f.utf8_state = 0;
    int total_w = 0;
    const char* p = text;
    while (*p) {
        uint16_t e = CustomUiEngine::u8f.utf8_next((uint8_t)*p);
        p++;
        if (e == 0x0ffff) break;
        if (e != 0x0fffe) {
            int delta = 0;
            // 1. 韩文谚文音节与字符 (Unifont 字符宽度为 16px，即使在 12px 模式下也占用 16px)
            if ((e >= 0xAC00 && e <= 0xD7A3) || (e >= 0x1100 && e <= 0x11FF) || (e >= 0x3130 && e <= 0x318F)) {
                delta = 16;
            } else if (e < 128) {
                // 2. ASCII 英文字母、数字、半角标点与空格 (按字体原生比例精确测宽)
                char c_buf[2] = {(char)e, 0};
                delta = CustomUiEngine::u8f.getUTF8Width(c_buf);
                if (delta <= 0) {
                    delta = (e == ' ') ? ((font_level == 4) ? 4 : 6) : ((font_level == 4) ? 6 : 8);
                }
            } else {
                // 3. 全量中文字符、全角标点、日文假名、拉丁扩展与音符 (12px 点阵占用 13px 步长，16px 占用 16px)
                delta = (font_level == 4) ? 13 : 16;
            }
            total_w += delta;
        }
    }
    return total_w;
}

static int getMultilingualTextWidth(const char* text, int font_level = 0) {
    if (!text || !*text) return 0;
    const uint8_t* font_main = u8g2_font_wqy13_t_gb2312a;
    if (font_level == 1) font_main = u8g2_font_wqy16_t_gb2312a;
    else if (font_level == 2) font_main = u8g2_font_wqy14_t_gb2312a;
    else if (font_level == 3) font_main = u8g2_font_wqy15_t_gb2312a;
    else if (font_level == 4) font_main = u8g2_font_wqy12_t_gb2312a;

    CustomUiEngine::u8f.setFont(font_main);
    CustomUiEngine::u8f.utf8_state = 0;
    int total_w = 0;
    const char* p = text;
    while (*p) {
        uint16_t e = CustomUiEngine::u8f.utf8_next((uint8_t)*p);
        p++;
        if (e == 0x0ffff) break;
        if (e != 0x0fffe) {
            if ((e >= 0xAC00 && e <= 0xD7A3) || (e >= 0x1100 && e <= 0x11FF) || (e >= 0x3130 && e <= 0x318F)) {
                // 韩文 Unifont 宽度为 16px
                total_w += 16;
            } else if (e < 0x80) {
                // ASCII 英文字母、数字与常规标点
                char c_buf[2] = {(char)e, 0};
                int cw = CustomUiEngine::u8f.getUTF8Width(c_buf);
                if (cw <= 0) {
                    cw = (e == ' ') ? ((font_level == 4) ? 4 : 6) : ((font_level == 4) ? 6 : 8);
                }
                total_w += cw;
            } else if ((e >= 0x3040 && e <= 0x30FF) || (e >= 0x31F0 && e <= 0x31FF)) {
                // 日文平假名与片假名 (12px / 16px)
                total_w += (font_level == 1 || font_level == 3) ? 16 : 12;
            } else {
                // 中文汉字及全角标点符号 (12px ~ 16px)
                total_w += (font_level == 4) ? 12 : ((font_level == 1) ? 16 : ((font_level == 3) ? 15 : ((font_level == 2) ? 14 : 13)));
            }
        }
    }
    return total_w;
}

// 🌟 辅助函数：深度清理歌词首尾不可见空白 (包含半角空格、制表符、回车换行与 UTF-8 全角空格 0xE3 0x80 0x80)
static String cleanLyricText(const String& src) {
    if (src.length() == 0) return "";
    String s = src;
    s.trim();
    bool changed = true;
    while (changed && s.length() > 0) {
        changed = false;
        // 去除 ASCII 首部空格
        if (s.startsWith(" ") || s.startsWith("\t") || s.startsWith("\r") || s.startsWith("\n")) {
            s = s.substring(1);
            changed = true;
        } else if (s.length() >= 3 && (uint8_t)s[0] == 0xE3 && (uint8_t)s[1] == 0x80 && (uint8_t)s[2] == 0x80) {
            // 去除全角空格 (0xE3 0x80 0x80)
            s = s.substring(3);
            changed = true;
        }
        // 去除 ASCII 尾部空格
        if (s.endsWith(" ") || s.endsWith("\t") || s.endsWith("\r") || s.endsWith("\n")) {
            s = s.substring(0, s.length() - 1);
            changed = true;
        } else if (s.length() >= 3 && (uint8_t)s[s.length() - 3] == 0xE3 && (uint8_t)s[s.length() - 2] == 0x80 && (uint8_t)s[s.length() - 1] == 0x80) {
            // 去除全角空格 (0xE3 0x80 0x80)
            s = s.substring(0, s.length() - 3);
            changed = true;
        }
    }
    return s;
}

static void drawChineseTextCentered(const char* text, int x, int y, int w, uint16_t color, uint16_t bg_color, int font_level = 0) {
    if (!text || !*text) return;
    String clean = cleanLyricText(text);
    if (clean.length() == 0) return;
    int tWidth = getMultilingualTextWidth(clean.c_str(), font_level);
    int drawX = x + (w - tWidth) / 2;
    drawChineseText(clean.c_str(), drawX < x ? x : drawX, y, color, bg_color, font_level);
}

static void drawMusicChineseTextCentered(const char* text, int x, int y, int w, uint16_t color, uint16_t bg_color, int font_level = 0) {
    if (!text || !*text) return;
    String clean = cleanLyricText(text);
    if (clean.length() == 0) return;
    int tWidth = getMusicTextWidth(clean.c_str(), font_level);
    int drawX = x + (w - tWidth) / 2;
    drawMusicChineseText(clean.c_str(), drawX < x ? x : drawX, y, color, bg_color, font_level);
}

static inline size_t getUtf8CharStep(const char* s, size_t pos, size_t len) {
    if (pos >= len) return 0;
    unsigned char c = (unsigned char)s[pos];
    if ((c & 0x80) == 0) return 1;
    if ((c & 0xE0) == 0xC0) return (pos + 2 <= len) ? 2 : 1;
    if ((c & 0xF0) == 0xE0) return (pos + 3 <= len) ? 3 : 1;
    if ((c & 0xF8) == 0xF0) return (pos + 4 <= len) ? 4 : 1;
    return 1;
}

// 🌟 辅助函数：长文本字符滑动跑马灯 (中英日韩全语言兼容，短文本在 box_w 内居中/居左展示，长文本在 box_w 内平滑滚动，绝不偏斜或越界)
static void drawMarqueeText(const char* text, int box_x, int box_y, int box_w, int box_h, uint16_t color, const uint8_t* font, bool is_title = false, uint16_t bg_color = CustomUiTheme::SURFACE) {
    if (!text || strlen(text) == 0 || box_w <= 0) return;
    int font_level = is_title ? 3 : 4; // 3=15/16px, 4=12px
    int text_w = getMultilingualTextWidth(text, font_level);
    int text_y = is_title ? (box_y + (box_h - 16) / 2 + 1) : (box_y + (box_h - 14) / 2);

    if (text_w <= box_w) {
        // 短文本：标题居中，列表项居左展示
        int tx = is_title ? (box_x + (box_w - text_w) / 2) : box_x;
        drawChineseText(text, tx, text_y, color, bg_color, font_level);
    } else {
        // 长文本：无缝循环 UTF-8 跑马灯流动
        String double_text = String(text) + "   " + String(text);
        size_t d_len = double_text.length();
        size_t t_len = strlen(text);
        
        // 统计原文本 UTF-8 字符总数
        size_t total_utf8_chars = 0;
        {
            size_t idx = 0;
            while (idx < t_len) {
                size_t step = getUtf8CharStep(text, idx, t_len);
                idx += (step > 0) ? step : 1;
                total_utf8_chars++;
            }
        }
        size_t loop_chars = total_utf8_chars + 3; // 加 3 个空格间隔
        size_t start_char = (millis() / 280) % (loop_chars > 0 ? loop_chars : 1);

        // 定位到 double_text 中的 start_char 字节偏移
        size_t byte_start = 0;
        size_t cur_char = 0;
        while (cur_char < start_char && byte_start < d_len) {
            size_t step = getUtf8CharStep(double_text.c_str(), byte_start, d_len);
            byte_start += (step > 0) ? step : 1;
            cur_char++;
        }

        // 从 byte_start 起截取能够填满 box_w 的最大子串
        size_t byte_end = byte_start;
        size_t last_valid_end = byte_start;
        while (byte_end < d_len) {
            size_t step = getUtf8CharStep(double_text.c_str(), byte_end, d_len);
            if (step == 0 || byte_end + step > d_len) break;
            String test_sub = double_text.substring(byte_start, byte_end + step);
            if (getMultilingualTextWidth(test_sub.c_str(), font_level) > box_w) {
                break;
            }
            byte_end += step;
            last_valid_end = byte_end;
        }

        String visible_str = double_text.substring(byte_start, last_valid_end);
        drawChineseText(visible_str.c_str(), box_x, text_y, color, bg_color, font_level);
    }
}

void CustomUiEngine::sleepDisplay() {
    analogWrite(static_cast<int>(AppConfig::TFT_BACKLIGHT), 0);
    if (tft) {
        tft->fillScreen(TFT_BLACK);
        tft->startWrite();
        tft->writecommand(0x10); // ST7789 SLPIN 睡眠指令
        tft->endWrite();
    }
}

void CustomUiEngine::init(TFT_eSPI* tft_ptr) {
    tft = tft_ptr;
    prefs.begin("settings", false);
    system_sound_enabled = prefs.getBool("sound", true);
    auto_rotate_enabled = prefs.getBool("rotate", true);
    screen_brightness = prefs.getInt("bright", 255);
    if (screen_brightness < 10 || screen_brightness > 255) screen_brightness = 255;
    tts_volume = prefs.getInt("tts_vol", 100);
    if (tts_volume < 0 || tts_volume > 100) tts_volume = 100;
    auto_sleep_time = prefs.getInt("sleep_time", 1);
    
    analogWriteFrequency(10000);
    analogWrite(static_cast<int>(AppConfig::TFT_BACKLIGHT), screen_brightness);
    
    spr = TFT_eSprite(tft);
    spr.setColorDepth(16);
    if (psramFound()) {
        spr.setAttribute(PSRAM_ENABLE, true);
    }
    // 一次性申请 320x320 最大画布：横屏(320x240)与竖屏(240x320)旋转时复用，
    // 不重建 Sprite，避免内存碎片；绘制与推送时按当前屏幕尺寸取可见区域。
    spr.createSprite(320, 320);
    
    u8f.begin(spr);
    u8f.setFontMode(1);
    u8f.setFontDirection(0);
    ui_needs_redraw = true;
}

void CustomUiEngine::setScreenMirror(bool enabled) {
    if (screen_mirror_enabled != enabled) {
        screen_mirror_enabled = enabled;
        ui_needs_redraw = true;
        Serial.printf("[MIRROR-LOG] 投屏状态切换 -> %s (Native CDC Online: %s)\r\n",
                      enabled ? "ENABLED" : "DISABLED",
                      tud_cdc_n_connected(0) ? "YES" : "NO");
    }
}

void CustomUiEngine::setSystemSoundEnabled(bool enabled) {
    if (system_sound_enabled != enabled) {
        system_sound_enabled = enabled;
        prefs.putBool("sound", enabled);
        ui_needs_redraw = true;
    }
}

void CustomUiEngine::setAutoRotateEnabled(bool enabled) {
    if (auto_rotate_enabled != enabled) {
        auto_rotate_enabled = enabled;
        prefs.putBool("rotate", enabled);
        ui_needs_redraw = true;
    }
}

void CustomUiEngine::setScreenBrightness(int level) {
    if (screen_brightness != level) {
        screen_brightness = level;
        if (screen_brightness < 10) screen_brightness = 10;
        if (screen_brightness > 255) screen_brightness = 255;
        prefs.putInt("bright", screen_brightness);
        analogWrite(static_cast<int>(AppConfig::TFT_BACKLIGHT), screen_brightness);
        ui_needs_redraw = true;
    }
}

void CustomUiEngine::setAutoSleepTime(int minutes) {
    if (auto_sleep_time != minutes) {
        auto_sleep_time = minutes;
        prefs.putInt("sleep_time", minutes);
        ui_needs_redraw = true;
    }
}

void CustomUiEngine::setTtsVolume(int vol) {
    if (vol < 0) vol = 0;
    if (vol > 100) vol = 100;
    if (tts_volume != vol) {
        tts_volume = vol;
        prefs.putInt("tts_vol", vol);
        ui_needs_redraw = true;
    }
}

void CustomUiEngine::wakeUpScreen() {
    screen_sleeping = false;
    last_interaction_time = millis();
    analogWrite(static_cast<int>(AppConfig::TFT_BACKLIGHT), screen_brightness);
    ui_needs_redraw = true;
    notifyUiNeedsUpdate();
}

void CustomUiEngine::ensureScreenState() {
    if (!tft || screen_sleeping) return;
    static uint32_t last_screen_check = 0;
    const uint32_t now = millis();
    // 🌟 每 2 秒静默发送一次 ST7789 控制寄存器保活指令,
    // 即使遭遇物理电源凹陷/电磁干扰导致芯片复位, 也会在 2 秒内静默自愈恢复正确的黑色主题
    if (now - last_screen_check >= 2000) {
        last_screen_check = now;
        tft->startWrite();
        tft->writecommand(0x11); // SLPOUT 确保芯片处于开屏状态
        tft->writecommand(0x29); // DISPON 保证显示开启
        tft->writecommand(0x20); // INVOFF 锁定正确的深黑主题模式
        tft->writecommand(0x3A); tft->writedata(0x55); // 16bit 颜色格式
        tft->endWrite();
    }
}

void CustomUiEngine::update() {
    const uint32_t now = millis();
    ensureScreenState();

    // 灯光亮度模式: 从机心跳超时 -> 未检测到并锁定; 心跳恢复后自动解锁
    if (light_brightness_mode && smart_light_ack_state != -1 &&
        now - last_light_ack_time > 2500) {
        smart_light_ack_state = -1;
        ui_needs_redraw = true;
    }
    
    if (settings_ir_learning_mode) {
        static IRPairingState last_ir_state = IR_PAIRING_NONE;
        IRPairingState curState = IRService::getPairingState();
        if (curState != last_ir_state) {
            last_ir_state = curState;
            ui_needs_redraw = true;
            if (curState == IR_PAIRING_DONE) {
                settings_ir_learning_mode = false;
            }
        }
    } else {
        if (IRService::isStandby()) {
            static uint32_t last_standby_toggle = 0;
            if (millis() - last_standby_toggle > 600) {
                last_standby_toggle = millis();
                screen_sleeping = !screen_sleeping;
                analogWrite(static_cast<int>(AppConfig::TFT_BACKLIGHT), screen_sleeping ? 0 : screen_brightness);
                ui_needs_redraw = true;
                beep(30, 1);
            }
        } else if (IRService::isPowerOff()) {
            beep(60, 2);
            MusicPlayerService::sendStop(); // 🌟 关机前通知 STM32 停止音乐播放，释放总线与解码
            delay(50);
            spr.fillSprite(CustomUiTheme::BG_TOP);
            int cx = (screenWidth() - 216) / 2;
            int cy = (screenHeight() - 120) / 2;
            drawCard(cx, cy, 216, 120);
            drawChineseTextCentered("正在关机...", cx, cy + 25, 216, CustomUiTheme::DANGER, CustomUiTheme::SURFACE, true);
            drawChineseTextCentered("进入 ESP32 深度睡眠待机", cx, cy + 65, 216, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
            spr.pushSprite(0, 0);
            delay(800);
            IRService::enterDeepSleep();
        }
        
        static uint32_t gpio18_press_start = 0;
        if (digitalRead(static_cast<int>(AppConfig::RETURN_BUTTON)) == LOW) {
            if (gpio18_press_start == 0) {
                gpio18_press_start = millis();
            } else if (millis() - gpio18_press_start >= 2000) {
                gpio18_press_start = 0;
                beep(60, 2);
                MusicPlayerService::sendStop(); // 🌟 关机前通知 STM32 停止音乐播放，释放总线与解码
                delay(50);
                spr.fillSprite(CustomUiTheme::BG_TOP);
                int cx = (screenWidth() - 216) / 2;
                int cy = (screenHeight() - 120) / 2;
                drawCard(cx, cy, 216, 120);
                drawChineseTextCentered("板载按键长按关机...", cx, cy + 25, 216, CustomUiTheme::DANGER, CustomUiTheme::SURFACE, true);
                drawChineseTextCentered("进入 ESP32 深度睡眠待机", cx, cy + 65, 216, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
                spr.pushSprite(0, 0);
                delay(800);
                IRService::enterDeepSleep();
            }
        } else {
            gpio18_press_start = 0;
        }
    }

    if (auto_sleep_time > 0 && !screen_sleeping && (now - last_interaction_time > (uint32_t)auto_sleep_time * 60000)) {
        screen_sleeping = true;
        Serial.println("[TFT] 自动息屏触发");
        analogWrite(static_cast<int>(AppConfig::TFT_BACKLIGHT), 0);
        ui_needs_redraw = true;
    }

    static int last_second = -1;
    if (!screen_sleeping) {
        time_t t = time(nullptr);
        if (t >= 1700000000) {
            struct tm timeinfo;
            localtime_r(&t, &timeinfo);
            if (timeinfo.tm_sec != last_second) {
                last_second = timeinfo.tm_sec;
                if (current_page == PAGE_HOME || current_page == PAGE_SYSTEM) {
                    ui_needs_redraw = true;
                }
            }
        }

        // Camera page: redraw whenever a new frame arrives in the queue.
        if (current_page == PAGE_CAMERA && page_focused) {
            if (CameraService::hasQueuedFrames()) {
                ui_needs_redraw = true;
            }
        }

        // 🎵 音乐播放器页: 轮询串口并以 33 FPS 高帧率刷新歌词、状态与跳动频谱
        MusicPlayerService::update();
        if (s_seek_pending && (now >= s_seek_debounce_deadline)) {
            s_seek_pending = false;
            MusicPlayerService::sendSeek(s_preview_seek_sec);
            MusicPlayerService::setMuteState(false);
            ui_needs_redraw = true;
        }
        if (current_page == PAGE_MUSIC && (now - last_redraw_time >= 30)) {
            ui_needs_redraw = true;
        }

        // 📶 WiFi 异步扫描任务状态机轮询
        if (wifi_scanning_in_progress) {
            pollWifiScan();
        }

        // 📶 WiFi 扫描列表与键盘: 维持 20 FPS 刷新长 SSID 跑马灯平滑滚动与扫描动效
        if (current_page == PAGE_SETTINGS && (settings_wifi_mode == WIFI_SUB_SCAN_LIST || settings_wifi_mode == WIFI_SUB_KEYBOARD) && (now - last_redraw_time >= 50)) {
            ui_needs_redraw = true;
        }

        // 🌟 开启投屏时，维持 20 FPS (每 50ms) 动态刷新 UI 画布，防止时间停滞或屏保卡死
        if (screen_mirror_enabled && (now - last_redraw_time >= 50)) {
            ui_needs_redraw = true;
        }
    }

    if (ui_needs_redraw) {
        // Camera streaming & Music player spectrum & WiFi marquee need high FPS
        bool bypass_limit = (current_page == PAGE_CAMERA && CameraService::state() == CameraService::State::STREAMING) || 
                            (current_page == PAGE_MUSIC) ||
                            (current_page == PAGE_SETTINGS && (settings_wifi_mode == WIFI_SUB_SCAN_LIST || settings_wifi_mode == WIFI_SUB_KEYBOARD));
        if (bypass_limit || (now - last_redraw_time >= 33)) {
            ui_needs_redraw = false;
            last_redraw_time = now;
            renderPage();
        }
    }

    
    streamFrameToSerial();
}

// 按行压缩画布中可见矩形区域: 画布行宽 stride 可能大于可见宽度 w (320x320 画布上取 240x320 竖屏区)
static size_t rleEncodeRGB565Rect(const uint16_t* fb, size_t stride, size_t w, size_t h, uint8_t* out_buf, size_t out_capacity) {
    if (!fb || !out_buf || w == 0 || h == 0) return 0;
    size_t out_idx = 0;
    for (size_t r = 0; r < h; r++) {
        const uint16_t* row = fb + r * stride;
        size_t i = 0;
        while (i < w) {
            uint16_t color = row[i];
            uint8_t count = 1;
            i++;
            while (i < w && row[i] == color && count < 255) {
                count++;
                i++;
            }
            if (out_idx + 3 > out_capacity) return 0;
            out_buf[out_idx++] = count;
            out_buf[out_idx++] = static_cast<uint8_t>(color >> 8);
            out_buf[out_idx++] = static_cast<uint8_t>(color & 0xFF);
        }
    }
    return out_idx;
}

void CustomUiEngine::streamFrameToSerial() {
    if (!screen_mirror_enabled) return;

    static uint32_t last_warn_time = 0;
    const uint32_t now = millis();

    if (!tud_cdc_n_connected(0)) {
        if (now - last_warn_time > 3000) {
            last_warn_time = now;
            Serial.println("[MIRROR-WARN] 投屏已开启，等待电脑连接 USB Native Type-C 端口...");
        }
        return;
    }

    static uint32_t last_stream = 0;
    if (now - last_stream < 30) return;  // 30ms ~30 FPS 极速发送
    last_stream = now;

    uint16_t* fb = (uint16_t*)spr.getPointer();
    if (!fb) return;

    // RLE 压缩缓冲区 (PSRAM 分配，最大 153.6KB 覆盖最坏情况)
    static const size_t RLE_BUF_SIZE = 153600;
    static uint8_t* rle_buf = nullptr;
    if (!rle_buf) {
        rle_buf = (uint8_t*)ps_malloc(RLE_BUF_SIZE);
        if (!rle_buf) {
            Serial.println("[MIRROR-ERR] PSRAM 分配 RLE 缓冲区失败!");
            return;
        }
    }
    size_t rle_bytes = rleEncodeRGB565Rect(fb, 320, screenWidth(), screenHeight(), rle_buf, RLE_BUF_SIZE);
    if (rle_bytes == 0) return;

    // 尝试获取 USB CDC 发送互斥锁，避免并发写入冲撞
    if (!s_usb_cdc_mutex) {
        s_usb_cdc_mutex = xSemaphoreCreateMutex();
    }
    if (s_usb_cdc_mutex && xSemaphoreTake(s_usb_cdc_mutex, pdMS_TO_TICKS(10)) != pdPASS) {
        if (now - last_warn_time > 3000) {
            last_warn_time = now;
            Serial.println("[MIRROR-WARN] USB CDC 发送互斥锁繁忙，跳过本帧推送");
        }
        return;
    }

    // 确保 FIFO 空间至少有 12 字节容纳包头 + 4 字节 Payload 长度
    if (tud_cdc_n_write_available(0) < 12) {
        if (s_usb_cdc_mutex) xSemaphoreGive(s_usb_cdc_mutex);
        return;
    }

    // 8 字节包头 (0xFC 表示 RLE 压缩帧, 后含宽/高各16位大端) + 4 字节大端 payload 长度
    int w = screenWidth();
    int h = screenHeight();
    uint8_t header_pkt[12] = {
        0xAA, 0x55, 0xFC, 0x00,
        static_cast<uint8_t>((w >> 8) & 0xFF), static_cast<uint8_t>(w & 0xFF),
        static_cast<uint8_t>((h >> 8) & 0xFF), static_cast<uint8_t>(h & 0xFF),
        static_cast<uint8_t>((rle_bytes >> 24) & 0xFF),
        static_cast<uint8_t>((rle_bytes >> 16) & 0xFF),
        static_cast<uint8_t>((rle_bytes >> 8) & 0xFF),
        static_cast<uint8_t>(rle_bytes & 0xFF)
    };

    uint32_t hdr_written = tud_cdc_n_write(0, header_pkt, 12);
    if (hdr_written != 12) {
        if (s_usb_cdc_mutex) xSemaphoreGive(s_usb_cdc_mutex);
        return;
    }
    tud_cdc_n_write_flush(0);

    // 发送 RLE 压缩数据 (超时按数据量动态计算，保证完整传输)
    uint8_t* data = rle_buf;
    size_t to_send = rle_bytes;
    uint32_t start_time = millis();
    uint32_t timeout_ms = 200 + (rle_bytes / 500);  // 动态超时: 200ms 基础 + 每500字节加1ms

    while (to_send > 0) {
        if (!tud_cdc_n_connected(0) || (millis() - start_time > timeout_ms)) break;

        uint32_t space = tud_cdc_n_write_available(0);
        if (space > 0) {
            uint32_t chunk = (to_send < space) ? to_send : space;
            uint32_t written = tud_cdc_n_write(0, data, chunk);
            if (written > 0) {
                data += written;
                to_send -= written;
            } else {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));  // 让出 CPU，由 Core 0 原生 usb_task 异步清理 CDC FIFO
        }
    }

    tud_cdc_n_write_flush(0);
    if (s_usb_cdc_mutex) xSemaphoreGive(s_usb_cdc_mutex);

}

static char getKbChar(int row, int col, bool symbol_mode, bool upper_mode) {
    if (row < 0 || row > 3 || col < 0 || col > 9) return '\0';
    if (!symbol_mode) {
        if (row == 0) {
            const char r0[10] = {'1', '2', '3', '4', '5', '6', '7', '8', '9', '0'};
            return r0[col];
        } else if (row == 1) {
            const char r1_lower[10] = {'q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p'};
            const char r1_upper[10] = {'Q', 'W', 'E', 'R', 'T', 'Y', 'U', 'I', 'O', 'P'};
            return upper_mode ? r1_upper[col] : r1_lower[col];
        } else if (row == 2) {
            if (col == 9) return '\0'; // Aa
            const char r2_lower[9] = {'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l'};
            const char r2_upper[9] = {'A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L'};
            return upper_mode ? r2_upper[col] : r2_lower[col];
        } else if (row == 3) {
            if (col == 0 || col == 8 || col == 9) return '\0';
            const char r3_lower[7] = {'z', 'x', 'c', 'v', 'b', 'n', 'm'};
            const char r3_upper[7] = {'Z', 'X', 'C', 'V', 'B', 'N', 'M'};
            return upper_mode ? r3_upper[col - 1] : r3_lower[col - 1];
        }
    } else {
        if (row == 0) {
            const char r0[10] = {'1', '2', '3', '4', '5', '6', '7', '8', '9', '0'};
            return r0[col];
        } else if (row == 1) {
            const char r1_sym[10] = {'!', '@', '#', '$', '%', '^', '&', '*', '(', ')'};
            return r1_sym[col];
        } else if (row == 2) {
            const char r2_sym[10] = {'-', '_', '=', '+', '[', ']', '{', '}', ':', ';'};
            return r2_sym[col];
        } else if (row == 3) {
            if (col == 0 || col == 8 || col == 9) return '\0';
            const char r3_sym[7] = {'\"', '\'', '<', '>', '/', '?', '.'};
            return r3_sym[col - 1];
        }
    }
    return '\0';
}

void CustomUiEngine::handleKeyInput(uint32_t key) {
    last_interaction_time = millis();
    if (screen_sleeping) {
        wakeUpScreen();
        return;
    }

    if (!page_focused) {
        if (key == CUSTOM_KEY_LEFT) {
            // 首页往左 -> 循环到最后一页
            current_page = (current_page == 0) ? PAGE_COUNT - 1 : current_page - 1;
            ui_needs_redraw = true;
            beep(25, 1);
        } else if (key == CUSTOM_KEY_RIGHT) {
            // 最后一页往右 -> 循环到首页
            current_page = (current_page == PAGE_COUNT - 1) ? 0 : current_page + 1;
            ui_needs_redraw = true;
            beep(25, 1);
        } else if (key == CUSTOM_KEY_ENTER) {
            if (current_page == PAGE_AI) {
                page_focused = true;
                if (!isRecording) {
                    postAppCommand(AppCommandType::START_RECORDING);
                } else {
                    postAppCommand(AppCommandType::STOP_RECORDING);
                }
                ui_needs_redraw = true;
            } else if (current_page == PAGE_SMART_HOME ||
                current_page == PAGE_DEVICES ||
                current_page == PAGE_LOGS || current_page == PAGE_SETTINGS ||
                current_page == PAGE_CAMERA ||
                current_page == PAGE_MUSIC) {
                page_focused = true;
                focus_index = 0;
                settings_focus_index = 0;
                settings_wifi_mode = WIFI_SUB_NONE;
                music_tier = MUSIC_TIER_PROGRESS; // 默认聚焦到第一层: 进度条
                music_adjusting_progress = false;
                music_adjusting_volume = false;
                ui_needs_redraw = true;
                beep(35, 2);
            }
        } else if (key == CUSTOM_KEY_ESC) {
            beep(20, 1);
        }
    } else {
        if (current_page == PAGE_SMART_HOME && light_brightness_mode) {
            // 灯光亮度调节模式: 旋钮/摇杆调亮度, 返回键退出; 未检测到设备时不可调
            if (key == CUSTOM_KEY_ESC) {
                light_brightness_mode = false;
                ui_needs_redraw = true;
                beep(30, 1);
            } else if ((key == CUSTOM_KEY_LEFT || key == CUSTOM_KEY_RIGHT ||
                        key == CUSTOM_KEY_UP || key == CUSTOM_KEY_DOWN) &&
                       smart_light_ack_state != -1) {
                static uint32_t last_light_adj = 0;
                if (millis() - last_light_adj >= 40) {
                    last_light_adj = millis();
                    int v = light_brightness;
                    v += (key == CUSTOM_KEY_RIGHT || key == CUSTOM_KEY_UP) ? 5 : -5;
                    if (v < 20) v = 20; // 起始点 20%: 20%=关
                    if (v > 100) v = 100;
                    sendLightBrightnessFast(v);
                    ui_needs_redraw = true;
                    beep(15, 1);
                }
            }
            return; // 亮度模式下其它按键不处理
        }
        if (key == CUSTOM_KEY_ESC) {
            if (current_page == PAGE_SETTINGS && settings_wifi_mode != WIFI_SUB_NONE) {
                if (settings_wifi_mode == WIFI_SUB_CONNECTING) {
                    settings_wifi_mode = WIFI_SUB_SCAN_LIST;
                } else if (settings_wifi_mode == WIFI_SUB_KEYBOARD) {
                    settings_wifi_mode = WIFI_SUB_SCAN_LIST;
                } else if (settings_wifi_mode == WIFI_SUB_SCAN_LIST) {
                    settings_wifi_mode = WIFI_SUB_NONE;
                }
                ui_needs_redraw = true;
                beep(30, 1);
                return;
            }
            if (current_page == PAGE_SETTINGS && settings_ir_menu_mode) {
                if (settings_ir_learning_mode) {
                    IRService::cancelPairing();
                    settings_ir_learning_mode = false;
                } else {
                    settings_ir_menu_mode = false;
                    IRService::setShielding(false);
                }
                ui_needs_redraw = true;
                beep(30, 1);
                return;
            }
            if (current_page == PAGE_SETTINGS && settings_adjust_mode) {
                settings_adjust_mode = false;
                ui_needs_redraw = true;
                beep(30, 1);
                return;
            }
            if (current_page == PAGE_MUSIC) {
                if (music_playlist_modal) {
                    music_playlist_modal = false;
                    music_tier = MUSIC_TIER_BUTTONS;
                    music_btn_idx = 5; // 聚焦回 [歌单] 按钮
                    ui_needs_redraw = true;
                    beep(30, 1);
                    return; // 仅退出歌单抽屉，保持页面聚焦
                }
                if (music_adjusting_progress) {
                    music_adjusting_progress = false;
                    music_tier = MUSIC_TIER_PROGRESS;
                    ui_needs_redraw = true;
                    beep(30, 1);
                    return; // 仅退出进度条快进调节，保持页面聚焦
                }
                if (music_adjusting_volume) {
                    music_adjusting_volume = false;
                    music_tier = MUSIC_TIER_VOLUME;
                    ui_needs_redraw = true;
                    beep(30, 1);
                    return; // 仅退出音量调节，保持页面聚焦
                }
            }
            if (current_page == PAGE_CAMERA) {
                CameraService::stop();
            }
            if (current_page == PAGE_SETTINGS) {
                settings_wifi_mode = WIFI_SUB_NONE;
            }
            page_focused = false;
            ui_needs_redraw = true;
            beep(30, 1);
            return;
        }

        if (current_page == PAGE_SMART_HOME) {
            if (key == CUSTOM_KEY_UP) {
                focus_index = (focus_index > 0) ? focus_index - 1 : 4;
                ui_needs_redraw = true;
                beep(20, 1);
            } else if (key == CUSTOM_KEY_DOWN) {
                focus_index = (focus_index < 4) ? focus_index + 1 : 0;
                ui_needs_redraw = true;
                beep(20, 1);
            } else if (key == CUSTOM_KEY_ENTER) {
                beep(40, 1);
                smart_home_show_result[focus_index] = true; // 强制为 true，保证每次点击均触发测量与实时更新！
                ui_needs_redraw = true;
                switch (focus_index) {
                    case 0:
                        // 灯光: 进入亮度模式, 两端关闭休眠并实时探测设备
                        light_brightness_mode = true;
                        smart_light_ack_state = -1; // 先置未知, 等回执确认
                        sendLightBrightnessFast(light_brightness);
                        break;
                    case 1: postAppCommand(AppCommandType::READ_ENVIRONMENT); break;
                    case 2: postAppCommand(AppCommandType::READ_TEMPERATURE_PROBE); break;
                    case 3: postAppCommand(AppCommandType::READ_AMBIENT_LIGHT); break;
                    case 4:
                        IRService::clearLastRecvCode(); // 重新进入红外检测，清空旧码等待最新遥控键
                        break;
                }
            }
        } else if (current_page == PAGE_AI) {
            if (key == CUSTOM_KEY_UP) {
                ai_scroll_lines++;
                ui_needs_redraw = true;
                beep(15, 1);
            } else if (key == CUSTOM_KEY_DOWN) {
                if (ai_scroll_lines > 0) ai_scroll_lines--;
                ui_needs_redraw = true;
                beep(15, 1);
            } else if (key == CUSTOM_KEY_ENTER) {
                static uint32_t last_ai_enter_time = 0;
                if (millis() - last_ai_enter_time > 150) {
                    last_ai_enter_time = millis();
                    if (isRecording) {
                        postAppCommand(AppCommandType::STOP_RECORDING);
                    } else {
                        postAppCommand(AppCommandType::START_RECORDING);
                    }
                    ui_needs_redraw = true;
                }
            }
        } else if (current_page == PAGE_DEVICES) {
            if (key == CUSTOM_KEY_UP) {
                device_scroll_lines++;
                ui_needs_redraw = true;
                beep(15, 1);
            } else if (key == CUSTOM_KEY_DOWN) {
                if (device_scroll_lines > 0) device_scroll_lines--;
                ui_needs_redraw = true;
                beep(15, 1);
            } else if (key == CUSTOM_KEY_ENTER) {
                beep(40, 2);
                postAppCommand(AppCommandType::SCAN_DEVICES);
            }
        } else if (current_page == PAGE_LOGS) {
            if (key == CUSTOM_KEY_UP) {
                log_scroll_lines++;
                ui_needs_redraw = true;
                beep(15, 1);
            } else if (key == CUSTOM_KEY_DOWN) {
                if (log_scroll_lines > 0) log_scroll_lines--;
                ui_needs_redraw = true;
                beep(15, 1);
            }
        } else if (current_page == PAGE_SETTINGS) {
            if (settings_wifi_mode == WIFI_SUB_SCAN_LIST) {
                int total_count = 1 + (int)scanned_wifis.size();
                int max_visible = isLandscape() ? 4 : 5;
                if (key == CUSTOM_KEY_UP) {
                    if (wifi_list_focus_index > 0) {
                        wifi_list_focus_index--;
                        if (wifi_list_focus_index < wifi_list_scroll_top) {
                            wifi_list_scroll_top = wifi_list_focus_index;
                        }
                    } else {
                        wifi_list_focus_index = (total_count > 0) ? total_count - 1 : 0;
                        wifi_list_scroll_top = std::max(0, total_count - max_visible);
                    }
                    ui_needs_redraw = true;
                    beep(20, 1);
                } else if (key == CUSTOM_KEY_DOWN) {
                    if (wifi_list_focus_index < total_count - 1) {
                        wifi_list_focus_index++;
                        if (wifi_list_focus_index >= wifi_list_scroll_top + max_visible) {
                            wifi_list_scroll_top = wifi_list_focus_index - max_visible + 1;
                        }
                    } else {
                        wifi_list_focus_index = 0;
                        wifi_list_scroll_top = 0;
                    }
                    ui_needs_redraw = true;
                    beep(20, 1);
                } else if (key == CUSTOM_KEY_ENTER) {
                    beep(40, 1);
                    if (wifi_list_focus_index == 0) {
                        startWifiScan();
                    } else {
                        int idx = wifi_list_focus_index - 1;
                        if (idx >= 0 && idx < (int)scanned_wifis.size()) {
                            selected_wifi_ssid = scanned_wifis[idx].ssid;
                            if (scanned_wifis[idx].is_open) {
                                wifi_password_input = "";
                                settings_wifi_mode = WIFI_SUB_CONNECTING;
                                wifi_connect_pending = true;
                                wifi_connect_result = 0;
                            } else {
                                wifi_password_input = "";
                                kb_row = 0;
                                kb_col = 0;
                                kb_symbol_mode = false;
                                kb_upper_mode = false;
                                settings_wifi_mode = WIFI_SUB_KEYBOARD;
                            }
                            ui_needs_redraw = true;
                        }
                    }
                } else if (key == CUSTOM_KEY_ESC) {
                    stopWifiScan();
                    settings_wifi_mode = WIFI_SUB_NONE;
                    ui_needs_redraw = true;
                    beep(30, 1);
                }
            } else if (settings_wifi_mode == WIFI_SUB_KEYBOARD) {
                if (key == CUSTOM_KEY_UP) {
                    kb_row = (kb_row > 0) ? kb_row - 1 : 3;
                    ui_needs_redraw = true;
                    beep(15, 1);
                } else if (key == CUSTOM_KEY_DOWN) {
                    kb_row = (kb_row < 3) ? kb_row + 1 : 0;
                    ui_needs_redraw = true;
                    beep(15, 1);
                } else if (key == CUSTOM_KEY_LEFT) {
                    kb_col = (kb_col > 0) ? kb_col - 1 : 9;
                    ui_needs_redraw = true;
                    beep(15, 1);
                } else if (key == CUSTOM_KEY_RIGHT) {
                    kb_col = (kb_col < 9) ? kb_col + 1 : 0;
                    ui_needs_redraw = true;
                    beep(15, 1);
                } else if (key == CUSTOM_KEY_ENTER) {
                    if (kb_row == 3 && kb_col == 0) {
                        kb_symbol_mode = !kb_symbol_mode;
                        beep(25, 1);
                    } else if (kb_row == 3 && kb_col == 8) {
                        if (wifi_password_input.length() > 0) {
                            wifi_password_input.remove(wifi_password_input.length() - 1);
                        }
                        beep(25, 1);
                    } else if (kb_row == 3 && kb_col == 9) {
                        settings_wifi_mode = WIFI_SUB_CONNECTING;
                        wifi_connect_pending = true;
                        wifi_connect_result = 0;
                        beep(40, 1);
                    } else if (kb_row == 2 && kb_col == 9 && !kb_symbol_mode) {
                        kb_upper_mode = !kb_upper_mode;
                        beep(25, 1);
                    } else {
                        char ch = getKbChar(kb_row, kb_col, kb_symbol_mode, kb_upper_mode);
                        if (ch != '\0' && wifi_password_input.length() < 64) {
                            wifi_password_input += ch;
                            beep(20, 1);
                        }
                    }
                    ui_needs_redraw = true;
                } else if (key == CUSTOM_KEY_ESC) {
                    settings_wifi_mode = WIFI_SUB_SCAN_LIST;
                    ui_needs_redraw = true;
                    beep(30, 1);
                }
            } else if (settings_wifi_mode == WIFI_SUB_CONNECTING) {
                if (wifi_connect_result == 1) {
                    if (key == CUSTOM_KEY_ENTER || key == CUSTOM_KEY_ESC) {
                        settings_wifi_mode = WIFI_SUB_NONE;
                        ui_needs_redraw = true;
                        beep(30, 1);
                    }
                } else if (wifi_connect_result == 2) {
                    if (key == CUSTOM_KEY_ENTER) {
                        settings_wifi_mode = WIFI_SUB_KEYBOARD;
                        ui_needs_redraw = true;
                        beep(30, 1);
                    } else if (key == CUSTOM_KEY_ESC) {
                        settings_wifi_mode = WIFI_SUB_SCAN_LIST;
                        ui_needs_redraw = true;
                        beep(30, 1);
                    }
                }
            } else if (settings_ir_menu_mode) {
                if (settings_ir_learning_mode) {
                    if (key == CUSTOM_KEY_ESC) {
                        IRService::cancelPairing();
                        settings_ir_learning_mode = false;
                        ui_needs_redraw = true;
                        beep(30, 1);
                    }
                } else {
                    if (key == CUSTOM_KEY_UP) {
                        settings_ir_focus_index = (settings_ir_focus_index > 0) ? settings_ir_focus_index - 1 : 9;
                        ui_needs_redraw = true;
                        beep(20, 1);
                    } else if (key == CUSTOM_KEY_DOWN) {
                        settings_ir_focus_index = (settings_ir_focus_index < 9) ? settings_ir_focus_index + 1 : 0;
                        ui_needs_redraw = true;
                        beep(20, 1);
                    } else if (key == CUSTOM_KEY_LEFT || key == CUSTOM_KEY_RIGHT) {
                        if (settings_ir_focus_index == 0) {
                            int curP = IRService::getActiveProfile();
                            if (key == CUSTOM_KEY_LEFT) curP = (curP > 0) ? curP - 1 : 2;
                            else curP = (curP < 2) ? curP + 1 : 0;
                            IRService::setActiveProfile(curP);
                            ui_needs_redraw = true;
                            beep(10, 1);
                        }
                    } else if (key == CUSTOM_KEY_ENTER) {
                        beep(40, 1);
                        if (settings_ir_focus_index == 0) {
                            int curP = (IRService::getActiveProfile() + 1) % 3;
                            IRService::setActiveProfile(curP);
                            ui_needs_redraw = true;
                        } else if (settings_ir_focus_index == 1) {
                            IRService::startSequentialPairing();
                            settings_ir_learning_mode = true;
                            ui_needs_redraw = true;
                        } else if (settings_ir_focus_index >= 2 && settings_ir_focus_index <= 9) {
                            IRPairingState stateMap[] = {
                                IR_PAIRING_UP, IR_PAIRING_DOWN, IR_PAIRING_LEFT,
                                IR_PAIRING_RIGHT, IR_PAIRING_ENTER, IR_PAIRING_ESC,
                                IR_PAIRING_STANDBY, IR_PAIRING_POWEROFF
                            };
                            IRService::startPairingKey(stateMap[settings_ir_focus_index - 2]);
                            settings_ir_learning_mode = true;
                            ui_needs_redraw = true;
                        }
                    }
                }
            } else if (settings_adjust_mode) {
                // 在滑条调节模式下，左右键直接更改亮度或息屏
                if (key == CUSTOM_KEY_LEFT) {
                    if (settings_focus_index == 3) {
                        setScreenBrightness(screen_brightness - 10);
                        beep(10, 1);
                    } else if (settings_focus_index == 4) {
                        const int gears[] = {0, 1, 5, 10, 15, 20, 25, 30};
                        int current_idx = 0;
                        for (int k = 0; k < 8; k++) if (gears[k] == auto_sleep_time) current_idx = k;
                        if (current_idx > 0) setAutoSleepTime(gears[current_idx - 1]);
                        beep(10, 1);
                    } else if (settings_focus_index == 5) {
                        setTtsVolume(tts_volume - 10);
                        beep(10, 1);
                    }
                } else if (key == CUSTOM_KEY_RIGHT) {
                    if (settings_focus_index == 3) {
                        setScreenBrightness(screen_brightness + 10);
                        beep(10, 1);
                    } else if (settings_focus_index == 4) {
                        const int gears[] = {0, 1, 5, 10, 15, 20, 25, 30};
                        int current_idx = 0;
                        for (int k = 0; k < 8; k++) if (gears[k] == auto_sleep_time) current_idx = k;
                        if (current_idx < 7) setAutoSleepTime(gears[current_idx + 1]);
                        beep(10, 1);
                    } else if (settings_focus_index == 5) {
                        setTtsVolume(tts_volume + 10);
                        beep(10, 1);
                    }
                }
            } else {
                if (key == CUSTOM_KEY_UP) {
                    settings_focus_index = (settings_focus_index > 0) ? settings_focus_index - 1 : 7;
                    ui_needs_redraw = true;
                    beep(20, 1);
                } else if (key == CUSTOM_KEY_DOWN) {
                    settings_focus_index = (settings_focus_index < 7) ? settings_focus_index + 1 : 0;
                    ui_needs_redraw = true;
                    beep(20, 1);
                } else if (key == CUSTOM_KEY_ENTER) {
                    beep(40, 1);
                    if (settings_focus_index == 0) {
                        setScreenMirror(!screen_mirror_enabled);
                    } else if (settings_focus_index == 1) {
                        setSystemSoundEnabled(!system_sound_enabled);
                    } else if (settings_focus_index == 2) {
                        setAutoRotateEnabled(!auto_rotate_enabled);
                    } else if (settings_focus_index >= 3 && settings_focus_index <= 5) {
                        // 进入调节模式
                        settings_adjust_mode = true;
                        ui_needs_redraw = true;
                    } else if (settings_focus_index == 6) {
                        // 进入红外对码二级菜单
                        settings_ir_menu_mode = true;
                        settings_ir_focus_index = 0;
                        settings_ir_learning_mode = false;
                        IRService::setShielding(true);
                        ui_needs_redraw = true;
                    } else if (settings_focus_index == 7) {
                        // 进入 WiFi 设置二级界面
                        startWifiScan();
                        settings_wifi_mode = WIFI_SUB_SCAN_LIST;
                        ui_needs_redraw = true;
                    }
                }
            }
        } else if (current_page == PAGE_CAMERA) {
            if (key == CUSTOM_KEY_ENTER) {
                beep(40, 1);
                if (focus_index == 0) {
                    const CameraService::State s = CameraService::state();
                    if (s == CameraService::State::IDLE || s == CameraService::State::FAILED) {
                        CameraService::clearFreezeFrame(); // 开启新连接前清理旧快照
                        ai_vision_result = "";             // 清空旧识别结果
                        CameraService::start();
                    } else {
                        CameraService::stop();
                    }
                } else if (focus_index == 1) {
                    // 开始识图 (互斥拦截: 未开启传输时禁止触发)
                    if (CameraService::state() != CameraService::State::STREAMING) {
                        ai_vision_result = "未开启传输，无法识图！请先点击连接。";
                        beep(20, 2); // 警告双哔声
                    } else {
                        // 1. 抓取最新帧生成冻结快照
                        // 2. 断开 STM32 摄像头连接，释放 SRAM DMA 内存与外设
                        // 3. 投递异步大模型多模态识别与语音播报指令
                        CameraService::captureFreezeFrame();
                        CameraService::stop(); // 立即断开释放内存
                        ai_vision_result = "正在进行图像编码与云端识别...";
                        postAppCommand(AppCommandType::START_VISION_AI);
                    }
                } else if (focus_index == 2) {
                    CameraService::toggleFlash();
                }
                ui_needs_redraw = true;
            } else if (key == CUSTOM_KEY_UP || key == CUSTOM_KEY_LEFT) {
                focus_index = (focus_index > 0) ? focus_index - 1 : 2;
                ui_needs_redraw = true;
                beep(20, 1);
            } else if (key == CUSTOM_KEY_DOWN || key == CUSTOM_KEY_RIGHT) {
                focus_index = (focus_index < 2) ? focus_index + 1 : 0;
                ui_needs_redraw = true;
                beep(20, 1);
            }
        } else if (current_page == PAGE_MUSIC) {
            if (music_playlist_modal) {
                // 歌单抽屉已打开时的交互 (严格单首逐曲滚动，长按时平滑倍速连发，绝不跳步)
                int count = MusicPlayerService::getPlaylistCount();
                int visible_count = isLandscape() ? 6 : 7;

                if (key == CUSTOM_KEY_UP || key == CUSTOM_KEY_LEFT) {
                    if (count > 0) {
                        music_playlist_focus = (music_playlist_focus > 0) ? (music_playlist_focus - 1) : (count - 1);
                        if (music_playlist_focus < music_playlist_scroll) {
                            music_playlist_scroll = music_playlist_focus;
                        } else if (music_playlist_focus >= music_playlist_scroll + visible_count) {
                            music_playlist_scroll = (music_playlist_focus >= visible_count) ? (music_playlist_focus - visible_count + 1) : 0;
                        }
                    }
                    beep(8, 1);
                    ui_needs_redraw = true;
                } else if (key == CUSTOM_KEY_DOWN || key == CUSTOM_KEY_RIGHT) {
                    if (count > 0) {
                        music_playlist_focus = (music_playlist_focus < count - 1) ? (music_playlist_focus + 1) : 0;
                        if (music_playlist_focus >= music_playlist_scroll + visible_count) {
                            music_playlist_scroll = music_playlist_focus - visible_count + 1;
                        } else if (music_playlist_focus < music_playlist_scroll) {
                            music_playlist_scroll = music_playlist_focus;
                        }
                    }
                    beep(8, 1);
                    ui_needs_redraw = true;
                } else if (key == CUSTOM_KEY_ENTER) {
                    // 选中并播放该曲目
                    music_btn_idx = 3; // 🌟 焦点自动归位到 [播/暂] 按钮
                    MusicPlayerService::sendPlay(music_playlist_focus);
                    music_playlist_modal = false;
                    beep(40, 1);
                    ui_needs_redraw = true;
                } else if (key == CUSTOM_KEY_ESC) {
                    // 关闭歌单抽屉
                    music_playlist_modal = false;
                    beep(20, 1);
                    ui_needs_redraw = true;
                }
            } else if (music_adjusting_progress) {
                // 🌟 处于进度条调节交互中 (摇杆左右快退/快进，500ms 停手防抖原子提交)
                if (key == CUSTOM_KEY_LEFT) {
                    s_preview_seek_sec -= 5;
                    if (s_preview_seek_sec < 0) s_preview_seek_sec = 0;
                    s_seek_debounce_deadline = millis() + 500;
                    s_seek_pending = true;
                    MusicPlayerService::setMuteState(true);
                    beep(15, 1);
                    ui_needs_redraw = true;
                } else if (key == CUSTOM_KEY_RIGHT) {
                    s_preview_seek_sec += 5;
                    int tot = MusicPlayerService::getTotalSeconds();
                    if (tot > 0 && s_preview_seek_sec > tot) s_preview_seek_sec = tot;
                    s_seek_debounce_deadline = millis() + 500;
                    s_seek_pending = true;
                    MusicPlayerService::setMuteState(true);
                    beep(15, 1);
                    ui_needs_redraw = true;
                } else if (key == CUSTOM_KEY_ENTER || key == CUSTOM_KEY_ESC) {
                    if (s_seek_pending) {
                        MusicPlayerService::sendSeek(s_preview_seek_sec);
                        s_seek_pending = false;
                    }
                    MusicPlayerService::setMuteState(false);
                    music_adjusting_progress = false;
                    beep(30, 1);
                    ui_needs_redraw = true;
                }
            } else if (music_adjusting_volume) {
                // 🌟 处于音量调节交互中 (摇杆左右调节音量)
                if (key == CUSTOM_KEY_LEFT) {
                    int v = MusicPlayerService::getVolume() - 5;
                    MusicPlayerService::sendVolume(v < 0 ? 0 : v);
                    beep(15, 1);
                    ui_needs_redraw = true;
                } else if (key == CUSTOM_KEY_RIGHT) {
                    int v = MusicPlayerService::getVolume() + 5;
                    MusicPlayerService::sendVolume(v > 100 ? 100 : v);
                    beep(15, 1);
                    ui_needs_redraw = true;
                } else if (key == CUSTOM_KEY_ENTER || key == CUSTOM_KEY_ESC) {
                    music_adjusting_volume = false;
                    beep(30, 1);
                    ui_needs_redraw = true;
                }
            } else {
                // 🌟 三层级焦点循环导航: 0=进度条(上), 1=5个按键(中), 2=音量条(下)
                if (key == CUSTOM_KEY_UP) {
                    // 向上循环切换层级: 音量 -> 按键 -> 进度条 -> 音量
                    music_tier = (music_tier == MUSIC_TIER_PROGRESS) ? MUSIC_TIER_VOLUME : static_cast<MusicUiTier>(music_tier - 1);
                    beep(20, 1);
                    ui_needs_redraw = true;
                } else if (key == CUSTOM_KEY_DOWN) {
                    // 向下循环切换层级: 进度条 -> 按键 -> 音量 -> 进度条
                    music_tier = (music_tier == MUSIC_TIER_VOLUME) ? MUSIC_TIER_PROGRESS : static_cast<MusicUiTier>(music_tier + 1);
                    beep(20, 1);
                    ui_needs_redraw = true;
                } else if (key == CUSTOM_KEY_LEFT) {
                    if (music_tier == MUSIC_TIER_BUTTONS) {
                        music_btn_idx = (music_btn_idx > 0) ? music_btn_idx - 1 : 5;
                        beep(20, 1);
                        ui_needs_redraw = true;
                    }
                } else if (key == CUSTOM_KEY_RIGHT) {
                    if (music_tier == MUSIC_TIER_BUTTONS) {
                        music_btn_idx = (music_btn_idx < 5) ? music_btn_idx + 1 : 0;
                        beep(20, 1);
                        ui_needs_redraw = true;
                    }
                } else if (key == CUSTOM_KEY_ENTER) {
                    beep(40, 1);
                    if (music_tier == MUSIC_TIER_PROGRESS) {
                        // 选中进度条 -> 进入进度条交互调参模式
                        music_adjusting_progress = true;
                        s_preview_seek_sec = MusicPlayerService::getCurrentSeconds();
                        s_seek_pending = false;
                    } else if (music_tier == MUSIC_TIER_BUTTONS) {
                        if (music_btn_idx == 0) {
                            if (MusicPlayerService::isConnected()) {
                                MusicPlayerService::sendDisconnect();
                            } else {
                                MusicPlayerService::sendConnect();
                            }
                        } else if (music_btn_idx == 1) {
                            MusicPlayerService::sendTogglePlayMode();
                        } else if (music_btn_idx == 2) {
                            MusicPlayerService::sendPrev();
                        } else if (music_btn_idx == 3) {
                            MusicPlayerService::sendTogglePlayPause();
                        } else if (music_btn_idx == 4) {
                            MusicPlayerService::sendNext();
                        } else if (music_btn_idx == 5) {
                            int vcount = isLandscape() ? 6 : 7;
                            music_playlist_modal = true;
                            music_playlist_focus = MusicPlayerService::getCurrentTrackIndex();
                            music_playlist_scroll = (music_playlist_focus >= vcount) ? music_playlist_focus - vcount + 1 : 0;
                        }
                    } else if (music_tier == MUSIC_TIER_VOLUME) {
                        // 选中音量条 -> 进入音量交互调参模式
                        music_adjusting_volume = true;
                    }
                    ui_needs_redraw = true;
                } else if (key == CUSTOM_KEY_ESC) {
                    page_focused = false;
                    beep(20, 1);
                    ui_needs_redraw = true;
                }
            }
        }
    }
}

void CustomUiEngine::handleKeyRelease(uint32_t key) {
    // 录音逻辑已改为“点击切换”，此处无需再处理松开停止录音
    if (screen_sleeping) return;
}


void CustomUiEngine::renderCamera() {
    renderHeader("AI 识图");

    const CameraService::State s = CameraService::state();
    const bool is_streaming = (s == CameraService::State::STREAMING);
    const bool btn0_focused = page_focused && (focus_index == 0);
    const bool btn1_focused = page_focused && (focus_index == 1);
    const bool btn2_focused = page_focused && (focus_index == 2);

    const char* label = "连接";
    uint16_t color = CustomUiTheme::PRIMARY_DARK;
    switch (s) {
        case CameraService::State::WAKING:
            label = "唤醒中";
            color = CustomUiTheme::WARNING;
            break;
        case CameraService::State::CONNECTING:
            label = "连接中";
            color = CustomUiTheme::WARNING;
            break;
        case CameraService::State::STREAMING:
            label = "断开";
            color = CustomUiTheme::DANGER;
            break;
        case CameraService::State::FAILED:
            label = "重试";
            color = CustomUiTheme::WARNING;
            break;
        default:
            break;
    }

    // 🌟 识图按钮互斥视觉：只有开启传输 (STREAMING) 时才点亮金色，未开启传输时显示灰色禁用态
    const char* ai_btn_label = "识图";
    uint16_t ai_btn_color = is_streaming ? CustomUiTheme::FOCUS_GOLD : CustomUiTheme::SURFACE_ALT;

    const char* flash_label = CameraService::isFlashOn() ? "关灯" : "补光";
    uint16_t flash_color = CameraService::isFlashOn() ? CustomUiTheme::WARNING : CustomUiTheme::PRIMARY_DARK;

    auto drawFrame = [&](int fx, int fy, int fw, int fh) {
        drawCard(fx, fy, fw, fh);
        const int img_x = fx + (fw - CameraService::FRAME_W) / 2;
        const int img_y = fy + (fh - CameraService::FRAME_H) / 2;
        
        // Only clear the actual image area
        spr.fillRect(img_x, img_y, CameraService::FRAME_W, CameraService::FRAME_H, TFT_BLACK);
        uint16_t* frame = CameraService::lockFrame();
        if (frame) {
            spr.pushImage(img_x, img_y, CameraService::FRAME_W, CameraService::FRAME_H, frame);
            CameraService::unlockFrame();
        } else {
            drawChineseTextCentered("无画面", img_x,
                                    img_y + CameraService::FRAME_H / 2 - 7,
                                    CameraService::FRAME_W, CustomUiTheme::MUTED, TFT_BLACK, 2);
        }
    };

    auto drawWrappedResult = [&](int tx, int ty, int max_w, int max_h) {
        String fullText = ai_vision_result;
        if (fullText.length() == 0) fullText = "等待拍摄识图...";
        
        u8f.setFont(u8g2_font_wqy13_t_gb2312a);
        std::vector<String> lines;
        String curLine = "";
        
        for (size_t i = 0; i < fullText.length(); i++) {
            char c = fullText[i];
            if (c == '\n') {
                lines.push_back(curLine);
                curLine = "";
                continue;
            }
            if (c == '\r') continue;
            
            int charLen = 1;
            if ((c & 0x80) == 0) charLen = 1;
            else if ((c & 0xE0) == 0xC0) charLen = 2;
            else if ((c & 0xF0) == 0xE0) charLen = 3;
            else if ((c & 0xF8) == 0xF0) charLen = 4;
            
            String nextChar = fullText.substring(i, i + charLen);
            i += (charLen - 1);
            
            if (u8f.getUTF8Width((curLine + nextChar).c_str()) > max_w) {
                if (curLine.length() > 0) {
                    lines.push_back(curLine);
                    curLine = nextChar;
                } else {
                    lines.push_back(nextChar);
                    curLine = "";
                }
            } else {
                curLine += nextChar;
            }
        }
        if (curLine.length() > 0) lines.push_back(curLine);
        
        int line_y = ty;
        for (size_t i = 0; i < lines.size() && (line_y + 13 <= ty + max_h); i++) {
            drawChineseText(lines[i].c_str(), tx, line_y, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, 0);
            line_y += 16;
        }
    };

    if (isLandscape()) {
        // ===== 横屏布局 (320x240 / 280x240) =====
        // 左侧: 画面框 (X=8, W=168, Y=34~160)
        drawFrame(8, 34, 168, 126);

        // 下方 3 个按钮：三个按钮尺寸完全一致 (52x34)，总宽度 168px 与上方画面完全对齐 (8+52+6+52+6+52=176)
        drawActionButton(8,   176, 52, 34, label, color, btn0_focused, 2);
        drawActionButton(66,  176, 52, 34, ai_btn_label, ai_btn_color, btn1_focused, 2);
        drawActionButton(124, 176, 52, 34, flash_label, flash_color, btn2_focused, 2);

        // 右侧: AI 识图结果大卡片 (拉到与底栏对齐)
        int card_w = screenWidth() - 186;
        if (card_w < 80) card_w = 80;
        int card_h = screenHeight() - 34 - 14;
        drawCard(180, 34, card_w, card_h);
        drawChineseText("AI 识图结果:", 188, 40, CustomUiTheme::FOCUS_GOLD, CustomUiTheme::SURFACE, 2);
        spr.drawFastHLine(184, 58, card_w - 8, CustomUiTheme::BORDER);
        drawWrappedResult(188, 64, card_w - 16, card_h - 38);
    } else {
        // ===== 竖屏布局 (240x280 / 240x320) =====
        // 上部: 画面框卡片（X=10, 宽度220，与下方卡片完全对齐；内部 160x120 画面居中显示）
        drawFrame(10, 34, 220, 126);

        // 中部: 3 个动作控制按钮 (三个按钮尺寸完全一致: 68x30，间距8，总宽220与上方卡片严格对齐，文字双轴绝对居中)
        drawActionButton(10,  164, 68, 30, label, color, btn0_focused, 2);
        drawActionButton(86,  164, 68, 30, ai_btn_label, ai_btn_color, btn1_focused, 2);
        drawActionButton(162, 164, 68, 30, flash_label, flash_color, btn2_focused, 2);

        // 下部: 专属 AI 识图结果卡片区 (X=10, 宽度220，与画面卡片上下完美齐平)
        int card_h = screenHeight() - 199 - 14;
        if (card_h < 60) card_h = 60;
        drawCard(10, 199, 220, card_h);
        drawChineseText("AI 识图结果:", 18, 203, CustomUiTheme::FOCUS_GOLD, CustomUiTheme::SURFACE, 2);
        spr.drawFastHLine(14, 221, 212, CustomUiTheme::BORDER);
        
        // 多行 AI 回答自适应展示
        drawWrappedResult(18, 226, 204, card_h - 32);
    }
}

void CustomUiEngine::renderPage() {
    static int last_rendered_page = -1;
    static bool last_rendered_landscape = false;
    static bool camera_ui_drawn = false;
    static uint32_t last_cam_rev = 0;

    bool current_landscape = isLandscape();

    if (last_rendered_page != current_page || last_rendered_landscape != current_landscape) {
        if (last_rendered_page == PAGE_SMART_HOME && last_rendered_page != current_page) {
            for (int i = 0; i < 5; i++) smart_home_show_result[i] = false;
        }
        if (last_rendered_page == PAGE_CAMERA && last_rendered_page != current_page) {
            CameraService::stop();
        }
        
        if (last_rendered_landscape != current_landscape) {
            tft->fillScreen(CustomUiTheme::SURFACE);
        }
        
        last_rendered_page = current_page;
        last_rendered_landscape = current_landscape;
        camera_ui_drawn = false;
    }

    static int last_focus_index = -1;
    static bool last_page_focused = false;
    static bool last_flash_state = false;
    static CameraService::State last_camera_state = CameraService::State::IDLE;

    if (current_page == PAGE_CAMERA) {
        bool focus_changed = (last_focus_index != focus_index) ||
                             (last_page_focused != page_focused) ||
                             (last_flash_state != CameraService::isFlashOn()) ||
                             (last_camera_state != CameraService::state());
        if (focus_changed) {
            last_focus_index = focus_index;
            last_page_focused = page_focused;
            last_flash_state = CameraService::isFlashOn();
            last_camera_state = CameraService::state();
            camera_ui_drawn = false; // 焦点/状态变化时，强行触发一次 UI 重新全量绘制
        }
    }

    // 摄像头快速通道: UI已绘制后新帧直接推TFT跳过Sprite
    if (current_page == PAGE_CAMERA && camera_ui_drawn) {
        if (CameraService::hasQueuedFrames()) {
            uint16_t* frame = CameraService::lockFrame();
            if (frame) {
                int fx, fy, fw, fh;
                if (isLandscape()) { fx = 8; fy = 34; fw = 168; fh = 126; }
                else { fx = 10; fy = 34; fw = 220; fh = 126; }
                int ix = fx+(fw-CameraService::FRAME_W)/2;
                int iy = fy+(fh-CameraService::FRAME_H)/2;
                const bool fits = ix >= 0 && iy >= 0 &&
                                  ix + CameraService::FRAME_W <= screenWidth() &&
                                  iy + CameraService::FRAME_H <= screenHeight();
                if (fits) {
                    tft->startWrite();
                    tft->setAddrWindow(ix, iy, CameraService::FRAME_W, CameraService::FRAME_H);
                    uint32_t push_start = millis();
                    tft->pushPixels(frame, CameraService::FRAME_W * CameraService::FRAME_H);
                    uint32_t push_ms = millis() - push_start;
                    tft->endWrite();
                    
                    static uint32_t max_push_ms = 0;
                    if (push_ms > max_push_ms) max_push_ms = push_ms;
                    
                    static uint32_t last_tft_profile = 0;
                    if (millis() - last_tft_profile > 1000) {
                        last_tft_profile = millis();
                        Serial.printf("[PROFILER] TFT PushPixels Max 耗时: %lu ms\r\n", max_push_ms);
                        max_push_ms = 0;
                    }
                    
                    // --- 实时 FPS 统计与显示 ---
                    fps_frames++;
                    uint32_t now = millis();
                    if (now - fps_last_time >= 1000) {
                        current_fps = fps_frames;
                        fps_frames = 0;
                        fps_last_time = now;
                        Serial.printf("[ESP32-FPS] 收 %lu (好 %lu, 坏 %lu) | 显 %d fps\r\n", 
                                      CameraService::rxTotalFps(), 
                                      CameraService::rxGoodFps(), 
                                      CameraService::rxBadFps(), 
                                      current_fps);
                    }
                    // ----------------------------
                }
                CameraService::unlockFrame();
                if (CameraService::hasQueuedFrames()) {
                    CustomUiEngine::notifyUiNeedsUpdate();
                }
                if (fits) return; // 快速通道完成; 放不下时回退到 Sprite 全量渲染
            }
            return;
        }
    }

    spr.fillSprite(CustomUiTheme::BG_TOP);
    
    switch (current_page) {
        case PAGE_HOME:       renderHome(); break;
        case PAGE_WEATHER:    renderWeather(); break;
        case PAGE_AI:         renderAi(); break;
        case PAGE_SMART_HOME: renderSmartHome(); break;
        case PAGE_DEVICES:    renderDevices(); break;
        case PAGE_CAMERA:     renderCamera(); break;
        case PAGE_MUSIC:      renderMusic(); break;
        case PAGE_SYSTEM:     renderSystem(); break;
        case PAGE_SETTINGS:   renderSettings(); break;
        case PAGE_LOGS:       renderLogs(); break;
    }
    
    renderPageIndicator(current_page, PAGE_COUNT);
    const uint32_t push_start = millis();
    spr.pushSprite(0, 0);
    const uint32_t push_ms = millis() - push_start;
    if (push_ms > 70) {
        Serial.printf("[TFT] 推屏异常耗时 %lu ms (page=%d)\r\n",
                      (unsigned long)push_ms, current_page);
    }
    if (current_page == PAGE_CAMERA) {
        camera_ui_drawn = true;
        last_cam_rev = CameraService::frameRevision();
    }
}

void CustomUiEngine::renderHeader(const char* title) {
    int w = screenWidth();
    drawChineseTextCentered(title, 0, 6, w, CustomUiTheme::TEXT, CustomUiTheme::BG_TOP, true);
    spr.drawFastHLine(10, 28, w - 20, CustomUiTheme::BORDER);
}

void CustomUiEngine::renderPageIndicator(int current, int total) {
    int w = screenWidth();
    int h = screenHeight();
    int start_x = w / 2 - (total * 8) / 2;
    for (int i = 0; i < total; i++) {
        uint16_t color = (i == current) ? CustomUiTheme::PRIMARY : CustomUiTheme::SURFACE_ALT;
        int radius = (i == current) ? 3 : 2;
        spr.fillCircle(start_x + i * 8, h - 8, radius, color);
    }
}

void CustomUiEngine::drawCard(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t bg_color, uint16_t border_color) {
    spr.fillRoundRect(x, y, w, h, 6, bg_color);
    spr.drawRoundRect(x, y, w, h, 6, border_color);
}

void CustomUiEngine::drawProgressBar(int16_t x, int16_t y, int16_t w, int16_t h, uint32_t pct, uint16_t bar_color) {
    spr.fillRoundRect(x, y, w, h, h / 2, CustomUiTheme::SURFACE_ALT);
    if (pct > 100) pct = 100;
    int fill_w = (w * pct) / 100;
    if (fill_w > 0) {
        spr.fillRoundRect(x, y, fill_w, h, h / 2, bar_color);
    }
}

void CustomUiEngine::drawActionButton(int16_t x, int16_t y, int16_t w, int16_t h, const char* label, uint16_t bg_color, bool focused, int font_level) {
    uint16_t border = focused ? CustomUiTheme::FOCUS_GOLD : CustomUiTheme::BORDER;
    spr.fillRoundRect(x, y, w, h, 8, bg_color);
    spr.drawRoundRect(x, y, w, h, 8, border);
    if (focused) {
        spr.drawRoundRect(x - 1, y - 1, w + 2, h + 2, 9, CustomUiTheme::FOCUS_GOLD);
    }
    
    // 🌟 几何双轴绝对正中居中：基于字体基准线与行高精确对齐
    u8f.setFontMode(1);
    u8f.setForegroundColor(CustomUiTheme::TEXT);
    u8f.setBackgroundColor(bg_color);
    int font_h = 13;
    int ascent = 11;
    if (font_level == 1) {
        u8f.setFont(u8g2_font_wqy16_t_gb2312a);
        font_h = 16; ascent = 14;
    } else if (font_level == 2) {
        u8f.setFont(u8g2_font_wqy14_t_gb2312a);
        font_h = 14; ascent = 12;
    } else if (font_level == 3) {
        u8f.setFont(u8g2_font_wqy15_t_gb2312a);
        font_h = 15; ascent = 13;
    } else {
        u8f.setFont(u8g2_font_wqy13_t_gb2312a);
        font_h = 13; ascent = 11;
    }
    int tWidth = u8f.getUTF8Width(label);
    int drawX = x + (w - tWidth) / 2;
    int drawY = y + (h - font_h) / 2 + ascent;
    u8f.setCursor(drawX < x ? x : drawX, drawY);
    u8f.print(label);
}

void CustomUiEngine::renderSettings() {
    if (settings_wifi_mode == WIFI_SUB_SCAN_LIST) {
        renderWifiScanList();
        return;
    } else if (settings_wifi_mode == WIFI_SUB_KEYBOARD) {
        renderWifiKeyboard();
        return;
    } else if (settings_wifi_mode == WIFI_SUB_CONNECTING) {
        renderWifiConnecting();
        return;
    }

    if (settings_ir_menu_mode) {
        if (settings_ir_learning_mode) {
            renderHeader("红外对码学习");
            bool lan = isLandscape();
            int cw = lan ? 296 : 216;
            int y1 = lan ? 48 : 50, y2 = lan ? 74 : 85, y3 = lan ? 98 : 118;
            int y4 = lan ? 124 : 150, y5 = lan ? 148 : 180, y6 = lan ? 172 : 205, y7 = lan ? 198 : 235;
            drawCard(12, 40, cw, lan ? 188 : 258);
            
            bool isSeq = IRService::isSequentialPairing();
            IRPairingState state = IRService::getPairingState();
            
            const char* keyNames[] = {
                "上 (UP)", "下 (DOWN)", "左 (LEFT)", "右 (RIGHT)",
                "确认 (ENTER)", "退出 (ESC)", "息屏待机 (STANDBY)", "深度关机 (POWER OFF)"
            };
            int keyIndex = 0;
            if (state == IR_PAIRING_UP) keyIndex = 0;
            else if (state == IR_PAIRING_DOWN) keyIndex = 1;
            else if (state == IR_PAIRING_LEFT) keyIndex = 2;
            else if (state == IR_PAIRING_RIGHT) keyIndex = 3;
            else if (state == IR_PAIRING_ENTER) keyIndex = 4;
            else if (state == IR_PAIRING_ESC) keyIndex = 5;
            else if (state == IR_PAIRING_STANDBY) keyIndex = 6;
            else if (state == IR_PAIRING_POWEROFF) keyIndex = 7;
            else keyIndex = (settings_ir_focus_index >= 2 && settings_ir_focus_index <= 9) ? (settings_ir_focus_index - 2) : 0;
            
            const char* currentTarget = keyNames[keyIndex];
            
            bool isDup = IRService::isDuplicateError();
            int actP = IRService::getActiveProfile();
            int saveP = (actP == 0) ? 1 : actP;
            
            char prof_buf[64];
            snprintf(prof_buf, sizeof(prof_buf), "保存目标: [自定义组 %d]", saveP);
            
            if (isSeq) {
                char step_buf[64];
                snprintf(step_buf, sizeof(step_buf), "流水线对码 [%d/8]", keyIndex + 1);
                drawChineseTextCentered(step_buf, 12, y1, cw, CustomUiTheme::FOCUS_GOLD, CustomUiTheme::SURFACE, true);
                
                char target_buf[64];
                snprintf(target_buf, sizeof(target_buf), "请按下: [%s]", currentTarget);
                drawChineseTextCentered(target_buf, 12, y2, cw, CustomUiTheme::PRIMARY, CustomUiTheme::SURFACE, true);
                
                drawChineseTextCentered(prof_buf, 12, y3, cw, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
                
                if (isDup) {
                    drawChineseTextCentered("⚠️ 警告: 按键发生重复！", 12, y4, cw, CustomUiTheme::DANGER, CustomUiTheme::SURFACE, true);
                    drawChineseTextCentered("该红外码已被其他按键绑定", 12, y5, cw, CustomUiTheme::DANGER, CustomUiTheme::SURFACE, false);
                    drawChineseTextCentered("请按下未使用的按键重试", 12, y6, cw, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);
                } else {
                    drawChineseTextCentered("收到信号后自动跳到下一按键", 12, y4, cw, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);
                    drawChineseTextCentered("按顺序全套一次性对码完成", 12, y5, cw, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
                }
                drawChineseTextCentered("(按返回键取消对码)", 12, y7, cw, CustomUiTheme::DANGER, CustomUiTheme::SURFACE, false);
            } else {
                drawChineseTextCentered("正在接收红外信号...", 12, y1, cw, CustomUiTheme::FOCUS_GOLD, CustomUiTheme::SURFACE, true);
                
                char target_buf[64];
                snprintf(target_buf, sizeof(target_buf), "目标: [%s]", currentTarget);
                drawChineseTextCentered(target_buf, 12, y2, cw, CustomUiTheme::PRIMARY, CustomUiTheme::SURFACE, true);
                
                drawChineseTextCentered(prof_buf, 12, y3, cw, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
                
                if (isDup) {
                    drawChineseTextCentered("⚠️ 警告: 按键发生重复！", 12, y4, cw, CustomUiTheme::DANGER, CustomUiTheme::SURFACE, true);
                    drawChineseTextCentered("该红外码已被其他按键绑定", 12, y5, cw, CustomUiTheme::DANGER, CustomUiTheme::SURFACE, false);
                    drawChineseTextCentered("请按下未使用的按键重试", 12, y6, cw, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);
                } else {
                    drawChineseTextCentered("请对着接收头按下遥控器按键", 12, y4, cw, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);
                    drawChineseTextCentered("收到信号后自动保存退出", 12, y5, cw, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
                }
                drawChineseTextCentered("(按返回键取消对码)", 12, y7, cw, CustomUiTheme::DANGER, CustomUiTheme::SURFACE, false);
            }
        } else {
            renderHeader("红外对码菜单");
            bool lan = isLandscape();
            int cw = lan ? 296 : 216;
            int lx = lan ? 24 : 20;
            int bx = lan ? 32 : 20;
            int bw = lan ? 256 : 200;
            int bh = lan ? 40 : 38;
            drawCard(12, 40, cw, lan ? 188 : 258);
            
            const char* labels[] = {
                "当前遥控预设",
                "🚀 8键流水线自动对码",
                "1. 对码 - 上按键 (UP)",
                "2. 对码 - 下按键 (DOWN)",
                "3. 对码 - 左按键 (LEFT)",
                "4. 对码 - 右按键 (RIGHT)",
                "5. 对码 - 确认键 (ENTER)",
                "6. 对码 - 退出键 (ESC)",
                "7. 对码 - 息屏待机 (STANDBY)",
                "8. 对码 - 深度关机 (POWER OFF)"
            };
            
            int start_idx = (settings_ir_focus_index / 2) * 2;
            for (int i = start_idx; i < start_idx + 2 && i < 10; i++) {
                bool focused = (settings_ir_focus_index == i);
                int local_y = (lan ? 52 : 60) + (i - start_idx) * (lan ? 84 : 100);
                
                drawChineseText(labels[i], lx, local_y, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);
                
                if (i == 0) {
                    int p = IRService::getActiveProfile();
                    const char* pNames[] = {"预设: 通用 00FF", "预设: 自定义 1", "预设: 自定义 2"};
                    uint16_t pColors[] = {CustomUiTheme::SUCCESS, CustomUiTheme::PURPLE, CustomUiTheme::PRIMARY};
                    drawActionButton(bx, local_y + (lan ? 18 : 20), bw, bh, pNames[p], pColors[p], focused);
                    drawChineseText("说明: 按确认或左右键切换预设", lx, local_y + 70, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
                } else if (i == 1) {
                    drawActionButton(bx, local_y + (lan ? 18 : 20), bw, bh, "开启 8键流水线对码", CustomUiTheme::PURPLE, focused);
                    drawChineseText("说明: 依次全套自动顺序完成对码", lx, local_y + 70, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
                } else {
                    drawActionButton(bx, local_y + (lan ? 18 : 20), bw, bh, "点击单独接收信号", CustomUiTheme::PRIMARY_DARK, focused);
                    drawChineseText("说明: 针对单按键单独对码学习", lx, local_y + 70, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
                }
            }
            drawProgressBar(lan ? 150 : 110, lan ? 222 : 280, 20, 4, ((start_idx / 2) + 1) * 20, CustomUiTheme::FOCUS_GOLD);
        }
        return;
    }

    renderHeader("系统设置");
    bool lan = isLandscape();
    int cw = lan ? 296 : 216;
    int lx = lan ? 24 : 20;
    int bx = lan ? 32 : 20;
    int bw = lan ? 256 : 200;
    int bh = lan ? 40 : 38;
    drawCard(12, 40, cw, lan ? 188 : 258);
    
    int start_idx = (settings_focus_index / 2) * 2;
    
    for (int i = start_idx; i < start_idx + 2 && i < 8; i++) {
        bool focused = page_focused && (settings_focus_index == i);
        int local_y = (lan ? 52 : 60) + (i - start_idx) * (lan ? 84 : 100);
        
        if (i == 0) {
            drawChineseText("USB 电脑投屏", lx, local_y, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);
            drawActionButton(bx, local_y + (lan ? 18 : 20), bw, bh, screen_mirror_enabled ? "状态: 开启" : "状态: 关闭", 
                            screen_mirror_enabled ? CustomUiTheme::SUCCESS : CustomUiTheme::MUTED, focused);
            drawChineseText("说明: 原生画流, 极占带宽高负载", lx, local_y + 70, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
        } else if (i == 1) {
            drawChineseText("系统提示音", lx, local_y, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);
            drawActionButton(bx, local_y + (lan ? 18 : 20), bw, bh, system_sound_enabled ? "状态: 开启" : "状态: 静音", 
                            system_sound_enabled ? CustomUiTheme::SUCCESS : CustomUiTheme::MUTED, focused);
            drawChineseText("说明: 控制所有按键与通知音效", lx, local_y + 70, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
        } else if (i == 2) {
            drawChineseText("屏幕自动旋转", lx, local_y, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);
            drawActionButton(bx, local_y + (lan ? 18 : 20), bw, bh, auto_rotate_enabled ? "状态: 开启" : "状态: 锁定", 
                            auto_rotate_enabled ? CustomUiTheme::SUCCESS : CustomUiTheme::MUTED, focused);
            drawChineseText("说明: 依赖 ICM42688 姿态仪支持", lx, local_y + 70, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
        } else if (i == 3) {
            char title_buf[64];
            snprintf(title_buf, sizeof(title_buf), "屏幕亮度调节  (%d%%)", (screen_brightness * 100) / 255);
            drawChineseText(title_buf, lx, local_y, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);
            
            uint16_t border = focused ? (settings_adjust_mode ? CustomUiTheme::DANGER : CustomUiTheme::FOCUS_GOLD) : CustomUiTheme::SURFACE_ALT;
            spr.fillRoundRect(bx, local_y + (lan ? 18 : 20), bw, bh, 8, CustomUiTheme::SURFACE_ALT);
            if (focused) spr.drawRoundRect(bx - 2, local_y + (lan ? 16 : 18), bw + 4, bh + 4, 10, border);
            
            int fill_w = map(screen_brightness, 10, 255, 0, bw);
            spr.fillRoundRect(bx, local_y + (lan ? 18 : 20), fill_w, bh, 8, CustomUiTheme::PRIMARY);
            
            drawChineseText("说明: 按回车进入, 左右调节", lx, local_y + 70, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
        } else if (i == 4) {
            char title_buf[64];
            if (auto_sleep_time == 0) snprintf(title_buf, sizeof(title_buf), "无人自动息屏  (常亮)");
            else if (auto_sleep_time == 1) snprintf(title_buf, sizeof(title_buf), "无人自动息屏  (60秒)");
            else snprintf(title_buf, sizeof(title_buf), "无人自动息屏  (%d分钟)", auto_sleep_time);
            drawChineseText(title_buf, lx, local_y, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);
            
            uint16_t border = focused ? (settings_adjust_mode ? CustomUiTheme::DANGER : CustomUiTheme::FOCUS_GOLD) : CustomUiTheme::SURFACE_ALT;
            spr.fillRoundRect(bx, local_y + (lan ? 18 : 20), bw, bh, 8, CustomUiTheme::SURFACE_ALT);
            if (focused) spr.drawRoundRect(bx - 2, local_y + (lan ? 16 : 18), bw + 4, bh + 4, 10, border);
            
            const int gears[] = {0, 1, 5, 10, 15, 20, 25, 30};
            int current_idx = 0;
            for (int k = 0; k < 8; k++) if (gears[k] == auto_sleep_time) current_idx = k;
            
            int line_start_x = lan ? 42 : 30;
            int line_w = lan ? 236 : 180;
            int line_y = local_y + (lan ? 38 : 39);
            
            spr.fillRect(line_start_x, line_y - 2, line_w, 4, CustomUiTheme::SURFACE);
            int active_w = (line_w * current_idx) / 7;
            spr.fillRect(line_start_x, line_y - 2, active_w, 4, CustomUiTheme::PRIMARY);
            
            for (int k = 0; k < 8; k++) {
                int node_x = line_start_x + (line_w * k) / 7;
                if (k <= current_idx) {
                    spr.fillCircle(node_x, line_y, 6, CustomUiTheme::PRIMARY);
                    if (k == current_idx) spr.fillCircle(node_x, line_y, 3, TFT_WHITE);
                } else {
                    spr.fillCircle(node_x, line_y, 6, CustomUiTheme::SURFACE);
                }
            }
            
            drawChineseText("说明: 按回车进入, 左右调节", lx, local_y + 70, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
        } else if (i == 5) {
            char title_buf[64];
            snprintf(title_buf, sizeof(title_buf), "语音播放音量  (%d%%)", tts_volume);
            drawChineseText(title_buf, lx, local_y, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);
            
            uint16_t border = focused ? (settings_adjust_mode ? CustomUiTheme::DANGER : CustomUiTheme::FOCUS_GOLD) : CustomUiTheme::SURFACE_ALT;
            spr.fillRoundRect(bx, local_y + (lan ? 18 : 20), bw, bh, 8, CustomUiTheme::SURFACE_ALT);
            if (focused) spr.drawRoundRect(bx - 2, local_y + (lan ? 16 : 18), bw + 4, bh + 4, 10, border);
            
            int fill_w = map(tts_volume, 0, 100, 0, bw);
            spr.fillRoundRect(bx, local_y + (lan ? 18 : 20), fill_w, bh, 8, CustomUiTheme::PRIMARY);
            
            drawChineseText("说明: 仅控制 AI 语音播报声量", lx, local_y + 70, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
        } else if (i == 6) {
            drawChineseText("红外遥控对码", lx, local_y, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);
            drawActionButton(bx, local_y + (lan ? 18 : 20), bw, bh, "点击进入对码菜单", CustomUiTheme::PURPLE, focused);
            drawChineseText("说明: 自由适配任意红外遥控器", lx, local_y + 70, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
        } else if (i == 7) {
            drawChineseText("WiFi 网络设置", lx, local_y, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);
            String curSsid = NetworkRuntime::getActiveSsid();
            bool hasWifi = !curSsid.isEmpty();
            String btnText = "当前: " + (hasWifi ? curSsid : String("未连接"));
            uint16_t btnBg = hasWifi ? 0x0A84 : CustomUiTheme::SURFACE_ALT;
            drawActionButton(bx, local_y + (lan ? 18 : 20), bw, bh, btnText.c_str(), btnBg, focused);
            drawChineseText("说明: 搜索周边WiFi并连接入网", lx, local_y + 70, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
        }
    }
    
    drawProgressBar(lan ? 150 : 110, lan ? 222 : 280, 20, 4, ((start_idx / 2) + 1) * 25, CustomUiTheme::FOCUS_GOLD);
}

void CustomUiEngine::startWifiScan() {
    Serial.println("[WIFI_SCAN] 启动周边 WiFi 扫描流程...");
    NetworkRuntime::setScanBusy(true);
    wifi_scanning_in_progress = true;
    wifi_scan_state = WIFI_SCAN_STARTING;
    wifi_scan_start_time = millis();
    wifi_scan_retry_count = 0;
    wifi_scan_status_msg = "正在启动无线射频...";
    scanned_wifis.clear();
    wifi_list_focus_index = 0;
    wifi_list_scroll_top = 0;
    ui_needs_redraw = true;

    // 🌟 1. 禁用省电休眠，射频全功率常开
    esp_wifi_set_ps(WIFI_PS_NONE);

    // 🌟 2. 确保 STA 模式已启用
    WiFi.enableSTA(true);
    WiFi.mode(WIFI_STA);

    // 🌟 3. 清理可能残留的旧扫描
    WiFi.scanDelete();
}

void CustomUiEngine::stopWifiScan() {
    if (wifi_scanning_in_progress) {
        Serial.println("[WIFI_SCAN] 正在中止 WiFi 扫描...");
        WiFi.scanDelete();
        wifi_scanning_in_progress = false;
        wifi_scan_state = WIFI_SCAN_IDLE;
    }
    NetworkRuntime::setScanBusy(false);
    if (WiFi.status() == WL_CONNECTED) {
        esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    }
}

void CustomUiEngine::pollWifiScan() {
    if (!wifi_scanning_in_progress) return;

    const uint32_t now = millis();

    if (wifi_scan_state == WIFI_SCAN_STARTING) {
        // 让 UI 线程先至少绘制出 1 帧 "正在扫描周边 WiFi..." 界面，给射频启动留出充足时间
        if (now - wifi_scan_start_time < 150) return;

        Serial.println("[WIFI_SCAN] 调用 WiFi.scanNetworks(true, false, false, 300)...");
        // 🌟 核心：async=true 异步非阻塞扫描，show_hidden=false，passive=false，每个信道停留最多 300ms（底层总超时为 6000ms，足够全信道扫描）
        int16_t res = WiFi.scanNetworks(true, false, false, 300);
        Serial.printf("[WIFI_SCAN] scanNetworks 返回状态码: %d\r\n", res);

        if (res == WIFI_SCAN_RUNNING) {
            wifi_scan_state = WIFI_SCAN_SCANNING;
            wifi_scan_start_time = now;
            wifi_scan_status_msg = "正在侦听 2.4GHz 全信道...";
            ui_needs_redraw = true;
        } else if (res >= 0) {
            // 同步极速返回（若由驱动缓存命中）
            wifi_scan_state = WIFI_SCAN_DONE;
            int count = res;
            scanned_wifis.clear();
            for (int i = 0; i < count; i++) {
                String s = WiFi.SSID(i);
                s.trim();
                if (s.length() == 0) continue;
                bool dup = false;
                for (auto& it : scanned_wifis) {
                    if (it.ssid == s) {
                        dup = true;
                        if (WiFi.RSSI(i) > it.rssi) it.rssi = WiFi.RSSI(i);
                        break;
                    }
                }
                if (!dup) {
                    ScannedWifiItem it;
                    it.ssid = s;
                    it.rssi = WiFi.RSSI(i);
                    it.is_open = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN);
                    scanned_wifis.push_back(it);
                }
            }
            std::sort(scanned_wifis.begin(), scanned_wifis.end(), [](const ScannedWifiItem& a, const ScannedWifiItem& b) {
                return a.rssi > b.rssi;
            });
            WiFi.scanDelete();
            wifi_scanning_in_progress = false;
            NetworkRuntime::setScanBusy(false);
            if (WiFi.status() == WL_CONNECTED) esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
            ui_needs_redraw = true;
        } else {
            // 返回 WIFI_SCAN_FAILED (-2)
            Serial.printf("[WIFI_SCAN] 启动扫描失败(%d), 重试计数: %d\r\n", res, wifi_scan_retry_count);
            if (wifi_scan_retry_count < 2) {
                wifi_scan_retry_count++;
                WiFi.scanDelete();
                WiFi.disconnect(false, false);
                WiFi.mode(WIFI_STA);
                wifi_scan_state = WIFI_SCAN_STARTING;
                wifi_scan_start_time = now;
                wifi_scan_status_msg = "射频复位，准备重试...";
                ui_needs_redraw = true;
            } else {
                wifi_scan_state = WIFI_SCAN_FAILED_STATE;
                wifi_scan_status_msg = "射频扫描启动失败，请重试";
                wifi_scanning_in_progress = false;
                NetworkRuntime::setScanBusy(false);
                if (WiFi.status() == WL_CONNECTED) esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
                ui_needs_redraw = true;
            }
        }
    } else if (wifi_scan_state == WIFI_SCAN_SCANNING) {
        int16_t count = WiFi.scanComplete();

        if (count == WIFI_SCAN_RUNNING) {
            // 仍在扫描中，检查超时保护 (10秒)
            if (now - wifi_scan_start_time > 10000) {
                Serial.println("[WIFI_SCAN] 扫描已超时 (10秒)");
                WiFi.scanDelete();
                wifi_scan_state = WIFI_SCAN_FAILED_STATE;
                wifi_scan_status_msg = "扫描超时，请点击重试";
                wifi_scanning_in_progress = false;
                NetworkRuntime::setScanBusy(false);
                if (WiFi.status() == WL_CONNECTED) esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
                ui_needs_redraw = true;
            } else {
                static uint32_t last_scan_tick = 0;
                if (now - last_scan_tick > 100) {
                    last_scan_tick = now;
                    ui_needs_redraw = true;
                }
            }
        } else if (count >= 0) {
            Serial.printf("[WIFI_SCAN] 扫描成功完成！共发现 AP 数量: %d\r\n", count);
            scanned_wifis.clear();
            for (int i = 0; i < count; i++) {
                String s = WiFi.SSID(i);
                s.trim();
                if (s.length() == 0) continue;
                bool dup = false;
                for (auto& it : scanned_wifis) {
                    if (it.ssid == s) {
                        dup = true;
                        if (WiFi.RSSI(i) > it.rssi) it.rssi = WiFi.RSSI(i);
                        break;
                    }
                }
                if (!dup) {
                    ScannedWifiItem it;
                    it.ssid = s;
                    it.rssi = WiFi.RSSI(i);
                    it.is_open = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN);
                    scanned_wifis.push_back(it);
                }
            }

            std::sort(scanned_wifis.begin(), scanned_wifis.end(), [](const ScannedWifiItem& a, const ScannedWifiItem& b) {
                return a.rssi > b.rssi;
            });

            WiFi.scanDelete();
            wifi_scan_state = WIFI_SCAN_DONE;
            wifi_scanning_in_progress = false;
            NetworkRuntime::setScanBusy(false);
            if (WiFi.status() == WL_CONNECTED) esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
            wifi_list_focus_index = 0;
            wifi_list_scroll_top = 0;
            ui_needs_redraw = true;
        } else {
            // WIFI_SCAN_FAILED (-2)
            Serial.printf("[WIFI_SCAN] scanComplete 返回失败 (%d), 重试计数: %d\r\n", count, wifi_scan_retry_count);
            WiFi.scanDelete();
            if (wifi_scan_retry_count < 2) {
                wifi_scan_retry_count++;
                wifi_scan_state = WIFI_SCAN_STARTING;
                wifi_scan_start_time = now;
                wifi_scan_status_msg = "正在重新扫描...";
                ui_needs_redraw = true;
            } else {
                wifi_scan_state = WIFI_SCAN_FAILED_STATE;
                wifi_scan_status_msg = "未发现周边可用网络";
                wifi_scanning_in_progress = false;
                NetworkRuntime::setScanBusy(false);
                if (WiFi.status() == WL_CONNECTED) esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
                ui_needs_redraw = true;
            }
        }
    }
}

void CustomUiEngine::renderWifiScanList() {
    renderHeader("WiFi 网络");
    bool lan = isLandscape();
    int cw = lan ? 296 : 216;
    int ch = lan ? 188 : 258;
    drawCard(12, 40, cw, ch);

    if (wifi_scanning_in_progress) {
        int cx = 12;
        int title_y = lan ? 68 : 86;
        int sub_y = lan ? 98 : 120;
        int bar_y = lan ? 130 : 158;

        drawChineseTextCentered("正在扫描周边 WiFi 网络...", 12, title_y, cw, CustomUiTheme::FOCUS_GOLD, CustomUiTheme::SURFACE, true);

        // 🌟 动态加载小圆点动画
        uint32_t elapsed_ticks = (millis() - wifi_scan_start_time) / 300;
        String dots = "";
        for (uint32_t d = 0; d < (elapsed_ticks % 4); d++) dots += ".";
        String status_str = wifi_scan_status_msg + dots;
        drawChineseTextCentered(status_str.c_str(), 12, sub_y, cw, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);

        // 🌟 扫描流动进度条
        int anim_pct = ((millis() - wifi_scan_start_time) / 30) % 101;
        int bar_w = lan ? 180 : 160;
        int bar_x = cx + (cw - bar_w) / 2;
        drawProgressBar(bar_x, bar_y, bar_w, 6, anim_pct, CustomUiTheme::FOCUS_GOLD);

        char time_buf[32];
        snprintf(time_buf, sizeof(time_buf), "已耗时: %.1f 秒", (millis() - wifi_scan_start_time) / 1000.0f);
        drawChineseTextCentered(time_buf, 12, bar_y + 18, cw, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);

        drawChineseTextCentered("按 [ESC] 可取消扫描", 12, bar_y + 40, cw, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
        return;
    }

    int total_count = 1 + (int)scanned_wifis.size();
    int max_visible = lan ? 4 : 5;
    int item_h = lan ? 34 : 38;
    int item_gap = lan ? 6 : 8;
    int start_y = lan ? 48 : 52;
    int bx = lan ? 20 : 18;
    int bw = lan ? 280 : 204;

    for (int vi = 0; vi < max_visible; vi++) {
        int idx = wifi_list_scroll_top + vi;
        if (idx >= total_count) break;
        int y = start_y + vi * (item_h + item_gap);
        bool focused = (wifi_list_focus_index == idx);

        if (idx == 0) {
            drawActionButton(bx, y, bw, item_h, "重新扫描周围网络", CustomUiTheme::SURFACE_ALT, focused);
        } else {
            const auto& wifi = scanned_wifis[idx - 1];
            bool isConnected = (NetworkRuntime::getActiveSsid() == wifi.ssid);

            // 🌟 1. 背景与边框：当前连接的 WiFi 拥有专属墨绿底色，绝不与背景重叠混淆
            uint16_t bg, border, text_col;
            if (isConnected) {
                if (focused) {
                    bg = 0x0348; // 深林墨翠底色
                    border = CustomUiTheme::FOCUS_GOLD;
                    text_col = 0xFFFF; // 纯白高亮文字
                } else {
                    bg = 0x0A84; // 醒目的森林绿背景卡片
                    border = CustomUiTheme::SUCCESS; // 翡翠绿边框
                    text_col = 0xFFFF; // 纯白文字，100% 极高清晰度与辨识度
                }
            } else {
                if (focused) {
                    bg = CustomUiTheme::PRIMARY_DARK;
                    border = CustomUiTheme::FOCUS_GOLD;
                    text_col = CustomUiTheme::FOCUS_GOLD;
                } else {
                    bg = CustomUiTheme::SURFACE_ALT;
                    border = CustomUiTheme::BORDER;
                    text_col = CustomUiTheme::TEXT;
                }
            }

            spr.fillRoundRect(bx, y, bw, item_h, 6, bg);
            spr.drawRoundRect(bx, y, bw, item_h, 6, border);
            if (focused) {
                spr.drawRoundRect(bx - 1, y - 1, bw + 2, item_h + 2, 7, CustomUiTheme::FOCUS_GOLD);
            }

            // 🌟 2. 信号强度条 (最右侧)
            int sig_x = bx + bw - 24;
            int sig_base_y = y + item_h - 9;
            int bars = (wifi.rssi >= -60) ? 3 : (wifi.rssi >= -75 ? 2 : 1);
            for (int b = 0; b < 3; b++) {
                int bh = 4 + b * 4;
                uint16_t bar_c = (b < bars) ? (isConnected ? 0x7FE0 : CustomUiTheme::PRIMARY) : CustomUiTheme::BORDER;
                spr.fillRect(sig_x + b * 6, sig_base_y - bh, 4, bh, bar_c);
            }

            // 🌟 3. 状态标签 (信号柱左侧)
            int badge_r_x = sig_x - 6;
            int badge_w = 0;
            if (isConnected) {
                badge_w = 44;
                int badge_x = badge_r_x - badge_w;
                uint16_t badge_bg = focused ? 0x0545 : 0x0E66;
                spr.fillRoundRect(badge_x, y + (item_h - 18) / 2, badge_w, 18, 4, badge_bg);
                spr.drawRoundRect(badge_x, y + (item_h - 18) / 2, badge_w, 18, 4, CustomUiTheme::SUCCESS);
                drawChineseText("已连接", badge_x + 4, y + (item_h - 12) / 2, 0x7FE0, badge_bg, 4);
            } else if (wifi.is_open) {
                badge_w = 36;
                int badge_x = badge_r_x - badge_w;
                drawChineseText("[免密]", badge_x, y + (item_h - 12) / 2, CustomUiTheme::MUTED, bg, 4);
            } else {
                badge_w = 26;
                int badge_x = badge_r_x - badge_w;
                drawChineseText("[密]", badge_x, y + (item_h - 12) / 2, CustomUiTheme::MUTED, bg, 4);
            }

            // 🌟 4. WiFi 名字文本框与跑马灯滚动
            int name_x = bx + 8;
            int name_w = (badge_r_x - badge_w - 6) - name_x;
            if (name_w < 20) name_w = 20;

            if (focused) {
                // 聚焦时：如果长名字超宽，自动启用无缝 UTF-8 跑马灯平滑滚动！
                drawMarqueeText(wifi.ssid.c_str(), name_x, y + (item_h - 16) / 2, name_w, 16, text_col, u8g2_font_wqy12_t_gb2312a, false, bg);
            } else {
                // 未聚焦时：若不超长直接画，超长则以 ".." 优雅截断
                int text_w = getMultilingualTextWidth(wifi.ssid.c_str(), 4);
                if (text_w <= name_w) {
                    drawChineseText(wifi.ssid.c_str(), name_x, y + (item_h - 12) / 2, text_col, bg, 4);
                } else {
                    int dots_w = getMultilingualTextWidth("..", 4);
                    int target_w = name_w - dots_w - 2;
                    if (target_w < 10) target_w = 10;
                    size_t s_len = wifi.ssid.length();
                    size_t cidx = 0, best_end = 0;
                    while (cidx < s_len) {
                        size_t step = getUtf8CharStep(wifi.ssid.c_str(), cidx, s_len);
                        if (step == 0) break;
                        String test = wifi.ssid.substring(0, cidx + step);
                        if (getMultilingualTextWidth(test.c_str(), 4) <= target_w) {
                            best_end = cidx + step;
                            cidx += step;
                        } else {
                            break;
                        }
                    }
                    String sub = wifi.ssid.substring(0, best_end) + "..";
                    drawChineseText(sub.c_str(), name_x, y + (item_h - 12) / 2, text_col, bg, 4);
                }
            }
        }
    }

    if (scanned_wifis.empty()) {
        int tip_y = start_y + item_h + (lan ? 18 : 25);
        if (wifi_scan_state == WIFI_SCAN_FAILED_STATE || wifi_scan_status_msg.length() > 0) {
            drawChineseTextCentered(wifi_scan_status_msg.c_str(), 12, tip_y, cw, CustomUiTheme::FOCUS_GOLD, CustomUiTheme::SURFACE, false);
            drawChineseTextCentered("可按 [ENTER] 重新发起扫描", 12, tip_y + 24, cw, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
        } else {
            drawChineseTextCentered("未搜索到周边 WiFi，可点击上方重试", 12, tip_y, cw, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
        }
    }

    if (total_count > max_visible) {
        int bar_h = ch - 24;
        int thumb_h = std::max(10, bar_h * max_visible / total_count);
        int thumb_y = start_y + (bar_h - thumb_h) * wifi_list_scroll_top / (total_count - max_visible);
        spr.fillRoundRect(12 + cw - 6, thumb_y, 3, thumb_h, 1, CustomUiTheme::MUTED);
    }
}

void CustomUiEngine::renderWifiKeyboard() {
    renderHeader("输入 WiFi 密码");
    bool lan = isLandscape();
    int cw = lan ? 296 : 216;

    int top_h = lan ? 82 : 110;
    drawCard(12, 34, cw, top_h);

    int lx = lan ? 22 : 18;
    int box_y = lan ? 54 : 64;
    int box_w = cw - 20;
    int box_h = lan ? 26 : 30;

    // 标题与目标 SSID (长 SSID 自动跑马灯滑动)
    drawChineseText("网络:", lx, 38, CustomUiTheme::MUTED, CustomUiTheme::SURFACE);
    int ssid_box_x = lx + 36;
    int ssid_box_w = cw - 46;
    drawMarqueeText(selected_wifi_ssid.c_str(), ssid_box_x, 36, ssid_box_w, 18, CustomUiTheme::PRIMARY, u8g2_font_wqy12_t_gb2312a, false, CustomUiTheme::SURFACE);

    spr.fillRoundRect(lx, box_y, box_w, box_h, 4, CustomUiTheme::SURFACE_ALT);
    spr.drawRoundRect(lx, box_y, box_w, box_h, 4, CustomUiTheme::PRIMARY);

    if (wifi_password_input.length() == 0) {
        drawChineseText("请输入密码...", lx + 8, box_y + (box_h - 14) / 2, CustomUiTheme::MUTED, CustomUiTheme::SURFACE_ALT);
    } else {
        String vis = wifi_password_input;
        if (vis.length() > (lan ? 24 : 16)) {
            vis = "..." + vis.substring(vis.length() - (lan ? 21 : 13));
        }
        vis += "_";
        drawChineseText(vis.c_str(), lx + 8, box_y + (box_h - 14) / 2, CustomUiTheme::TEXT, CustomUiTheme::SURFACE_ALT);
    }

    int kw = lan ? 28 : 21;
    int kh = lan ? 23 : 30;
    int gap_x = lan ? 3 : 2;
    int gap_y = lan ? 3 : 4;
    int grid_w = 10 * kw + 9 * gap_x;
    int start_x = (screenWidth() - grid_w) / 2;
    int start_y = lan ? 122 : 154;

    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 10; c++) {
            int kx = start_x + c * (kw + gap_x);
            int ky = start_y + r * (kh + gap_y);
            bool focused = (r == kb_row && c == kb_col);

            char key_str[8] = "";
            uint16_t key_bg = CustomUiTheme::SURFACE_ALT;
            uint16_t text_color = CustomUiTheme::TEXT;

            if (r == 3 && c == 0) {
                if (kb_symbol_mode) {
                    strcpy(key_str, "ABC");
                } else {
                    strcpy(key_str, "#+=");
                }
                key_bg = CustomUiTheme::PURPLE;
            } else if (r == 3 && c == 8) {
                strcpy(key_str, "DEL");
                key_bg = CustomUiTheme::DANGER;
            } else if (r == 3 && c == 9) {
                strcpy(key_str, "OK");
                key_bg = CustomUiTheme::SUCCESS;
            } else if (r == 2 && c == 9 && !kb_symbol_mode) {
                strcpy(key_str, kb_upper_mode ? "a" : "A");
                key_bg = CustomUiTheme::PRIMARY_DARK;
            } else {
                char ch = getKbChar(r, c, kb_symbol_mode, kb_upper_mode);
                key_str[0] = ch;
                key_str[1] = '\0';
            }

            if (focused) {
                spr.fillRoundRect(kx, ky, kw, kh, 4, CustomUiTheme::FOCUS_GOLD);
                spr.drawRoundRect(kx, ky, kw, kh, 4, TFT_WHITE);
                u8f.setFontMode(1);
                u8f.setForegroundColor(TFT_BLACK);
                u8f.setFont(u8g2_font_wqy12_t_gb2312a);
                int tw = u8f.getUTF8Width(key_str);
                int tx = kx + (kw - tw) / 2;
                int ty = ky + (kh / 2) + 4;
                u8f.setCursor(tx, ty);
                u8f.print(key_str);
            } else {
                spr.fillRoundRect(kx, ky, kw, kh, 4, key_bg);
                spr.drawRoundRect(kx, ky, kw, kh, 4, CustomUiTheme::BORDER);
                u8f.setFontMode(1);
                u8f.setForegroundColor(text_color);
                u8f.setFont(u8g2_font_wqy12_t_gb2312a);
                int tw = u8f.getUTF8Width(key_str);
                int tx = kx + (kw - tw) / 2;
                int ty = ky + (kh / 2) + 4;
                u8f.setCursor(tx, ty);
                u8f.print(key_str);
            }
        }
    }
}

void CustomUiEngine::renderWifiConnecting() {
    renderHeader("连接 WiFi");
    bool lan = isLandscape();
    int cw = lan ? 296 : 216;
    int ch = lan ? 188 : 258;
    drawCard(12, 40, cw, ch);

    int y_center = lan ? 70 : 90;

    if (wifi_connect_pending) {
        drawChineseTextCentered("正在连接至 WiFi 网络...", 12, y_center, cw, CustomUiTheme::FOCUS_GOLD, CustomUiTheme::SURFACE, true);

        char ssid_buf[64];
        snprintf(ssid_buf, sizeof(ssid_buf), "SSID: %s", selected_wifi_ssid.c_str());
        drawChineseTextCentered(ssid_buf, 12, y_center + 30, cw, CustomUiTheme::PRIMARY, CustomUiTheme::SURFACE, true);

        drawChineseTextCentered("正在配置 IP 与网络时钟...", 12, y_center + 60, cw, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);

        spr.pushSprite(0, 0);

        NetworkRuntime::setScanBusy(true);
        bool ok = NetworkRuntime::reconnect(selected_wifi_ssid, wifi_password_input, 15000);
        NetworkRuntime::setScanBusy(false);
        wifi_connect_pending = false;
        wifi_connect_result = ok ? 1 : 2;
        wifi_connect_finish_time = millis();

        if (ok) {
            refreshNetworkInfo(true);
            beep(60, 2);
        } else {
            beep(100, 3);
        }
        ui_needs_redraw = true;
        return;
    }

    if (wifi_connect_result == 1) {
        drawChineseTextCentered("WiFi 连接成功！", 12, y_center, cw, CustomUiTheme::SUCCESS, CustomUiTheme::SURFACE, true);

        char ssid_buf[64];
        snprintf(ssid_buf, sizeof(ssid_buf), "已连接: %s", selected_wifi_ssid.c_str());
        drawChineseTextCentered(ssid_buf, 12, y_center + 30, cw, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);

        String ip_str = "IP: " + WiFi.localIP().toString();
        drawChineseTextCentered(ip_str.c_str(), 12, y_center + 55, cw, CustomUiTheme::PRIMARY, CustomUiTheme::SURFACE, false);

        drawChineseTextCentered("凭据已持久化至 Flash", 12, y_center + 80, cw, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
        drawChineseTextCentered("按 [ENTER] 或 [ESC] 返回设置", 12, y_center + 105, cw, CustomUiTheme::FOCUS_GOLD, CustomUiTheme::SURFACE, false);

        if (millis() - wifi_connect_finish_time > 3000) {
            settings_wifi_mode = WIFI_SUB_NONE;
            ui_needs_redraw = true;
        }
    } else if (wifi_connect_result == 2) {
        drawChineseTextCentered("连接失败！", 12, y_center, cw, CustomUiTheme::DANGER, CustomUiTheme::SURFACE, true);

        char ssid_buf[64];
        snprintf(ssid_buf, sizeof(ssid_buf), "目标: %s", selected_wifi_ssid.c_str());
        drawChineseTextCentered(ssid_buf, 12, y_center + 30, cw, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);

        drawChineseTextCentered("密码错误或信号微弱超时", 12, y_center + 55, cw, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);

        drawChineseTextCentered("按 [ENTER] 重新输入密码", 12, y_center + 85, cw, CustomUiTheme::FOCUS_GOLD, CustomUiTheme::SURFACE, false);
        drawChineseTextCentered("按 [ESC] 返回列表", 12, y_center + 110, cw, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
    }
}

/* ---------------- 1. 还原 100% 精美【家庭中控】主页 ---------------- */
void CustomUiEngine::renderHome() {
    // 局部快照, 遮蔽全局: 天气/城市由网络任务加锁写入, 这里加锁读取避免 String 竞争
    WeatherForecast forecasts[1];
    String current_city;
    if (state_mutex && xSemaphoreTake(state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        forecasts[0] = ::forecasts[0];
        current_city = ::current_city;
        xSemaphoreGive(state_mutex);
    } else {
        forecasts[0] = ::forecasts[0];
        current_city = ::current_city;
    }
    renderHeader("家庭中控");
    
    time_t now = time(nullptr);
    char timeStr[16] = "--:--:--";
    char dateNumStr[32] = "----/--/--";
    const char* weekdays[] = {"星期日", "星期一", "星期二", "星期三", "星期四", "星期五", "星期六"};
    const char* wdayStr = "--";
    bool clock_ok = (now >= 1700000000);
    if (clock_ok) {
        struct tm timeinfo;
        localtime_r(&now, &timeinfo);
        snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        snprintf(dateNumStr, sizeof(dateNumStr), "%04d-%02d-%02d", timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday);
        wdayStr = weekdays[timeinfo.tm_wday];
    }

    // 追加: 城市与星期信息 (16号中文字体，利用透明叠加渲染算法实现完美加粗)
    String cityText = (current_city.length() > 0) ? current_city : "定位中...";
    char cityWdayBuf[64];
    if (clock_ok) {
        snprintf(cityWdayBuf, sizeof(cityWdayBuf), "%s    %s", cityText.c_str(), wdayStr);
    } else {
        snprintf(cityWdayBuf, sizeof(cityWdayBuf), "%s    时间未同步", cityText.c_str());
    }

    if (isLandscape()) {
        // ================= 横屏布局 (320x240) =================
        u8f.setFont(u8g2_font_helvB24_tf);
        u8f.setForegroundColor(CustomUiTheme::PRIMARY);
        u8f.setBackgroundColor(CustomUiTheme::BG_TOP);
        u8f.setCursor(16, 56);
        u8f.print(timeStr);

        u8f.setFont(u8g2_font_helvB18_tf);
        u8f.setForegroundColor(CustomUiTheme::TEXT);
        u8f.setBackgroundColor(CustomUiTheme::BG_TOP);
        u8f.setCursor(16, 78);
        u8f.print(dateNumStr);

        drawChineseTextCentered(cityWdayBuf, 12, 86, 140, CustomUiTheme::TEXT, CustomUiTheme::BG_TOP, 1);

        // 今日天气卡片 (左)
        String wCondition = (weather_ready && forecasts[0].weather.length() > 0) ? forecasts[0].weather : "--";
        drawCard(12, 108, 144, 120);
        drawChineseTextCentered("今日天气", 12, 114, 144, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, 1);
        
        // 图标与文字居中，间距略微调整
        int iconX = 40;
        int textX = iconX + 44;
        drawWeatherIcon(getWeatherIconMap(wCondition), iconX, 132, 1.0f);
        drawChineseTextCentered(wCondition.c_str(), textX, 144, 144 - textX, CustomUiTheme::WARNING, CustomUiTheme::SURFACE, 3);
        
        char wTempBuf[32];
        if (weather_ready) {
            snprintf(wTempBuf, sizeof(wTempBuf), "%.0f°C", (float)forecasts[0].low_temp);
        } else {
            snprintf(wTempBuf, sizeof(wTempBuf), "--°C");
        }
        drawChineseTextCentered(wTempBuf, 12, 180, 72, CustomUiTheme::PRIMARY, CustomUiTheme::SURFACE, 3);
        drawChineseTextCentered("低温", 12, 204, 72, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, 3);
        
        if (weather_ready) {
            snprintf(wTempBuf, sizeof(wTempBuf), "%.0f°C", (float)forecasts[0].high_temp);
        } else {
            snprintf(wTempBuf, sizeof(wTempBuf), "--°C");
        }
        drawChineseTextCentered(wTempBuf, 84, 180, 72, CustomUiTheme::WARNING, CustomUiTheme::SURFACE, 3);
        drawChineseTextCentered("高温", 84, 204, 72, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, 3);

        // 室内环境卡片 (右)
        drawCard(164, 34, 144, 194);
        drawChineseTextCentered("室内环境", 164, 44, 144, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, 1);
        spr.drawFastHLine(172, 70, 128, CustomUiTheme::BORDER);
        
        spr.drawFastVLine(236, 76, 146, CustomUiTheme::BORDER);
        spr.drawFastHLine(172, 149, 128, CustomUiTheme::BORDER);

        extern volatile float ack_light_lux;
        char valBuf[32];

        // Top Left
        drawChineseTextCentered("室温", 164, 94, 72, CustomUiTheme::PRIMARY, CustomUiTheme::SURFACE, 3);
        if (ack_temperature_c > 0.0f) snprintf(valBuf, sizeof(valBuf), "%.1f°C", ack_temperature_c);
        else snprintf(valBuf, sizeof(valBuf), "--.-°C");
        drawChineseTextCentered(valBuf, 164, 124, 72, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, 3);

        // Top Right
        drawChineseTextCentered("气压", 236, 94, 72, CustomUiTheme::PRIMARY, CustomUiTheme::SURFACE, 3);
        if (ack_pressure_hpa > 0.0f) snprintf(valBuf, sizeof(valBuf), "%.0fhPa", ack_pressure_hpa);
        else snprintf(valBuf, sizeof(valBuf), "----hPa");
        drawChineseTextCentered(valBuf, 236, 124, 72, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, 3);

        // Bottom Left
        drawChineseTextCentered("湿度", 164, 172, 72, CustomUiTheme::PRIMARY, CustomUiTheme::SURFACE, 3);
        if (ack_dht_hum > 0.0f) snprintf(valBuf, sizeof(valBuf), "%.0f%%", ack_dht_hum);
        else snprintf(valBuf, sizeof(valBuf), "--%%");
        drawChineseTextCentered(valBuf, 164, 202, 72, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, 3);

        // Bottom Right
        drawChineseTextCentered("光照", 236, 172, 72, CustomUiTheme::PRIMARY, CustomUiTheme::SURFACE, 3);
        if (ack_light_lux > 0.0f) snprintf(valBuf, sizeof(valBuf), "%.0fLux", ack_light_lux);
        else snprintf(valBuf, sizeof(valBuf), "--Lux");
        drawChineseTextCentered(valBuf, 236, 202, 72, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, 3);
        return;
    }
    
    // 竖屏时间/日期绘制
    u8f.setFont(u8g2_font_helvB24_tf);
    u8f.setForegroundColor(CustomUiTheme::PRIMARY);
    u8f.setBackgroundColor(CustomUiTheme::BG_TOP);
    int tWidth = u8f.getUTF8Width(timeStr);
    u8f.setCursor((240 - tWidth) / 2, 64);
    u8f.print(timeStr);
    
    u8f.setFont(u8g2_font_helvB18_tf);
    u8f.setForegroundColor(CustomUiTheme::TEXT);
    u8f.setBackgroundColor(CustomUiTheme::BG_TOP);
    int dWidth = u8f.getUTF8Width(dateNumStr);
    u8f.setCursor((240 - dWidth) / 2, 86);
    u8f.print(dateNumStr);

    drawChineseTextCentered(cityWdayBuf, 0, 94, 240, CustomUiTheme::TEXT, CustomUiTheme::BG_TOP, 1);

    // 4. 【今日天气】卡片 (Y = 114 ~ 194, H = 80)
    drawCard(12, 114, 216, 80);
    
    // 标题居左，垂直居中于图标
    drawChineseText("今日天气", 24, 128, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, 1);
    
    String wCondition = (weather_ready && forecasts[0].weather.length() > 0) ? forecasts[0].weather : "--";
    
    // 🌟 图标居中对齐，卡片中线为 X=120，图标宽40，因此 X=100
    drawWeatherIcon(getWeatherIconMap(wCondition), 100, 116, 1.25f);
    
    // 天气文字位于图标右侧，垂直居中于图标
    drawChineseText(wCondition.c_str(), 148, 129, CustomUiTheme::WARNING, CustomUiTheme::SURFACE, 3);
    
    // 底部高低温，左右分块居中
    char wTempBuf[32];
    if (weather_ready) {
        snprintf(wTempBuf, sizeof(wTempBuf), "低温 %d°C", forecasts[0].low_temp);
    } else {
        snprintf(wTempBuf, sizeof(wTempBuf), "低温 --°C");
    }
    drawChineseTextCentered(wTempBuf, 12, 168, 108, CustomUiTheme::PRIMARY, CustomUiTheme::SURFACE, 3);
    
    if (weather_ready) {
        snprintf(wTempBuf, sizeof(wTempBuf), "高温 %d°C", forecasts[0].high_temp);
    } else {
        snprintf(wTempBuf, sizeof(wTempBuf), "高温 --°C");
    }
    drawChineseTextCentered(wTempBuf, 120, 168, 108, CustomUiTheme::WARNING, CustomUiTheme::SURFACE, 3);

    // 绘制精致的中间横线代替波浪号
    spr.drawFastHLine(106, 175, 28, CustomUiTheme::MUTED);

    // 5. 【室内环境】卡片 (Y = 206 ~ 296, H = 90)
    //    GY-63 温压 + DHT 湿度 + VEML7700 光照，2x2 整齐排列
    drawCard(12, 206, 216, 90);
    // 标题移到卡片左上角
    drawChineseText("室内环境", 24, 210, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, 1);

    extern volatile float ack_light_lux;
    char valBuf[32];

    // 2x2 布局：左上室温、右上气压、左下湿度、右下光照
    // 列1: 标签 X=24 / 数值 X=58；列2: 标签 X=126 / 数值 X=160
    // 行1: Y=234；行2: Y=262
    drawChineseText("室温", 24, 234, CustomUiTheme::PRIMARY, CustomUiTheme::SURFACE, 3);
    if (ack_temperature_c > 0.0f) snprintf(valBuf, sizeof(valBuf), "%.1f °C", ack_temperature_c);
    else snprintf(valBuf, sizeof(valBuf), "--.- °C");
    drawChineseText(valBuf, 58, 234, CustomUiTheme::PRIMARY, CustomUiTheme::SURFACE, 3);

    drawChineseText("气压", 126, 234, CustomUiTheme::PRIMARY, CustomUiTheme::SURFACE, 3);
    if (ack_pressure_hpa > 0.0f) snprintf(valBuf, sizeof(valBuf), "%.0f hPa", ack_pressure_hpa);
    else snprintf(valBuf, sizeof(valBuf), "---- hPa");
    drawChineseText(valBuf, 160, 234, CustomUiTheme::PRIMARY, CustomUiTheme::SURFACE, 3);

    drawChineseText("湿度", 24, 262, CustomUiTheme::PRIMARY, CustomUiTheme::SURFACE, 3);
    if (ack_dht_hum > 0.0f) snprintf(valBuf, sizeof(valBuf), "%.0f %%", ack_dht_hum);
    else snprintf(valBuf, sizeof(valBuf), "-- %%");
    drawChineseText(valBuf, 58, 262, CustomUiTheme::PRIMARY, CustomUiTheme::SURFACE, 3);

    drawChineseText("光照", 126, 262, CustomUiTheme::PRIMARY, CustomUiTheme::SURFACE, 3);
    if (ack_light_lux > 0.0f) snprintf(valBuf, sizeof(valBuf), "%.0f Lux", ack_light_lux);
    else snprintf(valBuf, sizeof(valBuf), "-- Lux");
    drawChineseText(valBuf, 160, 262, CustomUiTheme::PRIMARY, CustomUiTheme::SURFACE, 3);
}

/* ---------------- 2. 完整 4 天天气预报与趋势图 ---------------- */
void CustomUiEngine::renderWeather() {
    // 局部快照, 遮蔽全局: 加锁读取天气预报, 避免与网络任务写天气时竞争
    WeatherForecast forecasts[4];
    if (state_mutex && xSemaphoreTake(state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < 4; i++) forecasts[i] = ::forecasts[i];
        xSemaphoreGive(state_mutex);
    } else {
        for (int i = 0; i < 4; i++) forecasts[i] = ::forecasts[i];
    }
    renderHeader("天气预报");
    if (isLandscape()) {
        // ================= 横屏布局 (320x240) =================
        drawCard(12, 34, 296, 194);
        for (int i = 0; i < 4; i++) {
            int colX = 20 + i * 69;
            String dateStr = (weather_ready && forecasts[i].date.length() > 0) ? forecasts[i].date : "--/--";
            String weekStr = (weather_ready && forecasts[i].week.length() > 0) ? forecasts[i].week : (i == 0 ? "今天" : (i == 1 ? "明天" : (i == 2 ? "后天" : "大后天")));
            String wtrStr = (weather_ready && forecasts[i].weather.length() > 0) ? forecasts[i].weather : "--";

            drawChineseTextCentered(weekStr.c_str(), colX, 38, 68, CustomUiTheme::PRIMARY, CustomUiTheme::SURFACE, false);
            drawChineseTextCentered(dateStr.c_str(), colX, 52, 68, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
            drawWeatherIcon(getWeatherIconMap(wtrStr), colX + 18, 70, 1.0f);
            drawChineseTextCentered(wtrStr.c_str(), colX, 104, 68, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);

            char tempBuf[16];
            if (weather_ready) snprintf(tempBuf, sizeof(tempBuf), "%d°C", forecasts[i].high_temp);
            else snprintf(tempBuf, sizeof(tempBuf), "--°C");
            drawChineseTextCentered(tempBuf, colX, 200, 68, CustomUiTheme::WARNING, CustomUiTheme::SURFACE, false);

            if (weather_ready) snprintf(tempBuf, sizeof(tempBuf), "%d°C", forecasts[i].low_temp);
            else snprintf(tempBuf, sizeof(tempBuf), "--°C");
            drawChineseTextCentered(tempBuf, colX, 214, 68, CustomUiTheme::PRIMARY, CustomUiTheme::SURFACE, false);
        }

        if (weather_ready) {
            int px_high[4], py_high[4];
            int px_low[4], py_low[4];
            for (int i = 0; i < 4; i++) {
                px_high[i] = 54 + i * 69;
                py_high[i] = 138 - (forecasts[i].high_temp - 30) * 2;
                if (py_high[i] < 124) py_high[i] = 124;
                if (py_high[i] > 156) py_high[i] = 156;

                px_low[i] = 54 + i * 69;
                py_low[i] = 178 - (forecasts[i].low_temp - 20) * 2;
                if (py_low[i] < 160) py_low[i] = 160;
                if (py_low[i] > 192) py_low[i] = 192;
            }
            for (int i = 0; i < 3; i++) {
                spr.drawWideLine(px_high[i], py_high[i], px_high[i+1], py_high[i+1], 2, CustomUiTheme::WARNING, CustomUiTheme::SURFACE);
                spr.drawWideLine(px_low[i], py_low[i], px_low[i+1], py_low[i+1], 2, CustomUiTheme::PRIMARY, CustomUiTheme::SURFACE);
            }
            for (int i = 0; i < 4; i++) {
                spr.fillCircle(px_high[i], py_high[i], 4, CustomUiTheme::WARNING);
                spr.fillCircle(px_low[i], py_low[i], 4, CustomUiTheme::PRIMARY);
            }
        } else {
            drawChineseTextCentered("天气数据获取中...", 12, 150, 296, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
        }
        return;
    }
    drawCard(12, 34, 216, 264); // 底部对齐 Y = 298
    
    for (int i = 0; i < 4; i++) {
        int colX = 18 + i * 52;
        String dateStr = (weather_ready && forecasts[i].date.length() > 0) ? forecasts[i].date : "--/--";
        String weekStr = (weather_ready && forecasts[i].week.length() > 0) ? forecasts[i].week : (i == 0 ? "今天" : (i == 1 ? "明天" : (i == 2 ? "后天" : "大后天")));
        String wtrStr = (weather_ready && forecasts[i].weather.length() > 0) ? forecasts[i].weather : "--";

        drawChineseTextCentered(weekStr.c_str(), colX, 38, 48, CustomUiTheme::PRIMARY, CustomUiTheme::SURFACE, false);
        drawChineseTextCentered(dateStr.c_str(), colX, 52, 48, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
        
        // 🌟 绘制 4 天列中对应的 1.25x 统一放大彩图天气图标 (与主页 100% 尺寸与细节一致)
        drawWeatherIcon(getWeatherIconMap(wtrStr), colX + 4, 66, 1.25f);

        drawChineseTextCentered(wtrStr.c_str(), colX, 108, 48, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);
        
        char tempBuf[16];
        if (weather_ready) snprintf(tempBuf, sizeof(tempBuf), "%d°C", forecasts[i].high_temp);
        else snprintf(tempBuf, sizeof(tempBuf), "--°C");
        drawChineseTextCentered(tempBuf, colX, 230, 48, CustomUiTheme::WARNING, CustomUiTheme::SURFACE, false);
        
        if (weather_ready) snprintf(tempBuf, sizeof(tempBuf), "%d°C", forecasts[i].low_temp);
        else snprintf(tempBuf, sizeof(tempBuf), "--°C");
        drawChineseTextCentered(tempBuf, colX, 252, 48, CustomUiTheme::PRIMARY, CustomUiTheme::SURFACE, false);
    }
    
    if (weather_ready) {
        // 🌟 按原版自然比例 (1°C = 3px 纵向平缓起伏)，恢复优雅自然的真实折线波动
        int px_high[4], py_high[4];
        int px_low[4], py_low[4];
        for (int i = 0; i < 4; i++) {
            px_high[i] = 42 + i * 52;
            // 高温基缓线 30°C 对应 Y = 150，1°C 对应 3px
            py_high[i] = 150 - (forecasts[i].high_temp - 30) * 3;
            if (py_high[i] < 125) py_high[i] = 125;
            if (py_high[i] > 175) py_high[i] = 175;

            px_low[i] = 42 + i * 52;
            // 低温基缓线 20°C 对应 Y = 200，1°C 对应 3px
            py_low[i] = 200 - (forecasts[i].low_temp - 20) * 3;
            if (py_low[i] < 180) py_low[i] = 180;
            if (py_low[i] > 222) py_low[i] = 222;
        }
        
        // 绘制 2px 粗线条与数据高亮圆点
        for (int i = 0; i < 3; i++) {
            spr.drawWideLine(px_high[i], py_high[i], px_high[i+1], py_high[i+1], 2, CustomUiTheme::WARNING, CustomUiTheme::SURFACE);
            spr.drawWideLine(px_low[i], py_low[i], px_low[i+1], py_low[i+1], 2, CustomUiTheme::PRIMARY, CustomUiTheme::SURFACE);
        }
        for (int i = 0; i < 4; i++) {
            spr.fillCircle(px_high[i], py_high[i], 4, CustomUiTheme::WARNING);
            spr.fillCircle(px_low[i], py_low[i], 4, CustomUiTheme::PRIMARY);
        }
    } else {
        drawChineseTextCentered("天气预报数据获取中...", 12, 170, 216, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
    }
}

/* ---------------- 3. AI 语音助手页 ---------------- */
void CustomUiEngine::renderAi() {
    renderHeader("小乐语音助手");

    bool lan = isLandscape();
    int cardW = lan ? 296 : 216;
    int cardH = lan ? 146 : 220;
    int bannerW = lan ? 296 : 216;
    int maxLineWidth = lan ? 266 : 186;
    
    // 🌟 1. 顶部动态语音状态卡片 (支持录音水波纹、思考脉冲与待命引导)
    int bannerX = 12;
    int bannerY = lan ? 30 : 32;
    int bannerH = lan ? 38 : 40;

    if (isRecording) {
        // 🎙️ 聆听状态：深蓝底色 + 动态声波跳动动画
        spr.fillRoundRect(bannerX, bannerY, bannerW, bannerH, 8, 0x0113); // 深夜蓝
        spr.drawRoundRect(bannerX, bannerY, bannerW, bannerH, 8, CustomUiTheme::PRIMARY); // 荧光青边框
        
        uint32_t t = millis() / 70;
        int waveStartX = bannerX + 10;
        int waveCenterY = bannerY + bannerH / 2 - 2;
        for (int b = 0; b < 7; b++) {
            int barH = 5 + (int)(sinf((float)(t + b * 2) * 0.85f) * 8.0f) + (int)(cosf((float)(t * 2 + b) * 0.5f) * 3.0f);
            if (barH < 3) barH = 3;
            if (barH > 20) barH = 20;
            spr.fillRoundRect(waveStartX + b * 5, waveCenterY - barH / 2, 3, barH, 1, CustomUiTheme::PRIMARY);
        }
        
        drawChineseText("正在聆听您的指令...", bannerX + 52, bannerY + 4, CustomUiTheme::TEXT, 0x0113, 0);
        drawChineseText("说完停顿自动上传大模型", bannerX + 52, bannerY + 21, CustomUiTheme::MUTED, 0x0113, 0);
        spr.fillRoundRect(bannerX + 4, bannerY + bannerH - 4, bannerW - 8, 2, 1, CustomUiTheme::PRIMARY);
    } else if (isCommunicating) {
        // 🧠 思考与执行状态：深紫琥珀底色 + 脉冲呼吸动画
        spr.fillRoundRect(bannerX, bannerY, bannerW, bannerH, 8, 0x2084);
        spr.drawRoundRect(bannerX, bannerY, bannerW, bannerH, 8, CustomUiTheme::FOCUS_GOLD);
        
        uint32_t dotStep = (millis() / 200) % 4;
        int dotX = bannerX + 16;
        int dotY = bannerY + bannerH / 2;
        for (int d = 0; d < 3; d++) {
            uint16_t c = (d <= (int)dotStep) ? CustomUiTheme::FOCUS_GOLD : CustomUiTheme::MUTED;
            spr.fillCircle(dotX + d * 8, dotY, 3, c);
        }
        
        drawChineseText("正在深度思考与执行...", bannerX + 50, bannerY + 11, CustomUiTheme::FOCUS_GOLD, 0x2084, 0);
        spr.fillRoundRect(bannerX + 4, bannerY + bannerH - 4, bannerW - 8, 2, 1, CustomUiTheme::FOCUS_GOLD);
    } else if (isSpeaking) {
        // 🔊 语音播报状态：深翡翠底色 + 播音动态波纹
        spr.fillRoundRect(bannerX, bannerY, bannerW, bannerH, 8, 0x0208);
        spr.drawRoundRect(bannerX, bannerY, bannerW, bannerH, 8, CustomUiTheme::SUCCESS);
        
        uint32_t t = millis() / 80;
        int spkX = bannerX + 14;
        int spkY = bannerY + bannerH / 2;
        for (int b = 0; b < 5; b++) {
            int bh = 5 + (int)(cosf((float)(t + b * 2) * 0.9f) * 7.0f);
            spr.fillRoundRect(spkX + b * 6, spkY - bh / 2, 3, bh, 1, CustomUiTheme::SUCCESS);
        }
        
        drawChineseText("正在语音播报回复...", bannerX + 52, bannerY + 11, CustomUiTheme::SUCCESS, 0x0208, 0);
        spr.fillRoundRect(bannerX + 4, bannerY + bannerH - 4, bannerW - 8, 2, 1, CustomUiTheme::SUCCESS);
    } else {
        // 💡 待命就绪状态：引导呼唤与摇杆交互
        drawCard(bannerX, bannerY, bannerW, bannerH);
        drawChineseTextCentered("呼唤「你好小乐」随时唤醒", bannerX, bannerY + 5, bannerW, CustomUiTheme::PRIMARY, CustomUiTheme::SURFACE, 0);
        drawChineseTextCentered("或点击摇杆按键直接对话", bannerX, bannerY + 22, bannerW, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, 0);
    }

    // 🌟 2. 下方对话卡片区域 (严密限制在卡片内部)
    int chatCardY = bannerY + bannerH + 6;
    drawCard(12, chatCardY, cardW, cardH);
    int textBottom = chatCardY + cardH - 8;
    
    const char* src = currentText.c_str();
    if (strlen(src) == 0) {
        drawChineseText("欢迎使用智能语音助手！", 20, chatCardY + 10, CustomUiTheme::PRIMARY, CustomUiTheme::SURFACE, 0);
        drawChineseText("您可以随时对我说：", 20, chatCardY + 32, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, 0);
        drawChineseText("• “帮我把客厅的灯打开”", 20, chatCardY + 54, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, 0);
        String sampleWeather = "• “今天" + ((current_city.length() > 0 && current_city != "定位中..." && current_city != "未知城市") ? current_city : "本地") + "天气怎么样”";
        if (getMultilingualTextWidth(sampleWeather.c_str(), 0) > maxLineWidth) {
            sampleWeather = "• “今天本地天气怎么样”";
        }
        drawChineseText(sampleWeather.c_str(), 20, chatCardY + 74, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, 0);
        drawChineseText("• “播放音乐”", 20, chatCardY + 94, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, 0);
        drawChineseText("• “查询环境光照与温湿度”", 20, chatCardY + 114, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, 0);
        return;
    }

    std::vector<String> visualLines;
    String currentLine = "";
    
    // 按 \n 切割并应用 UTF-8 多语言精确像素换行 (严格依据 maxLineWidth 约束)
    for (size_t i = 0; i <= strlen(src); i++) {
        if (src[i] == '\n' || src[i] == '\0') {
            if (currentLine.length() > 0) {
                String remaining = currentLine;
                while (remaining.length() > 0) {
                    size_t charIndex = 0;
                    size_t lastValidLen = 0;

                    while (charIndex < remaining.length()) {
                        size_t step = getUtf8CharStep(remaining.c_str(), charIndex, remaining.length());
                        if (step == 0) step = 1;
                        size_t nextIndex = charIndex + step;

                        String testSub = remaining.substring(0, nextIndex);
                        if (getMultilingualTextWidth(testSub.c_str(), 0) > maxLineWidth && lastValidLen > 0) {
                            break;
                        }
                        lastValidLen = nextIndex;
                        charIndex = nextIndex;
                    }
                    if (lastValidLen == 0) lastValidLen = (charIndex > 0) ? charIndex : 1;

                    String subLine = remaining.substring(0, lastValidLen);
                    remaining = remaining.substring(lastValidLen);
                    visualLines.push_back(subLine);
                }
            }
            currentLine = "";
        } else {
            currentLine += src[i];
        }
    }

    int y = chatCardY + 8;
    int lineH = 18;
    int maxLinesOnScreen = (textBottom - y) / lineH;
    if (maxLinesOnScreen < 1) maxLinesOnScreen = 1;
    int totalLines = (int)visualLines.size();

    // 自动滚屏对齐最新消息
    int startIndex = 0;
    if (totalLines > maxLinesOnScreen) {
        startIndex = totalLines - maxLinesOnScreen - (int)ai_scroll_lines;
        if (startIndex < 0) startIndex = 0;
    }

    for (int i = startIndex; i < totalLines; i++) {
        if (y + 13 <= textBottom) {
            bool isMe = visualLines[i].startsWith("我:") || visualLines[i].startsWith("我：");
            uint16_t textColor = isMe ? CustomUiTheme::PRIMARY : CustomUiTheme::TEXT;
            drawChineseText(visualLines[i].c_str(), 20, y, textColor, CustomUiTheme::SURFACE, 0);
            y += lineH;
        } else {
            break;
        }
    }
}

/* ---------------- 4. 智能家居控制页 ---------------- */
void CustomUiEngine::renderSmartHome() {
    // 灯光亮度条: 按钮按下进入后显示; 未检测到设备时只显示状态不可调
    auto drawLightBar = [&](int x, int y, int w, int h, bool focused) {
        extern volatile int smart_light_ack_state;
        const bool detected = (smart_light_ack_state != -1);
        spr.fillRoundRect(x, y, w, h, 8, CustomUiTheme::SURFACE_ALT);
        if (focused) spr.drawRoundRect(x - 1, y - 1, w + 2, h + 2, 10, CustomUiTheme::FOCUS_GOLD);
        else spr.drawRoundRect(x, y, w, h, 8, CustomUiTheme::BORDER);

        char barBuf[40];
        const char* barLabel = "灯光控制 [未检测到]";
        if (detected && light_brightness > 20) {
            snprintf(barBuf, sizeof(barBuf), "灯光控制 [亮度 %d%%]", light_brightness);
            barLabel = barBuf;
            int fill_w = (w - 8) * (light_brightness - 20) / 80; // 20%=空, 100%=满
            if (fill_w > 4) spr.fillRoundRect(x + 4, y + 4, fill_w, h - 8, 4, CustomUiTheme::PRIMARY);
        } else if (detected) {
            barLabel = "灯光控制 [关]";
        }
        // 透明文字直接叠在亮度条上, 避免背景方块
        u8f.setFont(u8g2_font_wqy13_t_gb2312a);
        u8f.setFontMode(1);
        u8f.setForegroundColor(CustomUiTheme::TEXT);
        int tw = u8f.getUTF8Width(barLabel);
        u8f.setCursor(x + (w - tw) / 2, y + (h - 18) / 2 + 12);
        u8f.print(barLabel);
    };
    renderHeader("智能家居控制");
    
    // Dynamic Labels based on global states and show_result flags
    String lightText = "灯光控制";
    if (smart_home_show_result[0]) {
        extern volatile int smart_light_ack_state;
        if (smart_light_ack_state == 1) lightText = "灯光控制 [开]";
        else if (smart_light_ack_state == 0) lightText = "灯光控制 [关]";
        else lightText = "灯光控制 [未检测到]";
    }
    
    String envText = "气象数据";
    if (smart_home_show_result[1]) {
        if (ack_temperature_c != 0.0 || ack_pressure_hpa != 0.0) {
            envText = "温度:" + String(ack_temperature_c, 1) + "C 气压:" + String((int)ack_pressure_hpa) + "hPa";
        } else {
            envText = "气象数据 [未读取]";
        }
    }
    
    String dhtText = "外置温湿度";
    if (smart_home_show_result[2]) {
        if (ack_dht_temp > 0 || ack_dht_hum > 0) {
            dhtText = "温度:" + String(ack_dht_temp, 1) + "C 湿度:" + String((int)ack_dht_hum) + "%";
        } else {
            dhtText = "外置温湿 [未读取]";
        }
    }

    String vemlText = "环境光照";
    extern volatile uint8_t light_read_state;
    extern volatile float ack_light_lux;
    if (smart_home_show_result[3]) {
        if (light_read_state == 1) {
            vemlText = "环境光照 [请求中...]";
        } else if (light_read_state == 2) {
            vemlText = "光照强度: " + String(ack_light_lux, 1) + " Lux";
        } else {
            vemlText = "环境光照 [未读取]";
        }
    }
    
    String irText = "红外解码";
    if (smart_home_show_result[4]) {
        uint64_t lastCode = IRService::getLastRecvCode();
        if (lastCode != 0) {
            const char* actName = "未匹配";
            if (lastCode == IRService::getCodeUp()) actName = "上";
            else if (lastCode == IRService::getCodeDown()) actName = "下";
            else if (lastCode == IRService::getCodeLeft()) actName = "左";
            else if (lastCode == IRService::getCodeRight()) actName = "右";
            else if (lastCode == IRService::getCodeEnter()) actName = "确认";
            else if (lastCode == IRService::getCodeEsc()) actName = "退出";
            else if (lastCode == IRService::getCodeStandby()) actName = "待机";
            else if (lastCode == IRService::getCodePowerOff()) actName = "关机";
            
            char codePkt[64];
            snprintf(codePkt, sizeof(codePkt), "码:0x%08X [%s]", (uint32_t)lastCode, actName);
            irText = codePkt;
        }
    }

    if (isLandscape()) {
        // ================= 横屏布局 (320x240) =================
        // 前 4 项 2 列 x 2 行，底栏红外占满整行大卡片，字数更宽裕、视觉更均衡
        extern volatile int smart_light_ack_state;
        String lLight = "灯光";
        if (smart_light_ack_state == 1) lLight = "灯光 [开]";
        else if (smart_light_ack_state == 0) lLight = "灯光 [关]";
        else lLight = "灯光 [未检测]";

        String lEnv = "气象数据";
        if (smart_home_show_result[1] && (ack_temperature_c != 0.0 || ack_pressure_hpa != 0.0)) {
            lEnv = String(ack_temperature_c, 1) + "°C " + String((int)ack_pressure_hpa) + "hPa";
        }

        String lDht = "外置温湿";
        if (smart_home_show_result[2] && (ack_dht_temp > 0 || ack_dht_hum > 0)) {
            lDht = String(ack_dht_temp, 1) + "°C " + String((int)ack_dht_hum) + "%";
        }

        String lVeml = "环境光照";
        if (smart_home_show_result[3]) {
            if (light_read_state == 1) lVeml = "光照 [请求中]";
            else if (light_read_state == 2) lVeml = "光照 " + String(ack_light_lux, 1) + " Lux";
            else lVeml = "光照 [未读取]";
        }

        String lIr = "红外";
        if (smart_home_show_result[4]) {
            uint64_t lastCode = IRService::getLastRecvCode();
            if (lastCode != 0) {
                const char* actName = "未匹配";
                if (lastCode == IRService::getCodeUp()) actName = "上";
                else if (lastCode == IRService::getCodeDown()) actName = "下";
                else if (lastCode == IRService::getCodeLeft()) actName = "左";
                else if (lastCode == IRService::getCodeRight()) actName = "右";
                else if (lastCode == IRService::getCodeEnter()) actName = "确认";
                else if (lastCode == IRService::getCodeEsc()) actName = "退出";
                else if (lastCode == IRService::getCodeStandby()) actName = "待机";
                else if (lastCode == IRService::getCodePowerOff()) actName = "关机";
                char codePkt[32];
                snprintf(codePkt, sizeof(codePkt), "红外 0x%08X [%s]", (uint32_t)lastCode, actName);
                lIr = codePkt;
            }
        }

        String* labels[5] = {&lLight, &lEnv, &lDht, &lVeml, &lIr};
        uint16_t colors[5] = {
            CustomUiTheme::PRIMARY_DARK,
            CustomUiTheme::SUCCESS, CustomUiTheme::WARNING,
            CustomUiTheme::DANGER, CustomUiTheme::FOCUS_GOLD
        };

        // 绘制前 4 个两两成对的按钮
        for (int i = 0; i < 4; i++) {
            int c = i % 2;
            int r = i / 2;
            int x = (c == 0) ? 12 : 164;
            int y = (r == 0) ? 34 : 98;
            int w = 144;
            int h = 58;
            if (i == 0 && light_brightness_mode) {
                drawLightBar(x, y, w, h, page_focused && (focus_index == i));
            } else {
                drawActionButton(x, y, w, h, labels[i]->c_str(), colors[i],
                                 page_focused && (focus_index == i), 2);
            }
        }
        // 绘制第 5 个独占底行的红外解码按钮 (宽 296)
        drawActionButton(12, 162, 296, 58, labels[4]->c_str(), colors[4],
                         page_focused && (focus_index == 4), 2);
        return;
    }
    
    // 5 个控制按钮平分纵向空间 (Y = 36 ~ 284)
    if (light_brightness_mode) {
        drawLightBar(20, 36, 200, 42, page_focused && (focus_index == 0));
    } else {
        drawActionButton(20, 36,  200, 42, lightText.c_str(),   CustomUiTheme::PRIMARY_DARK, page_focused && (focus_index == 0));
    }
    drawActionButton(20, 86,  200, 42, envText.c_str(),     CustomUiTheme::SUCCESS,      page_focused && (focus_index == 1));
    drawActionButton(20, 136, 200, 42, dhtText.c_str(),     CustomUiTheme::WARNING,      page_focused && (focus_index == 2));
    drawActionButton(20, 186, 200, 42, vemlText.c_str(),    CustomUiTheme::DANGER,       page_focused && (focus_index == 3));
    drawActionButton(20, 236, 200, 48, irText.c_str(),      CustomUiTheme::FOCUS_GOLD,   page_focused && (focus_index == 4));
}

/* ---------------- 5. 系统状态页 ---------------- */
void CustomUiEngine::renderSystem() {
    renderHeader("系统状态");

    if (isLandscape()) {
        // ================= 横屏布局 (320x240) =================
        // 左侧三张资源卡, 右侧系统信息卡
        uint32_t s_free = ESP.getFreeHeap() / 1024;
        uint32_t s_size = ESP.getHeapSize() / 1024;
        uint32_t s_used = s_size > s_free ? s_size - s_free : 0;
        uint32_t s_pct = s_size > 0 ? s_used * 100 / s_size : 0;

        uint32_t p_free = ESP.getFreePsram() / 1024;
        uint32_t p_size = ESP.getPsramSize() / 1024;
        uint32_t p_used = p_size > p_free ? p_size - p_free : 0;
        uint32_t p_pct = p_size > 0 ? p_used * 100 / p_size : 0;

        size_t fs_total = LittleFS.totalBytes() / 1024;
        size_t fs_used = LittleFS.usedBytes() / 1024;
        uint32_t f_pct = fs_total > 0 ? fs_used * 100 / fs_total : 0;

        char buf[64];
        drawCard(8, 34, 176, 60);
        snprintf(buf, sizeof(buf), "SRAM:%u/%uKB(%u%%)", (unsigned)s_used, (unsigned)s_size, (unsigned)s_pct);
        drawChineseText(buf, 16, 40, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);
        drawProgressBar(16, 66, 160, 7, s_pct, CustomUiTheme::PRIMARY);

        drawCard(8, 101, 176, 60);
        snprintf(buf, sizeof(buf), "PSRAM:%u/%uKB(%u%%)", (unsigned)p_used, (unsigned)p_size, (unsigned)p_pct);
        drawChineseText(buf, 16, 107, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);
        drawProgressBar(16, 133, 160, 7, p_pct, CustomUiTheme::PURPLE);

        drawCard(8, 168, 176, 60);
        snprintf(buf, sizeof(buf), "Flash:%u/%uKB(%u%%)", (unsigned)fs_used, (unsigned)fs_total, (unsigned)f_pct);
        drawChineseText(buf, 16, 174, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);
        drawProgressBar(16, 200, 160, 7, f_pct, CustomUiTheme::WARNING);

        drawCard(188, 34, 124, 194);
        drawChineseText("芯片:", 196, 40, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);
        drawChineseText("ESP32-S3", 196, 54, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);
        drawChineseText("(双核240MHz)", 196, 68, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
        String wifiSsid = (WiFi.status() == WL_CONNECTED) ? ("WiFi: " + WiFi.SSID()) : "WiFi: 未连接";
        drawChineseText(wifiSsid.c_str(), 196, 92, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);
        String ipStr = (WiFi.status() == WL_CONNECTED) ? ("网络: " + WiFi.localIP().toString()) : "网络: 未连接";
        
        if (WiFi.status() == WL_CONNECTED) {
            drawChineseText("网络:", 196, 112, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);
            drawChineseText(WiFi.localIP().toString().c_str(), 196, 126, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);
        } else {
            drawChineseText("网络: 未连接", 196, 112, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);
        }
        
        int rssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;
        snprintf(buf, sizeof(buf), "信号: %d dBm", rssi);
        drawChineseText(buf, 196, 150, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);
        uint32_t sec = millis() / 1000;
        uint32_t hrs = sec / 3600;
        uint32_t mins = (sec % 3600) / 60;
        uint32_t secs = sec % 60;
        snprintf(buf, sizeof(buf), "运行:", (unsigned)hrs, (unsigned)mins, (unsigned)secs);
        drawChineseText(buf, 196, 176, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
        snprintf(buf, sizeof(buf), "%02u:%02u:%02u", (unsigned)hrs, (unsigned)mins, (unsigned)secs);
        drawChineseText(buf, 196, 190, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
        return;
    }
    
    uint32_t s_free = ESP.getFreeHeap() / 1024;
    uint32_t s_size = ESP.getHeapSize() / 1024;
    uint32_t s_used = s_size > s_free ? s_size - s_free : 0;
    uint32_t s_pct = s_size > 0 ? s_used * 100 / s_size : 0;
    drawCard(12, 32, 216, 44);
    char buf[64];
    snprintf(buf, sizeof(buf), "SRAM:%u/%uKB(%u%%)", (unsigned)s_used, (unsigned)s_size, (unsigned)s_pct);
    drawChineseText(buf, 20, 36, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);
    drawProgressBar(20, 58, 200, 7, s_pct, CustomUiTheme::PRIMARY);

    uint32_t p_free = ESP.getFreePsram() / 1024;
    uint32_t p_size = ESP.getPsramSize() / 1024;
    uint32_t p_used = p_size > p_free ? p_size - p_free : 0;
    uint32_t p_pct = p_size > 0 ? p_used * 100 / p_size : 0;
    drawCard(12, 82, 216, 44);
    snprintf(buf, sizeof(buf), "PSRAM:%u/%uKB(%u%%)", (unsigned)p_used, (unsigned)p_size, (unsigned)p_pct);
    drawChineseText(buf, 20, 86, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);
    drawProgressBar(20, 108, 200, 7, p_pct, CustomUiTheme::PURPLE);

    size_t fs_total = LittleFS.totalBytes() / 1024;
    size_t fs_used = LittleFS.usedBytes() / 1024;
    uint32_t f_pct = fs_total > 0 ? fs_used * 100 / fs_total : 0;
    drawCard(12, 132, 216, 44);
    snprintf(buf, sizeof(buf), "Flash:%u/%uKB(%u%%)", (unsigned)fs_used, (unsigned)fs_total, (unsigned)f_pct);
    drawChineseText(buf, 20, 136, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);
    drawProgressBar(20, 158, 200, 7, f_pct, CustomUiTheme::WARNING);

    drawCard(12, 182, 216, 116); // 底部对齐 Y = 298
    drawChineseText("芯片: ESP32-S3 (双核240MHz)", 20, 186, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);
    
    String wifiSsid = (WiFi.status() == WL_CONNECTED) ? ("WiFi: " + WiFi.SSID()) : "WiFi: 未连接";
    drawChineseText(wifiSsid.c_str(), 20, 208, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);

    String ipStr = (WiFi.status() == WL_CONNECTED) ? ("网络: " + WiFi.localIP().toString()) : "网络: 未连接";
    drawChineseText(ipStr.c_str(), 20, 230, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);

    int rssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;
    snprintf(buf, sizeof(buf), "信号: %d dBm", rssi);
    drawChineseText(buf, 20, 252, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);

    uint32_t sec = millis() / 1000;
    uint32_t hrs = sec / 3600;
    uint32_t mins = (sec % 3600) / 60;
    uint32_t secs = sec % 60;
    snprintf(buf, sizeof(buf), "运行: %02u:%02u:%02u", (unsigned)hrs, (unsigned)mins, (unsigned)secs);
    drawChineseText(buf, 20, 274, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
}

/* ---------------- 6. 完整多行设备日志页 ---------------- */
void CustomUiEngine::renderLogs() {
    renderHeader("设备日志");
    bool lan = isLandscape();
    int textBottom = lan ? 228 : 290;
    drawCard(12, 34, lan ? 296 : 216, lan ? 194 : 264);
    
    // 🌟 SRAM 优化：8KB 日志缓冲区改为从 PSRAM 分配，释放 8KB 宝贵的 SRAM 内存
    static char* psramLogBuf = nullptr;
    if (!psramLogBuf) {
        psramLogBuf = (char*)heap_caps_malloc(8192, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    char* logBuf = psramLogBuf;
    if (!logBuf) return;

    size_t copied = DebugLogService::copyWindow(logBuf, 8192, log_scroll_lines);
    if (copied == 0) {
        drawChineseText("暂无日志记录", 20, 44, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
        return;
    }

    u8f.setFont(u8g2_font_wqy13_t_gb2312a);
    int maxLineWidth = lan ? 276 : 196; // 卡片内部文字可用最大像素宽度
    String currentLine = "";
    std::vector<String> visualLines;
    
    for (size_t i = 0; i <= copied; i++) {
        if (logBuf[i] == '\n' || logBuf[i] == '\0') {
            if (currentLine.length() > 0) {
                // 🌟 核心修复 2：保留全量模块标签（[NET]、[SYS]、[AUDIO]、[ERR]等），绝不丢弃任何串口日志
                String remaining = currentLine;
                bool isFirstSubLine = true;

                // 动态计算前缀缩进长度以对齐时间戳
                String padString = "   ";
                int bracketIndex = currentLine.indexOf(']');
                if (bracketIndex != -1 && bracketIndex < 20) {
                    String prefix = currentLine.substring(0, bracketIndex + 1) + " ";
                    int w = u8f.getUTF8Width(prefix.c_str());
                    int sw = u8f.getUTF8Width(" ");
                    if (sw > 0) {
                        int numSpaces = (w + sw / 2) / sw;
                        padString = "";
                        for (int s = 0; s < numSpaces; s++) padString += " ";
                    }
                }

                while (remaining.length() > 0) {
                    size_t charIndex = 0;
                    size_t lastValidLen = 0;

                    // 沿着 UTF-8 字符边界递增，计算不超过 maxLineWidth 的最大子串
                    while (charIndex < remaining.length()) {
                        unsigned char c = (unsigned char)remaining[charIndex];
                        size_t nextIndex = charIndex + 1;
                        if ((c & 0x80) != 0) {
                            if ((c & 0xE0) == 0xC0) nextIndex = charIndex + 2;
                            else if ((c & 0xF0) == 0xE0) nextIndex = charIndex + 3;
                            else if ((c & 0xF8) == 0xF0) nextIndex = charIndex + 4;
                        }
                        if (nextIndex > remaining.length()) nextIndex = remaining.length();

                        String testSub = remaining.substring(0, nextIndex);
                        if (!isFirstSubLine) {
                            testSub = padString + testSub;
                        }
                        
                        if (u8f.getUTF8Width(testSub.c_str()) > maxLineWidth && lastValidLen > 0) {
                            break; // 超过可画最大宽度像素，截断
                        }
                        
                        lastValidLen = nextIndex;
                        charIndex = nextIndex;
                    }

                    if (lastValidLen == 0) lastValidLen = (charIndex > 0) ? charIndex : 1;

                    String subLine = remaining.substring(0, lastValidLen);
                    remaining = remaining.substring(lastValidLen);

                    if (!isFirstSubLine) {
                        subLine = padString + subLine;
                    }
                    isFirstSubLine = false;

                    visualLines.push_back(subLine);
                }
            }
            currentLine = "";
        } else {
            currentLine += logBuf[i];
        }
    }

    int y = 42;
    int maxLinesOnScreen = (textBottom - 42) / 16;
    int totalLines = visualLines.size();
    int startIndex = totalLines - maxLinesOnScreen - log_scroll_lines;
    if (startIndex < 0) startIndex = 0;

    for (int i = startIndex; i < totalLines && (y + 13 < textBottom); i++) {
        drawChineseText(visualLines[i].c_str(), 18, y, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);
        y += 16; // 换行行距 16px
    }
}

/* ---------------- 7. 完整多行附近设备自检树 ---------------- */
void CustomUiEngine::renderDevices() {
    // 局部快照, 遮蔽全局: 加锁读取设备列表, 避免与网络任务扫描时竞争
    String online_devices_list;
    if (state_mutex && xSemaphoreTake(state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        online_devices_list = CustomUiEngine::online_devices_list;
        xSemaphoreGive(state_mutex);
    } else {
        online_devices_list = CustomUiEngine::online_devices_list;
    }
    renderHeader("附近设备扫描");
    bool lan = isLandscape();
    int textBottom = lan ? 228 : 290;
    
    drawActionButton(20, 32, lan ? 280 : 200, 38, "扫描附近设备", CustomUiTheme::PRIMARY_DARK, page_focused);
    drawCard(12, 76, lan ? 296 : 216, lan ? 152 : 222);
    
    if (online_devices_list.length() == 0) {
        drawChineseText("暂未扫描 (点击按钮开始)", 20, 88, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, false);
        return;
    }

    u8f.setFont(u8g2_font_wqy13_t_gb2312a);
    int y = 84;
    int lineCount = 0;
    String currentLine = "";
    
    // 确保最后一行不因缺少换行符而被遗漏
    String fullText = online_devices_list;
    if (!fullText.endsWith("\n")) fullText += "\n";

    for (size_t i = 0; i < fullText.length(); i++) {
        char c = fullText[i];
        if (c == '\n') {
            if (lineCount >= (int)device_scroll_lines && (y + 13 < textBottom)) {
                drawChineseText(currentLine.c_str(), 18, y, CustomUiTheme::TEXT, CustomUiTheme::SURFACE, false);
                y += 18;
            }
            lineCount++;
            currentLine = "";
        } else if (c != '\r') {
            currentLine += c;
        }
    }
}

// 🌟 辅助函数：歌词智能按词折行与居中自适应 (中英日韩全语言通用，优先单行居中，超长自动拆双行居中，深度清理首尾隐形空格，绝对物理对称居中)
static void wrapLyricText(const String& raw_src, int max_w, String& line1, String& line2, bool is_active = false) {
    String src = cleanLyricText(raw_src);
    if (src.length() == 0) {
        line1 = "";
        line2 = "";
        return;
    }

    // 🌟 优先判定单行能否容纳：当前高亮句先试 16px，若超长再试 12px 单行；非高亮句试 12px 单行
    if (is_active) {
        if (getMusicTextWidth(src.c_str(), 0) <= max_w) {
            line1 = src;
            line2 = "";
            return;
        }
        if (getMusicTextWidth(src.c_str(), 4) <= max_w) {
            line1 = src;
            line2 = "";
            return;
        }
    } else {
        if (getMusicTextWidth(src.c_str(), 4) <= max_w) {
            line1 = src;
            line2 = "";
            return;
        }
    }

    // 需要折为双行，统一采用 12px 原生真点阵测量并拆分
    const int font_level = 4;
    size_t s_len = src.length();

    // 1. 判断是否包含空格 (主要为英文、拉丁语等单词分词语言)
    bool has_space = (src.indexOf(' ') >= 0);

    if (has_space) {
        // 按单词贪心拆分：尽可能让 line1 装下更多完整单词，且 line1 <= max_w
        size_t last_fit_pos = 0;
        size_t cur_pos = 0;
        while (cur_pos < s_len) {
            int next_space = src.indexOf(' ', cur_pos);
            size_t token_end = (next_space >= 0) ? (size_t)next_space : s_len;
            String test_line = cleanLyricText(src.substring(0, token_end));
            if (getMusicTextWidth(test_line.c_str(), font_level) <= max_w) {
                last_fit_pos = token_end;
                if (next_space >= 0) cur_pos = next_space + 1;
                else break;
            } else {
                break;
            }
        }

        if (last_fit_pos > 0 && last_fit_pos < s_len) {
            line1 = cleanLyricText(src.substring(0, last_fit_pos));
            line2 = cleanLyricText(src.substring(last_fit_pos));
        } else {
            size_t cidx = 0, fit_cidx = 0;
            while (cidx < s_len) {
                size_t step = getUtf8CharStep(src.c_str(), cidx, s_len);
                if (step == 0) break;
                String test = src.substring(0, cidx + step);
                if (getMusicTextWidth(test.c_str(), font_level) <= max_w) {
                    fit_cidx = cidx + step;
                    cidx += step;
                } else {
                    break;
                }
            }
            line1 = cleanLyricText(src.substring(0, fit_cidx > 0 ? fit_cidx : 1));
            line2 = cleanLyricText(src.substring(fit_cidx > 0 ? fit_cidx : 1));
        }
    } else {
        // 中文 / 日文 / 无空格语言：按字粒度居中折行
        size_t best_split = 0;
        size_t cidx = 0;
        size_t mid_w = getMusicTextWidth(src.c_str(), font_level) / 2;
        size_t closest_mid_split = 0;
        int min_mid_diff = 9999;

        while (cidx < s_len) {
            size_t step = getUtf8CharStep(src.c_str(), cidx, s_len);
            if (step == 0) break;
            String sub = src.substring(0, cidx + step);
            int w = getMusicTextWidth(sub.c_str(), font_level);
            if (w <= max_w) {
                best_split = cidx + step;
                int diff = abs(w - (int)mid_w);
                if (diff < min_mid_diff) {
                    min_mid_diff = diff;
                    closest_mid_split = cidx + step;
                }
            } else {
                break;
            }
            cidx += step;
        }

        String test_l1 = cleanLyricText(src.substring(0, closest_mid_split));
        String test_l2 = cleanLyricText(src.substring(closest_mid_split));
        if (closest_mid_split > 0 &&
            getMusicTextWidth(test_l1.c_str(), font_level) <= max_w &&
            getMusicTextWidth(test_l2.c_str(), font_level) <= max_w) {
            line1 = test_l1;
            line2 = test_l2;
        } else {
            line1 = cleanLyricText(src.substring(0, best_split));
            line2 = cleanLyricText(src.substring(best_split));
        }
    }

    line1 = cleanLyricText(line1);
    line2 = cleanLyricText(line2);

    // 保护：对第二行进行最大宽度保护截断 (多截断 1~2 个字符，加 "..")
    int line2_max_w = max_w - 14;
    if (line2.length() > 0 && getMusicTextWidth(line2.c_str(), font_level) > line2_max_w) {
        size_t l2_len = line2.length();
        int dots_w = getMusicTextWidth("..", font_level);
        int target_w = line2_max_w - dots_w;
        if (target_w < 10) target_w = 10;
        size_t cidx = 0, valid_end = 0;
        while (cidx < l2_len) {
            size_t step = getUtf8CharStep(line2.c_str(), cidx, l2_len);
            if (step == 0) break;
            String test = line2.substring(0, cidx + step);
            if (getMusicTextWidth(test.c_str(), font_level) <= target_w) {
                valid_end = cidx + step;
                cidx += step;
            } else {
                break;
            }
        }
        line2 = cleanLyricText(line2.substring(0, valid_end)) + "..";
    }
}

void CustomUiEngine::renderMusic() {
    renderHeader("音乐播放器");
    u8f.setFontMode(1); // 🌟 全程强制开启透明文字渲染模式
    u8f.setBackgroundColor(CustomUiTheme::SURFACE);

    const int scr_w = screenWidth();
    const int scr_h = screenHeight();
    const bool is_land = isLandscape();

    const MusicPlayerService::SongItem* song = MusicPlayerService::getCurrentSong();
    String title = song ? song->title : "暂无歌曲";
    String artist = song ? song->artist : "请在 STM32 插入 SD 卡";
    String lPrev, lCurr, lNext;
    int displaySec = MusicPlayerService::getCurrentSeconds();
    if (music_adjusting_progress) {
        displaySec = s_preview_seek_sec;
        MusicPlayerService::getLyrics3LinesForTime(s_preview_seek_sec, lPrev, lCurr, lNext);
    } else {
        MusicPlayerService::getLyrics3Lines(lPrev, lCurr, lNext);
    }
    const bool playing = MusicPlayerService::isPlaying();
    const int curSec = displaySec;
    const int totSec = MusicPlayerService::getTotalSeconds();
    const int vol = MusicPlayerService::getVolume();
    const char* modeName = MusicPlayerService::getPlayModeName();
    const int trackIdx = MusicPlayerService::getCurrentTrackIndex();
    const int totalTracks = MusicPlayerService::getPlaylistCount();

    // 🌟 歌曲名与歌手合并为单行展示 (如 "说谎 - 林宥嘉" / "always online - 林俊杰 (jj Lin)")
    String fullTitle = title;
    if (artist.length() > 0 && artist != "未知歌手" && artist != "请在 STM32 插入 SD 卡" && title.indexOf(artist) == -1) {
        fullTitle = title + " - " + artist;
    }

    if (!is_land) {
        // ========== 竖屏模式 (240 x 320) ==========
        // 卡片 1: 歌曲信息、六行折行歌词与音频跳动频谱区 (Y=30 ~ 216, 高度 186)
        const int c1_x = 8;
        const int c1_y = 30;
        const int c1_w = 224;
        const int c1_h = 186;
        drawCard(c1_x, c1_y, c1_w, c1_h);

        // 🌟 1. 顶部标题栏重构 (加高至 30px, 左右各预留 34px 对称区域, 序号显示为优雅的 [分子/横线/分母] 格式)
        const int badge_w = 34;
        const int badge_x = c1_x + 6;

        char numBuf[8], denBuf[8];
        if (totalTracks == 0) {
            snprintf(numBuf, sizeof(numBuf), "00");
            snprintf(denBuf, sizeof(denBuf), "00");
        } else {
            if (trackIdx + 1 > 99) snprintf(numBuf, sizeof(numBuf), "%d", trackIdx + 1);
            else snprintf(numBuf, sizeof(numBuf), "%02d", trackIdx + 1);

            if (totalTracks > 99) snprintf(denBuf, sizeof(denBuf), "%d", totalTracks);
            else snprintf(denBuf, sizeof(denBuf), "%02d", totalTracks);
        }

        u8f.setFontMode(1);
        u8f.setBackgroundColor(CustomUiTheme::SURFACE);
        u8f.setFont(u8g2_font_wqy12_t_gb2312a);

        // 分子 (金色高亮)
        u8f.setForegroundColor(CustomUiTheme::FOCUS_GOLD);
        int nw = u8f.getUTF8Width(numBuf);
        u8f.drawUTF8(badge_x + (badge_w - nw) / 2, c1_y + 11, numBuf);

        // 分数横线
        spr.drawFastHLine(badge_x + 3, c1_y + 14, badge_w - 6, CustomUiTheme::BORDER);

        // 分母 (浅灰沉稳)
        u8f.setForegroundColor(CustomUiTheme::MUTED);
        int dw = u8f.getUTF8Width(denBuf);
        u8f.drawUTF8(badge_x + (badge_w - dw) / 2, c1_y + 25, denBuf);

        // 🌟 中间歌曲名+歌手跑马灯区域 (左右各预留 badge_w + 4 对称安全间距，绝对隔离零重叠！)
        const int title_box_x = badge_x + badge_w + 4;
        const int title_box_w = (c1_x + c1_w - 6 - badge_w - 4) - title_box_x;
        drawMarqueeText(fullTitle.c_str(), title_box_x, c1_y + 5, title_box_w, 20, CustomUiTheme::FOCUS_GOLD, u8g2_font_wqy15_t_gb2312a, true);

        // 分隔横线
        spr.drawFastHLine(c1_x + 6, c1_y + 28, c1_w - 12, CustomUiTheme::BORDER);

        // 2. 六行折行歌词渲染区 (双行智能换行 + 动态垂直居中自适应，第二行严格居中，预留充裕 184px 安全宽度)
        String p1, p2, c1, c2, n1, n2;
        wrapLyricText(lPrev, 184, p1, p2, false);
        wrapLyricText(lCurr, 184, c1, c2, true);
        wrapLyricText(lNext, 184, n1, n2, false);

        // 动态计算歌词总高度与居中起始 Y 坐标 (可用区域: Y=60 ~ 184, 共 124px)
        const int lyric_box_top = c1_y + 30;
        const int lyric_box_h = 124;

        // 智能判定当前句字体大小：单行且宽度充裕用 16px，超长单行或双行用 12px 饱满原生真点阵
        int c_font = (c2.length() == 0 && getMusicTextWidth(c1.c_str(), 0) <= 184) ? 0 : 4;
        int p_h = (p2.length() > 0) ? 28 : ((p1.length() > 0) ? 14 : 0);
        int c_h = (c2.length() > 0) ? 28 : ((c1.length() > 0) ? (c_font == 0 ? 16 : 14) : 0);
        int n_h = (n2.length() > 0) ? 28 : ((n1.length() > 0) ? 14 : 0);

        int gap1 = (p_h > 0 && c_h > 0) ? 10 : 0;
        int gap2 = (c_h > 0 && n_h > 0) ? 10 : 0;
        int total_h = p_h + gap1 + c_h + gap2 + n_h;
        if (total_h == 0) total_h = 16;

        int start_y = lyric_box_top + (lyric_box_h - total_h) / 2;
        if (start_y < lyric_box_top + 2) start_y = lyric_box_top + 2;

        int cur_y = start_y;
        // 上一句 (淡灰，换行后每行均绝对居中)
        if (p1.length() > 0) {
            drawMusicChineseTextCentered(p1.c_str(), c1_x + 6, cur_y, c1_w - 12, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, 4);
            if (p2.length() > 0) {
                drawMusicChineseTextCentered(p2.c_str(), c1_x + 6, cur_y + 14, c1_w - 12, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, 4);
                cur_y += 28 + gap1;
            } else {
                cur_y += 14 + gap1;
            }
        }

        // 当前句 (高亮金色/主题蓝，单行/双行换行后严格绝对居中，两端留有充足安全边距)
        uint16_t currColor = playing ? CustomUiTheme::PRIMARY : CustomUiTheme::FOCUS_GOLD;
        if (c1.length() > 0) {
            drawMusicChineseTextCentered(c1.c_str(), c1_x + 6, cur_y, c1_w - 12, currColor, CustomUiTheme::SURFACE, c_font);
            if (c2.length() > 0) {
                drawMusicChineseTextCentered(c2.c_str(), c1_x + 6, cur_y + 14, c1_w - 12, currColor, CustomUiTheme::SURFACE, c_font);
                cur_y += 28 + gap2;
            } else {
                cur_y += (c_font == 0 ? 16 : 14) + gap2;
            }
        }

        // 下一句 (淡灰，换行后每行均绝对居中)
        if (n1.length() > 0 && cur_y < c1_y + 156) {
            drawMusicChineseTextCentered(n1.c_str(), c1_x + 6, cur_y, c1_w - 12, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, 4);
            if (n2.length() > 0 && (cur_y + 14) < c1_y + 156) {
                drawMusicChineseTextCentered(n2.c_str(), c1_x + 6, cur_y + 14, c1_w - 12, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, 4);
            }
        }

        // 3. 底部 16 根跳动动态音频频谱 (高度 24px，底边距 4px，带浮顶悬停顶针)
        uint8_t spec[16] = {0};
        uint8_t peaks[16] = {0};
        MusicPlayerService::getSpectrumBars(spec);
        MusicPlayerService::getSpectrumPeaks(peaks);
        const int bar_w = 9;
        const int bar_gap = 4;
        const int spec_start_x = c1_x + (c1_w - (16 * bar_w + 15 * bar_gap)) / 2;
        const int spec_base_y = c1_y + c1_h - 4;
        for (int i = 0; i < 16; i++) {
            int bh = (spec[i] * 24) / 100;
            if (bh < 2) bh = 2;
            int bx = spec_start_x + i * (bar_w + bar_gap);
            uint16_t bcolor = (i < 4) ? CustomUiTheme::SUCCESS : ((i < 10) ? CustomUiTheme::PRIMARY : CustomUiTheme::PURPLE);
            spr.fillRect(bx, spec_base_y - bh, bar_w, bh, bcolor);

            // 🌟 浮顶峰值悬停顶针 (Peak Dot)
            int ph = (peaks[i] * 24) / 100;
            if (ph > bh + 1 && ph <= 24) {
                spr.fillRect(bx, spec_base_y - ph - 1, bar_w, 2, CustomUiTheme::FOCUS_GOLD);
            }
        }

        // 卡片 2: 紧凑压缩型播放控制区 (Y=220 ~ 296, 高度 76)
        const int c2_x = 8;
        const int c2_y = 220;
        const int c2_w = 224;
        const int c2_h = 76;
        drawCard(c2_x, c2_y, c2_w, c2_h);

        // 1. 时间显示与进度条
        char timeStr[48];
        snprintf(timeStr, sizeof(timeStr), "%02d:%02d / %02d:%02d", curSec / 60, curSec % 60, totSec / 60, totSec % 60);
        u8f.setFontMode(1);
        u8f.setBackgroundColor(CustomUiTheme::SURFACE);
        u8f.setFont(u8g2_font_wqy12_t_gb2312a);
        u8f.setForegroundColor(page_focused && music_tier == MUSIC_TIER_PROGRESS ? (music_adjusting_progress ? CustomUiTheme::WARNING : CustomUiTheme::FOCUS_GOLD) : CustomUiTheme::MUTED);
        u8f.drawUTF8(c2_x + 8, c2_y + 11, timeStr);

        int pb_x = c2_x + 94;
        int pb_y = c2_y + 5;
        int pb_w = c2_w - 102;
        int pb_h = 5;
        uint32_t pct = (totSec > 0) ? (curSec * 100 / totSec) : 0;
        drawProgressBar(pb_x, pb_y, pb_w, pb_h, pct, CustomUiTheme::PRIMARY);
        if (page_focused && music_tier == MUSIC_TIER_PROGRESS) {
            spr.drawRect(pb_x - 2, pb_y - 2, pb_w + 4, pb_h + 4, music_adjusting_progress ? CustomUiTheme::WARNING : CustomUiTheme::FOCUS_GOLD);
        }

        // 2. 六大标准控制按键: [连接] [模式] [上曲] [播/暂] [下曲] [歌单]
        const int btn_w = 32;
        const int btn_h = 24;
        const int btn_gap = 4;
        const int btn_start_x = c2_x + (c2_w - (6 * btn_w + 5 * btn_gap)) / 2;
        const int btn_y = c2_y + 18;

        const bool btn_tier_active = page_focused && (music_tier == MUSIC_TIER_BUTTONS);
        const bool connected = MusicPlayerService::isConnected();
        drawActionButton(btn_start_x + 0 * (btn_w + btn_gap), btn_y, btn_w, btn_h, connected ? "已连" : "连接", connected ? CustomUiTheme::SUCCESS : CustomUiTheme::PRIMARY_DARK, btn_tier_active && (music_btn_idx == 0), 1);
        drawActionButton(btn_start_x + 1 * (btn_w + btn_gap), btn_y, btn_w, btn_h, modeName, CustomUiTheme::PRIMARY_DARK, btn_tier_active && (music_btn_idx == 1), 1);
        drawActionButton(btn_start_x + 2 * (btn_w + btn_gap), btn_y, btn_w, btn_h, "上曲", CustomUiTheme::PRIMARY_DARK, btn_tier_active && (music_btn_idx == 2), 1);
        drawActionButton(btn_start_x + 3 * (btn_w + btn_gap), btn_y, btn_w, btn_h, playing ? "暂停" : "播放", playing ? CustomUiTheme::SUCCESS : CustomUiTheme::FOCUS_GOLD, btn_tier_active && (music_btn_idx == 3), 1);
        drawActionButton(btn_start_x + 4 * (btn_w + btn_gap), btn_y, btn_w, btn_h, "下曲", CustomUiTheme::PRIMARY_DARK, btn_tier_active && (music_btn_idx == 4), 1);
        drawActionButton(btn_start_x + 5 * (btn_w + btn_gap), btn_y, btn_w, btn_h, "歌单", CustomUiTheme::PRIMARY_DARK, btn_tier_active && (music_btn_idx == 5), 1);

        // 3. 音量调节条
        char volStr[24];
        snprintf(volStr, sizeof(volStr), "音量: %d%%", vol);
        u8f.setFontMode(1);
        u8f.setBackgroundColor(CustomUiTheme::SURFACE);
        u8f.setFont(u8g2_font_wqy12_t_gb2312a);
        u8f.setForegroundColor(page_focused && music_tier == MUSIC_TIER_VOLUME ? (music_adjusting_volume ? CustomUiTheme::WARNING : CustomUiTheme::FOCUS_GOLD) : CustomUiTheme::MUTED);
        u8f.drawUTF8(c2_x + 8, c2_y + 60, volStr);

        int vb_x = c2_x + 94;
        int vb_y = c2_y + 54;
        int vb_w = c2_w - 102;
        int vb_h = 5;
        drawProgressBar(vb_x, vb_y, vb_w, vb_h, vol, CustomUiTheme::SUCCESS);
        if (page_focused && music_tier == MUSIC_TIER_VOLUME) {
            spr.drawRect(vb_x - 2, vb_y - 2, vb_w + 4, vb_h + 4, music_adjusting_volume ? CustomUiTheme::WARNING : CustomUiTheme::FOCUS_GOLD);
        }
    } else {
        // ========== 横屏模式 (320 x 240) · 标题分子分母对称 & 控制卡片整体下移 ==========
        // 卡片 1: 歌曲信息、歌词与频谱区 (Y=24 ~ 162, 高度扩容至 138)
        const int mc_x = 8;
        const int mc_y = 24;
        const int mc_w = 304;
        const int mc_h = 138;
        drawCard(mc_x, mc_y, mc_w, mc_h);

        // 🌟 1. 顶部信息栏: 左侧序号 [分子/横线/分母] + 左右预留 34px 对称空间
        const int badge_w = 34;
        const int badge_x = mc_x + 6;

        char numBuf[8], denBuf[8];
        if (totalTracks == 0) {
            snprintf(numBuf, sizeof(numBuf), "00");
            snprintf(denBuf, sizeof(denBuf), "00");
        } else {
            if (trackIdx + 1 > 99) snprintf(numBuf, sizeof(numBuf), "%d", trackIdx + 1);
            else snprintf(numBuf, sizeof(numBuf), "%02d", trackIdx + 1);

            if (totalTracks > 99) snprintf(denBuf, sizeof(denBuf), "%d", totalTracks);
            else snprintf(denBuf, sizeof(denBuf), "%02d", totalTracks);
        }

        u8f.setFontMode(1);
        u8f.setBackgroundColor(CustomUiTheme::SURFACE);
        u8f.setFont(u8g2_font_wqy12_t_gb2312a);

        // 分子
        u8f.setForegroundColor(CustomUiTheme::FOCUS_GOLD);
        int nw = u8f.getUTF8Width(numBuf);
        u8f.drawUTF8(badge_x + (badge_w - nw) / 2, mc_y + 11, numBuf);

        // 分数横线
        spr.drawFastHLine(badge_x + 3, mc_y + 14, badge_w - 6, CustomUiTheme::BORDER);

        // 分母
        u8f.setForegroundColor(CustomUiTheme::MUTED);
        int dw = u8f.getUTF8Width(denBuf);
        u8f.drawUTF8(badge_x + (badge_w - dw) / 2, mc_y + 25, denBuf);

        // 中间跑马灯 (左右对称，严格在 title_box_w 内居中/滚动)
        const int title_box_x = badge_x + badge_w + 4;
        const int title_box_w = (mc_x + mc_w - 6 - badge_w - 4) - title_box_x;
        drawMarqueeText(fullTitle.c_str(), title_box_x, mc_y + 5, title_box_w, 20, CustomUiTheme::FOCUS_GOLD, u8g2_font_wqy15_t_gb2312a, true);

        // 分隔横线
        spr.drawFastHLine(mc_x + 6, mc_y + 28, mc_w - 12, CustomUiTheme::BORDER);

        // 2. 歌词渲染区 (动态垂直居中自适应布局，第二行严格居中)
        String p1, p2, c1, c2, n1, n2;
        wrapLyricText(lPrev, mc_w - 36, p1, p2, false);
        wrapLyricText(lCurr, mc_w - 36, c1, c2, true);
        wrapLyricText(lNext, mc_w - 36, n1, n2, false);

        const int lyric_box_top_l = mc_y + 30;
        const int lyric_box_h_l = 84;

        int c_font = (c2.length() == 0 && getMusicTextWidth(c1.c_str(), 0) <= (mc_w - 36)) ? 0 : 4;
        int p_h = (p2.length() > 0) ? 28 : ((p1.length() > 0) ? 14 : 0);
        int c_h = (c2.length() > 0) ? 28 : ((c1.length() > 0) ? (c_font == 0 ? 16 : 14) : 0);
        int n_h = (n2.length() > 0) ? 28 : ((n1.length() > 0) ? 14 : 0);

        int gap1 = (p_h > 0 && c_h > 0) ? 8 : 0;
        int gap2 = (c_h > 0 && n_h > 0) ? 8 : 0;
        int total_h = p_h + gap1 + c_h + gap2 + n_h;
        if (total_h == 0) total_h = 16;

        int start_y = lyric_box_top_l + (lyric_box_h_l - total_h) / 2;
        if (start_y < lyric_box_top_l + 2) start_y = lyric_box_top_l + 2;

        int cur_y = start_y;
        if (p1.length() > 0) {
            drawMusicChineseTextCentered(p1.c_str(), mc_x + 6, cur_y, mc_w - 12, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, 4);
            if (p2.length() > 0) {
                drawMusicChineseTextCentered(p2.c_str(), mc_x + 6, cur_y + 14, mc_w - 12, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, 4);
                cur_y += 28 + gap1;
            } else {
                cur_y += 14 + gap1;
            }
        }

        uint16_t currColor_l = playing ? CustomUiTheme::PRIMARY : CustomUiTheme::FOCUS_GOLD;
        if (c1.length() > 0) {
            drawMusicChineseTextCentered(c1.c_str(), mc_x + 6, cur_y, mc_w - 12, currColor_l, CustomUiTheme::SURFACE, c_font);
            if (c2.length() > 0) {
                drawMusicChineseTextCentered(c2.c_str(), mc_x + 6, cur_y + 14, mc_w - 12, currColor_l, CustomUiTheme::SURFACE, c_font);
                cur_y += 28 + gap2;
            } else {
                cur_y += (c_font == 0 ? 16 : 14) + gap2;
            }
        }

        if (n1.length() > 0 && cur_y < mc_y + 114) {
            drawMusicChineseTextCentered(n1.c_str(), mc_x + 6, cur_y, mc_w - 12, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, 4);
            if (n2.length() > 0 && (cur_y + 14) < mc_y + 114) {
                drawMusicChineseTextCentered(n2.c_str(), mc_x + 6, cur_y + 14, mc_w - 12, CustomUiTheme::MUTED, CustomUiTheme::SURFACE, 4);
            }
        }

        // 3. 20 柱宽屏跳动动态音频频谱 (高度 20px，带浮顶悬停顶针)
        uint8_t spec[16] = {0};
        uint8_t peaks[16] = {0};
        MusicPlayerService::getSpectrumBars(spec);
        MusicPlayerService::getSpectrumPeaks(peaks);
        const int bar_w = 9;
        const int bar_gap = 4;
        const int spec_total_w = 20 * bar_w + 19 * bar_gap;
        const int spec_start_x = mc_x + (mc_w - spec_total_w) / 2;
        const int spec_base_y = mc_y + mc_h - 4;
        for (int i = 0; i < 20; i++) {
            int s_idx = (i < 16) ? i : (31 - i);
            int bh = (spec[s_idx] * 20) / 100;
            if (bh < 2) bh = 2;
            int bx = spec_start_x + i * (bar_w + bar_gap);
            uint16_t bcolor = (i < 5) ? CustomUiTheme::SUCCESS : ((i < 15) ? CustomUiTheme::PRIMARY : CustomUiTheme::PURPLE);
            spr.fillRect(bx, spec_base_y - bh, bar_w, bh, bcolor);

            // 🌟 浮顶峰值悬停顶针 (Peak Dot)
            int ph = (peaks[s_idx] * 20) / 100;
            if (ph > bh + 1 && ph <= 20) {
                spr.fillRect(bx, spec_base_y - ph - 1, bar_w, 2, CustomUiTheme::FOCUS_GOLD);
            }
        }

        // 卡片 2: 控制区域整体下移 (Y=166 ~ 234, 高度 68, 底部空间充分利用)
        const int mc2_x = 8;
        const int mc2_y = 166;
        const int mc2_w = 304;
        const int mc2_h = 68;
        drawCard(mc2_x, mc2_y, mc2_w, mc2_h);

        // 1. 时间 + 进度条
        char timeStr[48];
        snprintf(timeStr, sizeof(timeStr), "%02d:%02d / %02d:%02d", curSec / 60, curSec % 60, totSec / 60, totSec % 60);
        u8f.setFontMode(1);
        u8f.setBackgroundColor(CustomUiTheme::SURFACE);
        u8f.setFont(u8g2_font_wqy12_t_gb2312a);
        u8f.setForegroundColor(page_focused && music_tier == MUSIC_TIER_PROGRESS ? (music_adjusting_progress ? CustomUiTheme::WARNING : CustomUiTheme::FOCUS_GOLD) : CustomUiTheme::MUTED);
        u8f.drawUTF8(mc2_x + 8, mc2_y + 10, timeStr);

        int pb_x = mc2_x + 104;
        int pb_y = mc2_y + 5;
        int pb_w = mc2_w - 114;
        int pb_h = 5;
        uint32_t pct = (totSec > 0) ? (curSec * 100 / totSec) : 0;
        drawProgressBar(pb_x, pb_y, pb_w, pb_h, pct, CustomUiTheme::PRIMARY);
        if (page_focused && music_tier == MUSIC_TIER_PROGRESS) {
            spr.drawRect(pb_x - 2, pb_y - 2, pb_w + 4, pb_h + 4, music_adjusting_progress ? CustomUiTheme::WARNING : CustomUiTheme::FOCUS_GOLD);
        }

        // 2. 六大控制按钮横向排开 (高度 22px，布局紧凑美观)
        const int btn_w = 42;
        const int btn_h = 22;
        const int btn_gap = 6;
        const int btn_start_x = mc2_x + (mc2_w - (6 * btn_w + 5 * btn_gap)) / 2;
        const int btn_y = mc2_y + 17;

        const bool btn_tier_active = page_focused && (music_tier == MUSIC_TIER_BUTTONS);
        const bool connected_l = MusicPlayerService::isConnected();
        drawActionButton(btn_start_x + 0 * (btn_w + btn_gap), btn_y, btn_w, btn_h, connected_l ? "已连" : "连接", connected_l ? CustomUiTheme::SUCCESS : CustomUiTheme::PRIMARY_DARK, btn_tier_active && (music_btn_idx == 0), 1);
        drawActionButton(btn_start_x + 1 * (btn_w + btn_gap), btn_y, btn_w, btn_h, modeName, CustomUiTheme::PRIMARY_DARK, btn_tier_active && (music_btn_idx == 1), 1);
        drawActionButton(btn_start_x + 2 * (btn_w + btn_gap), btn_y, btn_w, btn_h, "上曲", CustomUiTheme::PRIMARY_DARK, btn_tier_active && (music_btn_idx == 2), 1);
        drawActionButton(btn_start_x + 3 * (btn_w + btn_gap), btn_y, btn_w, btn_h, playing ? "暂停" : "播放", playing ? CustomUiTheme::SUCCESS : CustomUiTheme::FOCUS_GOLD, btn_tier_active && (music_btn_idx == 3), 1);
        drawActionButton(btn_start_x + 4 * (btn_w + btn_gap), btn_y, btn_w, btn_h, "下曲", CustomUiTheme::PRIMARY_DARK, btn_tier_active && (music_btn_idx == 4), 1);
        drawActionButton(btn_start_x + 5 * (btn_w + btn_gap), btn_y, btn_w, btn_h, "歌单", CustomUiTheme::PRIMARY_DARK, btn_tier_active && (music_btn_idx == 5), 1);

        // 3. 音量调节条
        char volStr[24];
        snprintf(volStr, sizeof(volStr), "音量: %d%%", vol);
        u8f.setFontMode(1);
        u8f.setBackgroundColor(CustomUiTheme::SURFACE);
        u8f.setFont(u8g2_font_wqy12_t_gb2312a);
        u8f.setForegroundColor(page_focused && music_tier == MUSIC_TIER_VOLUME ? (music_adjusting_volume ? CustomUiTheme::WARNING : CustomUiTheme::FOCUS_GOLD) : CustomUiTheme::MUTED);
        u8f.drawUTF8(mc2_x + 8, mc2_y + 52, volStr);

        int vb_x = mc2_x + 104;
        int vb_y = mc2_y + 47;
        int vb_w = mc2_w - 114;
        int vb_h = 5;
        drawProgressBar(vb_x, vb_y, vb_w, vb_h, vol, CustomUiTheme::SUCCESS);
        if (page_focused && music_tier == MUSIC_TIER_VOLUME) {
            spr.drawRect(vb_x - 2, vb_y - 2, vb_w + 4, vb_h + 4, music_adjusting_volume ? CustomUiTheme::WARNING : CustomUiTheme::FOCUS_GOLD);
        }
    }

    // ========== 悬浮歌单抽屉 (Playlist Modal / Drawer) ==========
    if (music_playlist_modal) {
        // 🌟 尺寸与两张卡片完美重合（竖屏 Y=30~296 高266，横屏 Y=24~234 高210），100% 严密遮挡底层，绝不漏缝
        const int mx = 8;
        const int my = is_land ? 24 : 30;
        const int mw = is_land ? 304 : 224;
        const int mh = is_land ? 210 : 266;

        drawCard(mx, my, mw, mh, 0x0841, CustomUiTheme::FOCUS_GOLD);

        // 歌单标题
        u8f.setFontMode(1);
        u8f.setBackgroundColor(0x0841);
        u8f.setFont(u8g2_font_wqy13_t_gb2312a);
        u8f.setForegroundColor(CustomUiTheme::FOCUS_GOLD);
        char listHeader[48];
        snprintf(listHeader, sizeof(listHeader), "🎵 SD卡 歌单列表 (%d首)", totalTracks);
        u8f.drawUTF8(mx + 10, my + 18, listHeader);
        spr.drawFastHLine(mx + 6, my + 24, mw - 12, CustomUiTheme::BORDER);

        // 歌曲列表条目 (若为空则居中显示暂无歌单占位)
        if (totalTracks == 0) {
            u8f.setFontMode(1);
            u8f.setBackgroundColor(0x0841);
            u8f.setFont(u8g2_font_wqy13_t_gb2312a);
            u8f.setForegroundColor(CustomUiTheme::MUTED);
            const char* noSongTip = "暂无歌单 · 请点击连接同步SD卡";
            int nsw = u8f.getUTF8Width(noSongTip);
            u8f.drawUTF8(mx + (mw - nsw) / 2, my + mh / 2, noSongTip);
        } else {
            const int item_h = is_land ? 26 : 31;
            const int visible_count = (mh - 30) / item_h;
            for (int i = 0; i < visible_count; i++) {
                int idx = music_playlist_scroll + i;
                if (idx >= totalTracks) break;
                const MusicPlayerService::SongItem* sitem = MusicPlayerService::getSong(idx);
                if (!sitem) continue;

                int iy = my + 28 + i * item_h;
                bool is_cur = (idx == trackIdx);
                bool is_foc = (idx == music_playlist_focus);

                if (is_foc) {
                    spr.fillRoundRect(mx + 4, iy, mw - 8, item_h - 2, 4, CustomUiTheme::PRIMARY_DARK);
                    spr.drawRoundRect(mx + 4, iy, mw - 8, item_h - 2, 4, CustomUiTheme::FOCUS_GOLD);
                }

                // 右侧时长文本
                char durStr[16];
                snprintf(durStr, sizeof(durStr), "%02d:%02d", sitem->durationSec / 60, sitem->durationSec % 60);
                u8f.setFont(u8g2_font_wqy12_t_gb2312a);
                int dw = u8f.getUTF8Width(durStr);
                u8f.setForegroundColor(CustomUiTheme::MUTED);
                u8f.setBackgroundColor(is_foc ? CustomUiTheme::PRIMARY_DARK : 0x0841);
                int dur_x = mx + mw - dw - 8;
                u8f.drawUTF8(dur_x, iy + (item_h - 12) / 2 + 10, durStr);

                // 左侧前缀 (如 "▶ 01. " 或 "  01. ")
                char prefixBuf[16];
                snprintf(prefixBuf, sizeof(prefixBuf), "%s%02d. ", is_cur ? "▶ " : "  ", idx + 1);
                u8f.setFont(u8g2_font_wqy13_t_gb2312a);
                int pw = u8f.getUTF8Width(prefixBuf);
                u8f.setForegroundColor(is_cur ? CustomUiTheme::FOCUS_GOLD : (is_foc ? CustomUiTheme::TEXT : CustomUiTheme::MUTED));
                u8f.setBackgroundColor(is_foc ? CustomUiTheme::PRIMARY_DARK : 0x0841);
                u8f.drawUTF8(mx + 8, iy + (item_h - 13) / 2 + 11, prefixBuf);

                // 🌟 歌曲名称 + 歌手信息：展示为 "歌名 - 歌手"
                String fullSongName = sitem->title;
                if (sitem->artist.length() > 0 && sitem->artist != "未知歌手" && sitem->title.indexOf(sitem->artist) == -1) {
                    fullSongName = sitem->title + " - " + sitem->artist;
                }

                // 歌曲名称区域：计算最大可用宽度 (与右侧时长保留 14px 宽裕物理隔离，多截断 1~2 个字彻底防溢出)
                int title_box_x = mx + 8 + pw + 2;
                int max_allowed_w = (dur_x - 14) - title_box_x;
                if (max_allowed_w < 20) max_allowed_w = 20;

                uint16_t title_color = is_cur ? CustomUiTheme::FOCUS_GOLD : (is_foc ? CustomUiTheme::TEXT : CustomUiTheme::MUTED);
                uint16_t item_bg = is_foc ? CustomUiTheme::PRIMARY_DARK : 0x0841;

                // 🌟 歌曲名称显示：未超长直接完整显示；超长且被选中焦点项滚动跑马灯显示；未选中焦点项多截断 1~2 个字加 ".."
                int tw = getMusicTextWidth(fullSongName.c_str(), 4);
                int text_y = iy + (item_h - 12) / 2;
                if (tw <= max_allowed_w) {
                    drawMusicChineseText(fullSongName.c_str(), title_box_x, text_y, title_color, item_bg, 4);
                } else {
                    if (is_foc) {
                        // 🌟 选中焦点项超长：自动平滑跑马灯无缝滚动显示完整歌名与歌手！
                        drawMarqueeText(fullSongName.c_str(), title_box_x, iy + (item_h - 16) / 2, max_allowed_w, 16, title_color, u8g2_font_wqy12_t_gb2312a, false, item_bg);
                    } else {
                        // 未选中项：静态 ".." 截断 (多截断 1~2 个字，预留充足空白边距)
                        size_t s_len = fullSongName.length();
                        int dots_w = getMusicTextWidth("..", 4);
                        // 🌟 额外扣除 14px (约 1~2 个字符宽度)，确保末尾标点/括号留出大安全距离
                        int target_w = max_allowed_w - dots_w - 14;
                        if (target_w < 10) target_w = 10;

                        size_t cidx = 0, best_end = 0;
                        while (cidx < s_len) {
                            size_t step = getUtf8CharStep(fullSongName.c_str(), cidx, s_len);
                            if (step == 0) break;
                            String test = fullSongName.substring(0, cidx + step);
                            if (getMusicTextWidth(test.c_str(), 4) <= target_w) {
                                best_end = cidx + step;
                                cidx += step;
                            } else {
                                break;
                            }
                        }
                        String sub = fullSongName.substring(0, best_end) + "..";
                        drawMusicChineseText(sub.c_str(), title_box_x, text_y, title_color, item_bg, 4);
                    }
                }
            }
        }
    }
}
