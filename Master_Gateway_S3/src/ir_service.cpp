#include "ir_service.h"
#include "app_config.h"
#include "custom_ui_engine.h"
#include "debug_log_service.h"
#define Serial DebugLog
#include <IRrecv.h>
#include <IRremoteESP8266.h>
#include <IRutils.h>
#include <Preferences.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <driver/gpio.h>

namespace IRService {

const uint16_t kRecvPin = static_cast<uint16_t>(AppConfig::IR_RECV_PIN);
IRrecv irrecv(kRecvPin);
decode_results results;
Preferences prefs;

int active_profile = 0; // 0 = 00FF Preset, 1 = Custom Profile 1, 2 = Custom Profile 2
bool sequential_mode = false;

uint64_t code_up = 0;
uint64_t code_down = 0;
uint64_t code_left = 0;
uint64_t code_right = 0;
uint64_t code_enter = 0;
uint64_t code_esc = 0;
uint64_t code_standby = 0;
uint64_t code_poweroff = 0;

uint64_t last_recv_code = 0;

IRPairingState pairing_state = IR_PAIRING_NONE;

bool flag_up = false;
bool flag_down = false;
bool flag_left = false;
bool flag_right = false;
bool flag_enter = false;
bool flag_esc = false;
bool flag_standby = false;
bool flag_poweroff = false;

unsigned long last_up_time = 0;
unsigned long last_down_time = 0;
unsigned long last_left_time = 0;
unsigned long last_right_time = 0;
unsigned long last_enter_time = 0;
unsigned long last_esc_time = 0;
unsigned long last_standby_time = 0;
unsigned long last_poweroff_time = 0;
const unsigned long KEY_HOLD_TIME = 150; // ms
unsigned long last_pair_time = 0;

bool ir_shielded = false;

void CustomUiEngine_notifyUiNeedsUpdate_helper();

void loadActiveProfile() {
    if (active_profile == 0) {
        // User's exact verified remote keycodes:
        code_poweroff = 0x00FFA25D; // 红色 POWER 关机键 (0xA2) -> 深度关机
        code_up       = 0x00FF629D; // Mode 键 (0x62) -> 上
        code_standby  = 0x00FFE21D; // 静音 MUTE 待机键 (0xE2) -> 息屏待机
        code_left     = 0x00FF22DD; // 左键 (0x22) -> 左
        code_enter    = 0x00FF02FD; // 中键/确认键 (0x02) -> 确认
        code_right    = 0x00FFC23D; // 蓝色右键 (0xC2) -> 右
        code_down     = 0x00FFA857; // VOL- 键 (0xA8) -> 下
        code_esc      = 0x00FFE01F; // 紫色 EQ 退出键 (0xE0) -> 退出/返回
    } else if (active_profile == 1) {
        code_poweroff = prefs.getULong64("p1_poweroff", 0x00FFA25D);
        code_up       = prefs.getULong64("p1_up", 0x00FF629D);
        code_standby  = prefs.getULong64("p1_standby", 0x00FFE21D);
        code_left     = prefs.getULong64("p1_left", 0x00FF22DD);
        code_enter    = prefs.getULong64("p1_enter", 0x00FF02FD);
        code_right    = prefs.getULong64("p1_right", 0x00FFC23D);
        code_down     = prefs.getULong64("p1_down", 0x00FFA857);
        code_esc      = prefs.getULong64("p1_esc", 0x00FFE01F);
    } else if (active_profile == 2) {
        code_poweroff = prefs.getULong64("p2_poweroff", 0x00FFA25D);
        code_up       = prefs.getULong64("p2_up", 0x00FF629D);
        code_standby  = prefs.getULong64("p2_standby", 0x00FFE21D);
        code_left     = prefs.getULong64("p2_left", 0x00FF22DD);
        code_enter    = prefs.getULong64("p2_enter", 0x00FF02FD);
        code_right    = prefs.getULong64("p2_right", 0x00FFC23D);
        code_down     = prefs.getULong64("p2_down", 0x00FFA857);
        code_esc      = prefs.getULong64("p2_esc", 0x00FFE01F);
    }
}

void setActiveProfile(int profile) {
    if (profile < 0 || profile > 2) profile = 0;
    active_profile = profile;
    prefs.putInt("profile", active_profile);
    loadActiveProfile();
    Serial.printf("[IR] Active Profile switched to: %d\n", active_profile);
}

int getActiveProfile() {
    return active_profile;
}

void init() {
    rtc_gpio_deinit(static_cast<gpio_num_t>(AppConfig::IR_RECV_PIN));
    pinMode(static_cast<int>(AppConfig::IR_RECV_PIN), INPUT_PULLUP);
    
    irrecv.enableIRIn();
    irrecv.resume();
    
    prefs.begin("ir_keys", false);
    // 读取上次保存的自定义配置档; 首次使用默认 Profile 0
    active_profile = prefs.getInt("profile", 0);
    if (active_profile < 0 || active_profile > 2) active_profile = 0;
    loadActiveProfile();
    
}

void resetRecvBuffer() {
    // 1. 等待红外接收引脚恢复高电平(空闲态)，确保开机按键的尾部脉冲完全发送完毕
    uint32_t wait_quiet = millis();
    while (digitalRead(static_cast<int>(AppConfig::IR_RECV_PIN)) == LOW && millis() - wait_quiet < 150) {
        delay(10);
    }
    delay(50); // 静默沉淀 50ms
    
    // 2. 彻底清空接收器与历史码
    irrecv.resume();
    last_recv_code = 0;
}

bool ir_duplicate_error = false;

static bool matchCode(uint64_t recv, uint64_t target) {
    if (target == 0) return false;
    // 全 64 位精确匹配, 避免不同遥控器命令码相同造成误触发
    return recv == target;
}

static bool isDuplicateCode(uint64_t code, IRPairingState current) {
    if (current != IR_PAIRING_UP && matchCode(code, code_up)) return true;
    if (current != IR_PAIRING_DOWN && matchCode(code, code_down)) return true;
    if (current != IR_PAIRING_LEFT && matchCode(code, code_left)) return true;
    if (current != IR_PAIRING_RIGHT && matchCode(code, code_right)) return true;
    if (current != IR_PAIRING_ENTER && matchCode(code, code_enter)) return true;
    if (current != IR_PAIRING_ESC && matchCode(code, code_esc)) return true;
    if (current != IR_PAIRING_STANDBY && matchCode(code, code_standby)) return true;
    if (current != IR_PAIRING_POWEROFF && matchCode(code, code_poweroff)) return true;
    return false;
}

void processPairing(uint64_t code) {
    if (isDuplicateCode(code, pairing_state)) {
        ir_duplicate_error = true;
        beep(80, 3);
        CustomUiEngine::notifyUiNeedsUpdate();
        return;
    }
    ir_duplicate_error = false;

    int target_profile = (active_profile == 0) ? 1 : active_profile;
    const char* prefix = (target_profile == 2) ? "p2_" : "p1_";
    
    char key_name[16];
    switch (pairing_state) {
        case IR_PAIRING_UP:
            snprintf(key_name, sizeof(key_name), "%sup", prefix);
            code_up = code;
            prefs.putULong64(key_name, code);
            if (sequential_mode) pairing_state = IR_PAIRING_DOWN;
            else pairing_state = IR_PAIRING_DONE;
            break;
        case IR_PAIRING_DOWN:
            snprintf(key_name, sizeof(key_name), "%sdown", prefix);
            code_down = code;
            prefs.putULong64(key_name, code);
            if (sequential_mode) pairing_state = IR_PAIRING_LEFT;
            else pairing_state = IR_PAIRING_DONE;
            break;
        case IR_PAIRING_LEFT:
            snprintf(key_name, sizeof(key_name), "%sleft", prefix);
            code_left = code;
            prefs.putULong64(key_name, code);
            if (sequential_mode) pairing_state = IR_PAIRING_RIGHT;
            else pairing_state = IR_PAIRING_DONE;
            break;
        case IR_PAIRING_RIGHT:
            snprintf(key_name, sizeof(key_name), "%sright", prefix);
            code_right = code;
            prefs.putULong64(key_name, code);
            if (sequential_mode) pairing_state = IR_PAIRING_ENTER;
            else pairing_state = IR_PAIRING_DONE;
            break;
        case IR_PAIRING_ENTER:
            snprintf(key_name, sizeof(key_name), "%senter", prefix);
            code_enter = code;
            prefs.putULong64(key_name, code);
            if (sequential_mode) pairing_state = IR_PAIRING_ESC;
            else pairing_state = IR_PAIRING_DONE;
            break;
        case IR_PAIRING_ESC:
            snprintf(key_name, sizeof(key_name), "%sesc", prefix);
            code_esc = code;
            prefs.putULong64(key_name, code);
            if (sequential_mode) pairing_state = IR_PAIRING_STANDBY;
            else pairing_state = IR_PAIRING_DONE;
            break;
        case IR_PAIRING_STANDBY:
            snprintf(key_name, sizeof(key_name), "%sstandby", prefix);
            code_standby = code;
            prefs.putULong64(key_name, code);
            if (sequential_mode) pairing_state = IR_PAIRING_POWEROFF;
            else pairing_state = IR_PAIRING_DONE;
            break;
        case IR_PAIRING_POWEROFF:
            snprintf(key_name, sizeof(key_name), "%spoweroff", prefix);
            code_poweroff = code;
            prefs.putULong64(key_name, code);
            pairing_state = IR_PAIRING_DONE;
            break;
        default:
            return;
    }
    beep(50, 2);
    CustomUiEngine::notifyUiNeedsUpdate();
}

void loop() {
    // max_skip=4: 允许跳过帧前的少量噪声边沿，提高干净 NEC 帧的命中率。
    if (irrecv.decode(&results, nullptr, 4, 0)) {
        uint64_t code = results.value;
        // UNKNOWN 是库对所有协议都匹配失败后降级的 32 位哈希乱码（如 0x93DF5027），
        // 溢出帧同样不可信；对码学习时保留任意解码以兼容非标准遥控器。
        bool pairing_active = (pairing_state != IR_PAIRING_NONE && pairing_state != IR_PAIRING_DONE);
        bool is_garbage = (results.decode_type == UNKNOWN || results.overflow);
        if (code != 0xFFFFFFFF && code != 0 && (!is_garbage || pairing_active)) {
            last_recv_code = code;
            CustomUiEngine::notifyUiNeedsUpdate();
            
            if (pairing_active) {
                if (millis() - last_pair_time > 400) {
                    processPairing(code);
                    Serial.printf("[IR] Pairing code mapped: 0x%08X\n", (uint32_t)code);
                    last_pair_time = millis();
                }
            } else if (!ir_shielded) {
                unsigned long now = millis();
                if (matchCode(code, code_up)) { flag_up = true; last_up_time = now; }
                else if (matchCode(code, code_down)) { flag_down = true; last_down_time = now; }
                else if (matchCode(code, code_left)) { flag_left = true; last_left_time = now; }
                else if (matchCode(code, code_right)) { flag_right = true; last_right_time = now; }
                else if (matchCode(code, code_enter)) { flag_enter = true; last_enter_time = now; }
                else if (matchCode(code, code_esc)) { flag_esc = true; last_esc_time = now; }
                else if (matchCode(code, code_standby)) { flag_standby = true; last_standby_time = now; }
                else if (matchCode(code, code_poweroff)) { flag_poweroff = true; last_poweroff_time = now; }
            }
        } else if (is_garbage) {
            // Serial.printf("[IR] Discarded garbage frame: type=%d value=0x%08X overflow=%d\n",
            //              results.decode_type, (uint32_t)code, results.overflow);
        }
        irrecv.resume();
    }
    
    unsigned long now = millis();
    if (now - last_up_time > KEY_HOLD_TIME) flag_up = false;
    if (now - last_down_time > KEY_HOLD_TIME) flag_down = false;
    if (now - last_left_time > KEY_HOLD_TIME) flag_left = false;
    if (now - last_right_time > KEY_HOLD_TIME) flag_right = false;
    if (now - last_enter_time > KEY_HOLD_TIME) flag_enter = false;
    if (now - last_esc_time > KEY_HOLD_TIME) flag_esc = false;
    if (now - last_standby_time > KEY_HOLD_TIME) flag_standby = false;
    if (now - last_poweroff_time > KEY_HOLD_TIME) flag_poweroff = false;
}

bool isUp() { return !ir_shielded && !isPairingActive() && flag_up; }
bool isDown() { return !ir_shielded && !isPairingActive() && flag_down; }
bool isLeft() { return !ir_shielded && !isPairingActive() && flag_left; }
bool isRight() { return !ir_shielded && !isPairingActive() && flag_right; }
bool isEnter() { return !ir_shielded && !isPairingActive() && flag_enter; }
bool isEsc() { return !ir_shielded && !isPairingActive() && flag_esc; }
bool isStandby() {
    if (!ir_shielded && !isPairingActive() && flag_standby) {
        flag_standby = false; // 立即消费掉标志，防止重复读帧
        return true;
    }
    return false;
}
bool isPowerOff() {
    if (!ir_shielded && !isPairingActive() && flag_poweroff) {
        flag_poweroff = false;
        return true;
    }
    return false;
}

uint64_t getLastRecvCode() { return last_recv_code; }
void clearLastRecvCode() { last_recv_code = 0; }
uint64_t getCodeUp() { return code_up; }
uint64_t getCodeDown() { return code_down; }
uint64_t getCodeLeft() { return code_left; }
uint64_t getCodeRight() { return code_right; }
uint64_t getCodeEnter() { return code_enter; }
uint64_t getCodeEsc() { return code_esc; }
uint64_t getCodeStandby() { return code_standby; }
uint64_t getCodePowerOff() { return code_poweroff; }

#include <driver/rtc_io.h>
#include <driver/gpio.h>

void enterDeepSleep() {
    Serial.println("[SYSTEM] Preparing ESP32 Deep Sleep...");
    
    // 1. 彻底切断背光 + 清空液晶屏画幅 + 发送 ST7789 硬件睡眠指令 (SLPIN 0x10)
    CustomUiEngine::sleepDisplay();
    
    // 2. 硬件电平锁定：使用 gpio_hold_en 强制锁定背光引脚 (GPIO 9) 为 0V 低电平！
    // 避免进入 Deep Sleep 后 GPIO 高阻浮空导致背光电路导通变成白屏！
    gpio_num_t bl_pin = static_cast<gpio_num_t>(AppConfig::TFT_BACKLIGHT);
    gpio_set_direction(bl_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(bl_pin, 0);
    gpio_hold_en(bl_pin);
    gpio_deep_sleep_hold_en();
    delay(50);
    
    // 2. 必须等待用户手指彻底松开板载按键 (GPIO 18)！
    // 否则在按键处于 LOW 电平时进入 Deep Sleep，EXT1 唤醒源会瞬间检测到低电平，在 1ms 内重新唤醒造成死循环重启！
    uint32_t wait_start = millis();
    while (digitalRead(static_cast<int>(AppConfig::RETURN_BUTTON)) == LOW && millis() - wait_start < 3000) {
        delay(30);
    }
    delay(200); // 消除机械按键释防抖动

    // 2.5 同样必须等待红外接收脚恢复高电平, 否则 EXT0 低电平唤醒会立即再次触发
    uint32_t ir_wait_start = millis();
    while (digitalRead(static_cast<int>(AppConfig::IR_RECV_PIN)) == LOW && millis() - ir_wait_start < 1500) {
        delay(20);
    }
    delay(100);
    
    // 3. 启用 RTC 管脚内部上拉，防止红外接收脚 (GPIO 16) 和按键脚 (GPIO 18) 在休眠时悬空拉低引发虚假唤醒
    rtc_gpio_pullup_en(static_cast<gpio_num_t>(AppConfig::IR_RECV_PIN));
    rtc_gpio_pulldown_dis(static_cast<gpio_num_t>(AppConfig::IR_RECV_PIN));
    rtc_gpio_pullup_en(static_cast<gpio_num_t>(AppConfig::RETURN_BUTTON));
    rtc_gpio_pulldown_dis(static_cast<gpio_num_t>(AppConfig::RETURN_BUTTON));
    
    // 4. 配置唤醒引脚
    esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(AppConfig::IR_RECV_PIN), 0);
    uint64_t button_mask = (1ULL << static_cast<int>(AppConfig::RETURN_BUTTON));
    esp_sleep_enable_ext1_wakeup(button_mask, ESP_EXT1_WAKEUP_ANY_LOW);
    
    Serial.println("[SYSTEM] Entering Deep Sleep Now!");
    delay(100);
    esp_deep_sleep_start();
}

void startPairingKey(IRPairingState targetKey) {
    if (active_profile == 0) {
        setActiveProfile(1);
    }
    sequential_mode = false;
    ir_duplicate_error = false;
    pairing_state = targetKey;
    last_pair_time = millis();
    Serial.println("[IR] Single key pairing mode...");
}

void startSequentialPairing() {
    if (active_profile == 0) {
        setActiveProfile(1);
    }
    sequential_mode = true;
    ir_duplicate_error = false;
    pairing_state = IR_PAIRING_UP;
    last_pair_time = millis();
    Serial.println("[IR] Sequential pairing mode started...");
}

bool isSequentialPairing() {
    return sequential_mode;
}

IRPairingState getPairingState() {
    return pairing_state;
}

void cancelPairing() {
    pairing_state = IR_PAIRING_NONE;
    sequential_mode = false;
    ir_duplicate_error = false;
}

bool isPairingActive() {
    return pairing_state != IR_PAIRING_NONE && pairing_state != IR_PAIRING_DONE;
}

bool isDuplicateError() {
    return ir_duplicate_error;
}

void clearDuplicateError() {
    ir_duplicate_error = false;
}

void setShielding(bool shield) {
    ir_shielded = shield;
    if (!shield) {
        pairing_state = IR_PAIRING_NONE;
    }
}

bool isShielded() {
    return ir_shielded;
}

} // namespace
