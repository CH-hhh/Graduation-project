/**
 * ============================================================================
 * [ESP32-S3 Master Gateway GPIO 分配表]
 * ----------------------------------------------------------------------------
 * 【SPI屏幕 (FSPI/SPI3)】
 * MOSI = 14, MISO = 11, SCLK = 13, CS = 12, DC = 10, BL(背光) = 9
 * 
 * 【STM32 摄像头通信 SPI】* SPI2 Slave to receive Camera data from STM32H743
 * CS = 47, SCK = 38, MISO = 39, MOSI = 45, HANDSHAKE = 21 
 * 
 * 【I2S 功放与麦克风 (NS4168/INMP441)】
 * BCLK = 5, LRCK = 6, DOUT = 7 (功放 DIN), DIN = 4 (麦克风 DATA)
 * 
 * 【I2C 传感器 (ICM42688 姿态仪)】
 * SDA = 41, SCL = 40
 * 
 * 【外设交互】
 * 摇杆: VRX = 2, VRY = 1, SW = 42
 * 按钮: RETURN = 18
 * 蜂鸣器: 15
 * 接收器: IR = 16
 * 状态灯: (未配置)
 * ============================================================================
 */
#include <Arduino.h>
#include <memory>
#include <USB.h>
#include "tusb.h"
// No duplicate USBCDC object
#include <WiFi.h>
#include <esp_wifi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <TFT_eSPI.h>
#include <driver/i2s.h>
#include <esp_now.h>
#include <ArduinoJson.h> 
#include "mbedtls/base64.h"
#include <LittleFS.h>
#include <driver/rtc_io.h>
#include <driver/gpio.h>
#include <freertos/semphr.h> // 🌟 新增：引入 FreeRTOS 信号量互斥锁
#include "app_config.h"
#include "app_events.h"
#include "ai_chat_store.h"
#include "audio_service.h"
#include "cloud_protocol.h"
#include "chat_log_store.h"
#include "private_config.h"
#include "network_runtime.h"
#include "protocol_utils.h"
#include "imu_service.h"
#include "hardware_validation.h"
#include "rpc_protocol.h"
#include "custom_ui_engine.h"
#include "camera_service.h"
#include "ir_service.h"
#include "buzzer_service.h"
#include "music_player_service.h"
#include "device_action_service.h"
#include "voice_trigger_service.h"
#include "jpeg_encoder.h"
#include "debug_log_service.h"
#define Serial DebugLog

static bool s_music_paused_for_voice = false;

// 0: 竖屏(正向), 1: 横屏(右旋), 2: 竖屏(倒置), 3: 横屏(左旋)
#define DEFAULT_ROTATION AppConfig::DEFAULT_ROTATION
int current_rotation = DEFAULT_ROTATION; // 全局旋转角度

// ==========================================
// [网络与 API 配置]
// ==========================================
const char* API_KEY = PrivateConfig::DASHSCOPE_AUTHORIZATION;
const char* AMAP_KEY = PrivateConfig::AMAP_API_KEY;
const char* LOCATIONIQ_TOKEN = PrivateConfig::LOCATIONIQ_TOKEN;

// ✅ 改为阿里云公共网关
const char* LLM_HOST = AppConfig::DASHSCOPE_HOST;
const int LLM_PORT = AppConfig::DASHSCOPE_PORT;

// --- 引脚定义 ---
#define JOY_VRY static_cast<int>(AppConfig::JOY_VRY)
#define JOY_VRX static_cast<int>(AppConfig::JOY_VRX)
#define JOY_SW static_cast<int>(AppConfig::JOY_SW)
#define BUZZER_PIN static_cast<int>(AppConfig::BUZZER)
#define LED_PIN static_cast<int>(AppConfig::STATUS_LED)
#define TFT_BLK static_cast<int>(AppConfig::TFT_BACKLIGHT)

// 🌟 I2S 引脚映射 (共享总线)
#define AMP_EN static_cast<int>(AppConfig::AMP_ENABLE)

// ==========================================
// [ESP-NOW RPC 极简控制协议]
// ==========================================
uint8_t broadcast_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ==========================================
// [全局状态与缓冲]
// ==========================================
TFT_eSPI tft = TFT_eSPI();

uint8_t* psram_audio_buffer = NULL; 
uint32_t recorded_bytes = 0;        
const uint32_t MAX_RECORD_SIZE = AppConfig::MAX_RECORD_SIZE;

volatile bool isRecording = false;
volatile bool isCommunicating = false;
volatile bool isSpeaking = false; // TTS 播报中（仅用于 UI 按钮状态）
volatile bool global_interrupt = false; // 🌟 全局无缝打断标志
volatile uint8_t g_mic_diag_state = 0; // 0: IDLE, 1: RECORDING, 2: SENDING, 3: DONE
volatile float g_last_diag_rms = 0.0f;
volatile int32_t g_last_diag_dc = 0;
volatile float g_last_diag_dur = 0.0f;
SemaphoreHandle_t buzzer_mutex = nullptr;
SemaphoreHandle_t esp_now_mutex = nullptr;

// ==========================================
// [蜂鸣器音效引擎]
// ==========================================
void beep(int ms, int count) {
    if (!CustomUiEngine::getSystemSoundEnabled()) return;
    if (buzzer_mutex && xSemaphoreTake(buzzer_mutex, pdMS_TO_TICKS(500)) != pdTRUE) return;
    for (int i = 0; i < count; i++) {
        BuzzerService::tone(2500, ms); // 无源蜂鸣器需要频率驱动
        vTaskDelay(pdMS_TO_TICKS(ms));
        BuzzerService::stop();
        if (i < count - 1) vTaskDelay(pdMS_TO_TICKS(80));
    }
    if (buzzer_mutex) xSemaphoreGive(buzzer_mutex);
}

#define VAD_THRESHOLD 400         
#define MAX_SILENCE_FRAMES 50     
int silence_counter = 0;

volatile bool ack_received = false;
volatile bool ack_success = false;
volatile int smart_light_ack_state = -1; // 灯光回执: -1未知 0关 1开
volatile uint32_t last_light_ack_time = 0; // 最近一次灯光回执时间 (实时在线检测用)
int light_brightness = 100;                // 灯光亮度 0~100, <=20 视为关
volatile int global_light_state = 0;
volatile float ack_temperature_c = 0.0;
volatile float ack_pressure_hpa = 0.0;
volatile uint8_t environment_read_state = 0; // 0=未读取, 1=读取中, 2=成功, 3=失败
volatile uint32_t environment_read_revision = 0;
volatile bool environment_request_pending = false;
volatile uint32_t environment_request_started_at = 0;
volatile bool discovery_request_pending = false;
volatile bool discovery_ack_received = false;
volatile float ack_dht_temp = 0.0;
volatile float ack_dht_hum = 0.0;
volatile uint8_t temp_probe_read_state = 0;
volatile uint32_t temp_probe_read_revision = 0;
volatile bool temp_probe_request_pending = false;
volatile uint32_t temp_probe_request_started_at = 0;

volatile float ack_light_lux = 0.0;
volatile uint8_t light_read_state = 0;
volatile uint32_t light_read_revision = 0;
volatile bool light_request_pending = false;
volatile uint32_t light_request_started_at = 0;
volatile bool dev2_online = false;
volatile uint32_t ack_sensor_mask = 0;
// online_devices_list 统一使用 CustomUiEngine::online_devices_list
// struct WeatherForecast defined in custom_ui_engine.h
WeatherForecast forecasts[4] = {
    {"今天", "--/--", "--", 0, 0},
    {"明天", "--/--", "--", 0, 0},
    {"后天", "--/--", "--", 0, 0},
    {"大后天", "--/--", "--", 0, 0}
};
volatile bool weather_ready = false;

// 🌟 新增全局变量，保存高德查询到的城市代码
String current_adcode = ""; 
String current_city = "定位中...";



// ==========================================
// [全局缓存与状态管理]
// ==========================================
const size_t MAX_PCM_CACHE_SIZE = AppConfig::MAX_PCM_CACHE_SIZE;
const size_t MAX_RAW_CACHE_SIZE = AppConfig::MAX_RAW_CACHE_SIZE;

uint8_t* pcm_cache = NULL;
char* raw_cache = NULL;
volatile size_t pcm_cache_len = 0;
volatile size_t raw_cache_len = 0;

SemaphoreHandle_t cache_mutex; // 互斥锁，用于隔离网络写入与网页读取
SemaphoreHandle_t state_mutex; // 保护跨核共享的城市、天气和设备列表
volatile bool trigger_pcm_playback = false; // 触发 Web 导入的 PCM 播放标志位

TaskHandle_t network_task_handle = nullptr;
TaskHandle_t gui_task_handle = nullptr;
volatile bool app_tasks_started = false;
volatile bool fatal_hardware_error = false;

class CommunicationGuard {
public:
    CommunicationGuard() : owner_(!isCommunicating) {
        if (owner_) {
            isCommunicating = true;
            CustomUiEngine::notifyUiNeedsUpdate(); // 状态切换立即刷新 UI，避免按钮文字滞留
        }
    }
    ~CommunicationGuard() {
        if (owner_) {
            isCommunicating = false;
            CustomUiEngine::notifyUiNeedsUpdate();
        }
    }
private:
    bool owner_;
};

class TtsSpeakingGuard {
public:
    TtsSpeakingGuard() {
        isSpeaking = true;
        CustomUiEngine::notifyUiNeedsUpdate();
    }
    ~TtsSpeakingGuard() {
        isSpeaking = false;
        CustomUiEngine::notifyUiNeedsUpdate();
    }
};

String currentTitle = "初始化中";
String currentText = "";

void resetHardwarePostTurn() {
    // 1. 全量清空底层 PSRAM 与 SRAM 数据缓冲区
    xSemaphoreTake(cache_mutex, portMAX_DELAY);
    pcm_cache_len = 0;
    raw_cache_len = 0;
    if (pcm_cache) memset(pcm_cache, 0, 1024);
    if (raw_cache) memset(raw_cache, 0, 1024);
    xSemaphoreGive(cache_mutex);

    // 2. 清空麦克风录音缓冲区与 VAD 计数器
    recorded_bytes = 0;
    silence_counter = 0;
    if (psram_audio_buffer) memset(psram_audio_buffer, 0, 4096);

    // 3. 硬件 I2S 复位并清空 DMA 缓冲
    AudioService::initForTTS(AppConfig::AUDIO_SAMPLE_RATE);
    AudioService::wake();
    i2s_zero_dma_buffer(I2S_NUM_0);
}

void triggerGlobalInterrupt() {
    Serial.println("[SYS] 强行打断清空");
    global_interrupt = true;

    resetHardwarePostTurn();

    isCommunicating = false;
    isSpeaking = false;
    isRecording = false;

    beep(30, 2);
}

// ==========================================
// [硬件控制与 I2S 全双工引擎]
// ==========================================
// ==========================================
// [ICM42688 传感器 I2C 读取逻辑]
// ==========================================
// ==========================================
// [Flash 存储：持久化日志管理]
// ==========================================
// ==========================================
// [UI 渲染逻辑 (Core 1)]
// ==========================================


void applyScreenMessage(const String& title, const String& content) {
    appendChatLog(title, content);
}

void showOnScreenSafe(const String& title, const String& content) {
    if (app_tasks_started && xTaskGetCurrentTaskHandle() != gui_task_handle) {
        if (!postUiMessage(title, content)) {
            Serial.println("[WARN] UI 消息队列已满，丢弃一条界面消息");
        }
        return;
    }
    applyScreenMessage(title, content);
}

// 请求-响应期间: 主机射频临时保持常开, 确保回包一次命中
uint32_t radio_awake_until = 0;

void sendRpcCommand(uint8_t dev_id, uint8_t action, uint8_t val, uint8_t retries = 0) {
    RpcCommand cmd = {dev_id, action, val, 0, 0, 0, 0};
    // 指令通常需要等待对端回包: 接下来 2 秒主机不休眠, 回包不会漏收
    radio_awake_until = millis() + 2000;
    // 重发次数: 显式指定则用之; 默认 15 次 (300ms 窗口最坏也能撞上从机常开窗口)
    if (retries == 0) retries = 15;
    for (int i = 0; i < retries; i++) {
        if (esp_now_mutex && xSemaphoreTake(esp_now_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            esp_now_send(broadcast_mac, (uint8_t *) &cmd, sizeof(cmd));
            xSemaphoreGive(esp_now_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}

// 发送灯光亮度 (0~100, <=20 从机视为关灯); 同时更新主机显示状态
void sendLightBrightness(int v) {
    if (v < 20) v = 20; // 起始点 20%: 20%=关, 再往上立即点亮
    if (v > 100) v = 100;
    light_brightness = v;
    global_light_state = (v > 20) ? 1 : 0;
    sendRpcCommand(2, 0x01, (uint8_t)v);
}

// 灯光亮度实时快发 (进入亮度模式后两端已关闭休眠, 2 包无间隔连发即可)
void sendLightBrightnessFast(int v) {
    if (v < 20) v = 20; // 起始点 20%: 20%=关, 再往上立即点亮
    if (v > 100) v = 100;
    light_brightness = v;
    global_light_state = (v > 20) ? 1 : 0;
    radio_awake_until = millis() + 2000;
    RpcCommand cmd = {2, 0x01, (uint8_t)v, 0, 0, 0, 0};
    for (int i = 0; i < 2; i++) {
        if (esp_now_mutex && xSemaphoreTake(esp_now_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            esp_now_send(broadcast_mac, (uint8_t *)&cmd, sizeof(cmd));
            xSemaphoreGive(esp_now_mutex);
        }
    }
}

// 主机是否需要临时保持射频常开 (正在等待回包)
bool isRadioAwakeRequested() {
    return millis() < radio_awake_until ||
           environment_request_pending ||
           temp_probe_request_pending ||
           light_request_pending;
}

bool consumeSuccessfulHttpHeaders(WiFiClient& client) {
    int statusCode = 0;
    String statusLine = "";
    unsigned long headerDeadline = millis() + 15000;
    while (client.connected() && millis() < headerDeadline) {
        statusLine = client.readStringUntil('\n');
        statusLine.trim();
        if (statusLine.length() == 0) continue;
        
        const int firstSpace = statusLine.indexOf(' ');
        statusCode = firstSpace >= 0 ? statusLine.substring(firstSpace + 1, firstSpace + 4).toInt() : 0;
        
        // 读取并丢弃当前状态的 header
        while (client.connected() && millis() < headerDeadline) {
            String header = client.readStringUntil('\n');
            if (header == "\r" || header.length() == 0) break;
        }
        
        if (statusCode >= 100 && statusCode < 200) {
            // 忽略 1xx 信息响应 (如 100 Continue)，继续读取最终响应
            continue;
        }
        break;
    }
    
    if (statusCode == 0) {
        Serial.println("[ERR] 云端响应头等待超时(15秒)");
        return false;
    }
    if (statusCode < 200 || statusCode >= 300) {
        Serial.println("[ERR] 云端 HTTP 状态异常: " + statusLine);
        return false;
    }
    return true;
}

void OnAckRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    if (len == sizeof(RpcAcknowledgement)) {
        RpcAcknowledgement ack;
        memcpy(&ack, incomingData, sizeof(ack));
        if (ack.device_id == 2) { // 确认是设备2的数据
            // 回包去重: 同一类型 500ms 内只处理第一份完整数据, 重复包直接丢弃,
            // 避免重复打印日志/重复刷新 UI/重复自增修订号
            static uint32_t last_ack_time = 0;
            static uint8_t last_ack_type = 0;
            uint32_t ack_now = millis();
            if (ack.is_success == last_ack_type && ack_now - last_ack_time < 500) return;
            last_ack_time = ack_now;
            last_ack_type = ack.is_success;

            ack_received = true;   // 收到包了！
            ack_success = (ack.is_success == RpcProtocol::STATUS_SUCCESS);
            if (ack.is_success == RpcProtocol::STATUS_ENVIRONMENT) {
                ack_temperature_c = ack.sensor_value;
                ack_pressure_hpa = ack.sensor_value2;
                environment_read_state = 2;
                ++environment_read_revision;
                environment_request_pending = false;
                CustomUiEngine::notifyUiNeedsUpdate();
                Serial.printf("[ESP-NOW] GY-63 温度 %.1f C，气压 %.1f hPa\r\n",
                              ack_temperature_c, ack_pressure_hpa);
            } else if (ack.is_success == RpcProtocol::STATUS_TEMPERATURE_PROBE) {
                ack_dht_temp = ack.sensor_value;
                ack_dht_hum = ack.sensor_value2;
                temp_probe_read_state = 2;
                ++temp_probe_read_revision;
                ++environment_read_revision; // Force homepage UI to refresh humidity
                temp_probe_request_pending = false;
                CustomUiEngine::notifyUiNeedsUpdate();
                Serial.printf("[ESP-NOW] DHT 温湿度 T:%.1f C, H:%.1f %%\r\n", ack_dht_temp, ack_dht_hum);
            } else if (ack.is_success == RpcProtocol::STATUS_AMBIENT_LIGHT) {
                ack_light_lux = ack.sensor_value;
                light_read_state = 2;
                ++light_read_revision;
                light_request_pending = false;
                CustomUiEngine::notifyUiNeedsUpdate();
                Serial.printf("[ESP-NOW] VEML7700 光照: %.1f Lux\r\n", ack_light_lux);
            } else if (ack.is_success == RpcProtocol::STATUS_SUCCESS) {
                // 灯光回执: 从机确认已按指令动作, 实际状态 = 最近一次指令值
                smart_light_ack_state = global_light_state ? 1 : 0;
                last_light_ack_time = millis();
                CustomUiEngine::notifyUiNeedsUpdate();
                Serial.printf("[ESP-NOW] 灯光回执: %s\r\n", smart_light_ack_state ? "开" : "关");
            } else if (ack.is_success == RpcProtocol::STATUS_FAILURE &&
                       environment_request_pending) {
                environment_read_state = 3;
                ++environment_read_revision;
                environment_request_pending = false;
            } else if (ack.is_success == RpcProtocol::STATUS_FAILURE &&
                       temp_probe_request_pending) {
                temp_probe_read_state = 3;
                ++temp_probe_read_revision;
                temp_probe_request_pending = false;
            } else if (ack.is_success == RpcProtocol::STATUS_FAILURE &&
                       light_request_pending) {
                light_read_state = 3;
                ++light_read_revision;
                light_request_pending = false;
            } else if (ack.is_success == RpcProtocol::STATUS_FAILURE &&
                       discovery_request_pending) {
                // 设备发现应答: 从机用 status=0 + sensor_value=传感器掩码 回应
                discovery_ack_received = true;
                ack_sensor_mask = static_cast<uint32_t>(ack.sensor_value);
                Serial.printf("[ESP-NOW] 设备发现应答, 传感器掩码: 0x%02X\r\n",
                              static_cast<unsigned>(ack_sensor_mask));
            }
            
            // 🌟 收到回包后标记设备及传感器掩码
            dev2_online = true; 
        }
    }
}

bool startEnvironmentSensorQuery() {
    if (environment_request_pending) return false;
    ack_received = false;
    ack_success = false;
    environment_read_state = 1;
    ++environment_read_revision;
    environment_request_pending = true;
    environment_request_started_at = millis();
    sendRpcCommand(RpcProtocol::DEVICE_SMART_SOCKET,
                   RpcProtocol::ACTION_ENVIRONMENT, 0);
    return true;
}

void serviceEnvironmentSensorTimeout(uint32_t timeoutMs = AppConfig::ENVIRONMENT_QUERY_TIMEOUT_MS) {
    if (environment_request_pending &&
        millis() - environment_request_started_at >= timeoutMs) {
        environment_request_pending = false;
        environment_read_state = 3;
        ++environment_read_revision;
    }
}

bool queryEnvironmentSensor(uint32_t timeoutMs = AppConfig::ENVIRONMENT_QUERY_TIMEOUT_MS) {
    if (!environment_request_pending) startEnvironmentSensorQuery();
    const uint32_t startedAt = millis();
    while (millis() - startedAt < timeoutMs) {
        if (!environment_request_pending) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    serviceEnvironmentSensorTimeout(timeoutMs);
    return environment_read_state == 2;
}

String environmentReadingText() {
    return "温度 " + String(ack_temperature_c, 1) + " 摄氏度；气压 " +
           String(ack_pressure_hpa, 1) + " 百帕";
}

bool startTempProbeQuery() {
    if (temp_probe_request_pending) return false;
    ack_received = false;
    ack_success = false;
    temp_probe_read_state = 1;
    ++temp_probe_read_revision;
    temp_probe_request_pending = true;
    temp_probe_request_started_at = millis();
    sendRpcCommand(RpcProtocol::DEVICE_SMART_SOCKET,
                   RpcProtocol::ACTION_TEMPERATURE_PROBE, 0);
    return true;
}

void serviceTempProbeTimeout(uint32_t timeoutMs = AppConfig::ENVIRONMENT_QUERY_TIMEOUT_MS) {
    if (temp_probe_request_pending &&
        millis() - temp_probe_request_started_at >= timeoutMs) {
        temp_probe_request_pending = false;
        temp_probe_read_state = 3;
        ++temp_probe_read_revision;
    }
}

bool queryTempProbeSensor(uint32_t timeoutMs = AppConfig::ENVIRONMENT_QUERY_TIMEOUT_MS) {
    if (!temp_probe_request_pending) startTempProbeQuery();
    const uint32_t startedAt = millis();
    while (millis() - startedAt < timeoutMs) {
        if (!temp_probe_request_pending) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    serviceTempProbeTimeout(timeoutMs);
    return temp_probe_read_state == 2;
}

String tempProbeReadingText() {
    return "T:" + String(ack_dht_temp, 1) + "C H:" + String(ack_dht_hum, 0) + "%";
}

bool startAmbientLightQuery() {
    if (light_request_pending) return false;
    ack_received = false;
    ack_success = false;
    light_read_state = 1;
    ++light_read_revision;
    light_request_pending = true;
    light_request_started_at = millis();
    sendRpcCommand(RpcProtocol::DEVICE_SMART_SOCKET, RpcProtocol::ACTION_AMBIENT_LIGHT, 0);
    return true;
}

void serviceAmbientLightTimeout(uint32_t timeoutMs = AppConfig::ENVIRONMENT_QUERY_TIMEOUT_MS) {
    if (light_request_pending && millis() - light_request_started_at >= timeoutMs) {
        light_request_pending = false;
        light_read_state = 3;
        ++light_read_revision;
    }
}

bool queryAmbientLightSensor(uint32_t timeoutMs = AppConfig::ENVIRONMENT_QUERY_TIMEOUT_MS) {
    if (!light_request_pending) startAmbientLightQuery();
    const uint32_t startedAt = millis();
    while (millis() - startedAt < timeoutMs) {
        if (!light_request_pending) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    serviceAmbientLightTimeout(timeoutMs);
    return light_read_state == 2;
}

String ambientLightReadingText() {
    return "Lux: " + String(ack_light_lux, 1);
}

// 🌟 修复：给大模型真实的模拟数据
void skillScanDevices() { 
    ack_received = false;
    dev2_online = false; 
    discovery_ack_received = false;
    discovery_request_pending = true;
    
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    CustomUiEngine::online_devices_list = "[系统] 正在探测 ESP-NOW...\n  - 探针: RpcDiscover\n  - 状态: 扫描中...";
    xSemaphoreGive(state_mutex);
    CustomUiEngine::notifyDeviceContentUpdate();

    // 与其他 skill 完全一致：先连发 15 次，再等回包
    sendRpcCommand(RpcProtocol::DEVICE_SMART_SOCKET, RpcProtocol::ACTION_DISCOVER, 0);
    
    unsigned long wait_start = millis();
    while (millis() - wait_start < 2000) {
        if (discovery_ack_received) {
            Serial.println("[ESP-NOW] 成功收到被控端回包！");
            break; 
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    discovery_request_pending = false;
    
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    
    // 1. 主控端 S3 自检
    bool psram_ok = psramFound() && (ESP.getPsramSize() > 0);
    bool fs_ok = (LittleFS.totalBytes() > 0);
    int16_t dummy_ax = 0, dummy_ay = 0, dummy_az = 0;
    
    String wifi_str = "未连";
    String ip_str = "";
    if (WiFi.status() == WL_CONNECTED) {
        wifi_str = "已连";
        ip_str = "  - IP地址: " + WiFi.localIP().toString() + "\n";
    }
    
    bool imu_ok = false;
    for (int i = 0; i < 3; i++) {
        if (ImuService::readAcceleration(dummy_ax, dummy_ay, dummy_az)) {
            imu_ok = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    CustomUiEngine::online_devices_list = "[主控 S3] 状态:在线 | WiFi:" + wifi_str + "\n";
    if (ip_str.length() > 0) CustomUiEngine::online_devices_list += ip_str;
    CustomUiEngine::online_devices_list += "  - 屏幕:OK | PSRAM:" + String(psram_ok ? "8M" : "无") + " | FS:" + String(fs_ok ? "4M" : "无") + "\n";
    CustomUiEngine::online_devices_list += "  - ICM42688姿态仪: " + String(imu_ok ? "正常" : "异常") + "\n";

    // 2. 边缘节点 A 自检
    if (dev2_online || discovery_ack_received) {
        bool led_online  = (ack_sensor_mask & (1 << 0)) != 0;
        bool gy63_online = (ack_sensor_mask & (1 << 2)) != 0;
        bool dht_online  = (ack_sensor_mask & (1 << 3)) != 0;
        bool oled_online = (ack_sensor_mask & (1 << 4)) != 0;

        CustomUiEngine::online_devices_list += "[节点 A] 通信:在线 | 指示灯:" + String(led_online ? "OK" : "断") + "\n";
        CustomUiEngine::online_devices_list += "  - OLED屏:" + String(oled_online ? "OK" : "断") + "\n";
        CustomUiEngine::online_devices_list += "  - GY63气压:" + String(gy63_online ? "OK" : "断") + " | DHT温湿度:" + String(dht_online ? "OK" : "断");
    } else {
        CustomUiEngine::online_devices_list += "[节点 A] 通信:离线 (未响应)\n";
        CustomUiEngine::online_devices_list += "  - OLED屏:未知\n";
        CustomUiEngine::online_devices_list += "  - GY63气压:未知 | DHT温湿度:未知";
        Serial.println("ESP-NOW 超时");
    }
    xSemaphoreGive(state_mutex);

    CustomUiEngine::notifyDeviceContentUpdate();
}

static String urlEncode(const String& str) {
    String encoded = "";
    const char* hex = "0123456789ABCDEF";
    for (size_t i = 0; i < str.length(); i++) {
        uint8_t c = (uint8_t)str[i];
        if (('a' <= c && c <= 'z') || ('A' <= c && c <= 'Z') || ('0' <= c && c <= '9') || c == '-' || c == '_' || c == '.' || c == '~') {
            encoded += (char)c;
        } else {
            encoded += '%';
            encoded += hex[(c >> 4) & 0xF];
            encoded += hex[c & 0xF];
        }
    }
    return encoded;
}

// 🌟 双引擎 IP 城市高精度定位 (优先 IPIP.net 精准识别蜂窝出口如武汉, 备用高德 IP 接口)
String skillIPLocation() {
    if (WiFi.status() != WL_CONNECTED) return "网络未连接";
    
    // 1. 优先通过中国公认高精度 IPIP.net 接口查询 (纯 HTTP 超快、免 Key、免企业认证、准确识别手机热点出口)
    Serial.println("[APP] 正在通过 IPIP.net 接口获取高精度城市定位...");
    HTTPClient http;
    http.setConnectTimeout(AppConfig::HTTP_CONNECT_TIMEOUT_MS);
    http.setTimeout(AppConfig::HTTP_READ_TIMEOUT_MS);
    http.begin("http://myip.ipip.net/json");
    http.setUserAgent("curl/7.68.0");
    int httpCode = http.GET();
    
    if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, payload);
        if (!err && doc["ret"] == "ok") {
            String province = doc["data"]["location"][1].as<String>();
            String city = doc["data"]["location"][2].as<String>();
            String pubIp = doc["data"]["ip"].as<String>();
            if (city.length() == 0 || city == "null") {
                city = (province.length() > 0 && province != "null") ? province : "未知城市";
            }
            
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            current_city = city;
            current_adcode = ""; // 稍后在天气接口由高德自动补全 adcode
            String cityCopy = current_city;
            xSemaphoreGive(state_mutex);
            
            Serial.printf("[APP] IPIP.net 定位成功: %s (%s, IP: %s)\r\n", cityCopy.c_str(), province.c_str(), pubIp.c_str());
            http.end();
            return cityCopy;
        }
    }
    http.end();

    // 2. 备选方案：高德原生 IP 接口
    Serial.println("[APP] IPIP.net 未响应，切换高德原生 IP 接口兜底...");
    WiFiClientSecure secureClient;
    secureClient.setInsecure();
    http.begin(secureClient, "https://restapi.amap.com/v3/ip?key=" + String(AMAP_KEY));
    httpCode = http.GET();
    String result = "获取失败";
    
    if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, payload);
        if (!err && doc["status"] == "1") {
            String city = doc["city"].as<String>();
            String province = doc["province"].as<String>();
            if (city == "[]" || city.length() == 0) {
                city = (province == "[]" || province.length() == 0) ? "未知城市" : province;
            }
            String adcode = doc["adcode"].as<String>();
            if (adcode == "[]") adcode = "";

            xSemaphoreTake(state_mutex, portMAX_DELAY);
            current_city = city;
            current_adcode = adcode;
            String cityCopy = current_city;
            String adcodeCopy = current_adcode;
            xSemaphoreGive(state_mutex);

            result = cityCopy;
            Serial.println("[APP] 高德定位成功: " + cityCopy + " (Adcode: " + adcodeCopy + ")");
        } else {
            Serial.printf("[APP] 高德定位接口解析失败: %s\r\n", payload.c_str());
        }
    } else {
        Serial.printf("[APP] 高德定位 HTTP 请求失败, Code=%d\r\n", httpCode);
    }
    http.end();
    return result;
}

void skillWeather();

// 🌟 全量刷新网络关联信息 (NTP时间、公网IP/城市定位、天气预报、屏幕重绘)
void refreshNetworkInfo(bool forceClockSync) {
    if (WiFi.status() != WL_CONNECTED) return;
    Serial.println("[NET] 正在全量刷新网络时钟、定位与天气信息...");
    if (forceClockSync) {
        NetworkRuntime::synchronizeClock(8000);
    }
    
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    current_city = "定位中...";
    current_adcode = "";
    xSemaphoreGive(state_mutex);
    CustomUiEngine::notifyUiNeedsUpdate();

    String locRes = skillIPLocation();
    if (locRes != "获取失败" && locRes != "网络未连接") {
        skillWeather();
    } else {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        if (current_city == "定位中...") {
            current_city = "未知城市";
        }
        xSemaphoreGive(state_mutex);
    }
    CustomUiEngine::notifyUiNeedsUpdate();
}

void skillWeather() {
    if (WiFi.status() != WL_CONNECTED) return;
    
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    String targetCityParam = (current_adcode.length() > 0) ? current_adcode : urlEncode(current_city);
    xSemaphoreGive(state_mutex);
    if (targetCityParam.length() == 0 || targetCityParam == "%E5%AE%9A%E4%BD%8D%E4%B8%AD...") return;
    
    WiFiClientSecure secureClient;
    secureClient.setInsecure();
    HTTPClient http;
    http.setConnectTimeout(AppConfig::HTTP_CONNECT_TIMEOUT_MS);
    http.setTimeout(AppConfig::HTTP_READ_TIMEOUT_MS);
    http.begin(secureClient, "https://restapi.amap.com/v3/weather/weatherInfo?city=" + targetCityParam + "&extensions=all&key=" + String(AMAP_KEY));
    int httpCode = http.GET();
    
    if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        JsonDocument doc;
        deserializeJson(doc, payload);
        if (doc["status"] == "1" && doc["forecasts"].size() > 0) {
            String respAdcode = doc["forecasts"][0]["adcode"].as<String>();
            String respCity = doc["forecasts"][0]["city"].as<String>();
            JsonArray casts = doc["forecasts"][0]["casts"];
            int count = casts.size() > 4 ? 4 : casts.size();
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            if (respAdcode.length() > 0 && respAdcode != "[]") current_adcode = respAdcode;
            if (respCity.length() > 0 && respCity != "[]" && (current_city == "未知城市" || current_city.length() == 0)) {
                current_city = respCity;
            }
            for(int i = 0; i < count; i++) {
                String d = casts[i]["date"].as<String>();
                if(d.length() >= 10) {
                    forecasts[i].date = d.substring(5);
                } else {
                    forecasts[i].date = d;
                }
                
                String w = casts[i]["week"].as<String>();
                if(i == 0) forecasts[i].week = "今天";
                else if(i == 1) forecasts[i].week = "明天";
                else {
                    if(w == "1") forecasts[i].week = "周一";
                    else if(w == "2") forecasts[i].week = "周二";
                    else if(w == "3") forecasts[i].week = "周三";
                    else if(w == "4") forecasts[i].week = "周四";
                    else if(w == "5") forecasts[i].week = "周五";
                    else if(w == "6") forecasts[i].week = "周六";
                    else if(w == "7") forecasts[i].week = "周日";
                }
                
                forecasts[i].weather = casts[i]["dayweather"].as<String>();
                forecasts[i].high_temp = casts[i]["daytemp"].as<int>();
                forecasts[i].low_temp = casts[i]["nighttemp"].as<int>();
            }
            weather_ready = true;
            xSemaphoreGive(state_mutex);
            CustomUiEngine::notifyUiNeedsUpdate();
        }
    } else {
        Serial.println("[ERR] 天气获取失败");
    }
    http.end();
}

// 🌟 修复一：真实高德天气查询
// ==========================================
// [大模型交互：第二轮全模态 流式 SSE 版 — 3MB缓冲 + 流式=号剔除 + 防溢出]
// ==========================================
// [TTS 语音合成引擎] — 文本 → DashScope TTS → I2S 播放
// ==========================================
// 按字符数截断文本，并保证不会切在 UTF-8 多字节字符中间
String limitUtf8Chars(const String& text, size_t maxChars) {
    size_t count = 0;
    size_t pos = 0;
    const size_t len = text.length();
    while (pos < len && count < maxChars) {
        const uint8_t c = (uint8_t)text[pos];
        size_t step = 1;
        if ((c & 0xE0) == 0xC0) step = 2;
        else if ((c & 0xF0) == 0xE0) step = 3;
        else if ((c & 0xF8) == 0xF0) step = 4;
        if (pos + step > len) break;
        pos += step;
        ++count;
    }
    return pos >= len ? text : text.substring(0, pos);
}

void playTTS(String text) {
    if (text.length() == 0) return;
    // 阿里云 TTS 单次上限 600 字符，这里保险限制在 500 字，避免超长回答直接报错不播报
    text = limitUtf8Chars(text, 500);
    if (!NetworkRuntime::cloudReady()) {
        showOnScreenSafe("播报失败", "WiFi 或系统时间未就绪");
        return;
    }
    // 🌟 若音乐播放器正在运行或处于点歌就绪态，先暂停音乐播放以独占功放硬件，并标记挂起状态
    if (MusicPlayerService::isPlaying() || MusicPlayerService::isPaused()) {
        s_music_paused_for_voice = true;
        MusicPlayerService::sendPause();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    VoiceTriggerService::setCooldown(15000); // 播报期间强制屏蔽麦克风自激唤醒
    TtsSpeakingGuard speakingGuard; // 播报期间按钮显示“正在播报...”
    showOnScreenSafe("播报", "正在请求语音合成...");

    WiFiClientSecure client;
    client.setInsecure();
    client.setTimeout(AppConfig::HTTP_READ_TIMEOUT_MS);

    HTTPClient http;
    http.begin(client, "https://dashscope.aliyuncs.com/api/v1/services/aigc/multimodal-generation/generation");
    http.setConnectTimeout(AppConfig::HTTP_CONNECT_TIMEOUT_MS);
    http.setTimeout(AppConfig::HTTP_READ_TIMEOUT_MS);
    http.addHeader("Authorization", String(API_KEY));
    http.addHeader("Content-Type", "application/json");

    JsonDocument doc;
    doc["model"] = "qwen3-tts-flash";
    doc["input"]["text"] = text;
    doc["parameters"]["voice"] = "Cherry";
    doc["parameters"]["format"] = "wav";
    doc["parameters"]["sample_rate"] = 24000; // 🌟 阿里 DashScope 原生支持 24000Hz (配合立体声DMA完美原速播放)

    String payload;
    serializeJson(doc, payload);

    int httpCode = http.POST(payload);
    if (httpCode == 200 || httpCode == HTTP_CODE_OK) {
        String responseStr = http.getString();
        JsonDocument respDoc;
        DeserializationError err = deserializeJson(respDoc, responseStr);
        if (!err) {
            if (respDoc["code"].is<const char*>()) {
                Serial.println("ERR: TTS报错 " + respDoc["code"].as<String>());
                http.end();
                return;
            }
            bool played = false;
            String audio_url = respDoc["output"]["audio"]["url"] | "";
            if (audio_url.length() > 10) {

                // 先释放TTS接口连接，避免两套TLS上下文同时占用内部SRAM。
                http.end();
                client.stop();
                vTaskDelay(pdMS_TO_TICKS(10));

                HTTPClient audioHttp;
                audioHttp.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
                audioHttp.setConnectTimeout(AppConfig::HTTP_CONNECT_TIMEOUT_MS);
                audioHttp.setTimeout(AppConfig::HTTP_READ_TIMEOUT_MS);
                WiFiClient plainClient;
                WiFiClientSecure secureClient;
                bool audioBeginOk = false;
                if (audio_url.startsWith("http://")) {
                    audioBeginOk = audioHttp.begin(plainClient, audio_url);
                } else if (audio_url.startsWith("https://")) {
                    secureClient.setInsecure();
                    audioBeginOk = audioHttp.begin(secureClient, audio_url);
                } else {
                    Serial.println("ERR: 格式不支持");
                    return;
                }
                const int audioCode = audioBeginOk ? audioHttp.GET() : HTTPC_ERROR_CONNECTION_REFUSED;
                if (audioCode == 200 || audioCode == HTTP_CODE_OK) {
                    WiFiClient* stream = audioHttp.getStreamPtr();
                    int totalLen = audioHttp.getSize();

                    // 🌟 1. 利用 ESP32-S3 的 2.5MB PSRAM 缓存极速下载完整 WAV 音频 (彻底杜绝边下载边推流的网络抖动与欠载波波声)
                    xSemaphoreTake(cache_mutex, portMAX_DELAY);
                    pcm_cache_len = 0;

                    uint8_t temp_buf[2048];
                    unsigned long last_read_ms = millis();
                    while (audioHttp.connected() && (totalLen > 0 ? ((int)pcm_cache_len < totalLen) : true)) {
                        size_t avail = stream->available();
                        if (avail > 0) {
                            last_read_ms = millis();
                            size_t to_read = (avail > sizeof(temp_buf)) ? sizeof(temp_buf) : avail;
                            int r = stream->read(temp_buf, to_read);
                            if (r > 0) {
                                if (pcm_cache_len + r < MAX_PCM_CACHE_SIZE) {
                                    memcpy(pcm_cache + pcm_cache_len, temp_buf, r);
                                    pcm_cache_len += r;
                                }
                            }
                        } else {
                            if (totalLen > 0 && (int)pcm_cache_len >= totalLen) break;
                            if (millis() - last_read_ms > 10000) break;
                            vTaskDelay(pdMS_TO_TICKS(5));
                        }
                    }
                    audioHttp.end(); // 🌟 极速收完立即释放网络连接与 TLS 资源

                    if (pcm_cache_len > 44 && pcm_cache[0] == 'R' && pcm_cache[1] == 'I' && pcm_cache[2] == 'F' && pcm_cache[3] == 'F') {
                        // 🌟 2. 动态精准解析 WAV 头部：提取真实采样率与声道数 (彻底消除 8.8% 加速感与音调上浮)
                        uint16_t num_channels = *(uint16_t*)(pcm_cache + 22);
                        uint32_t wav_sample_rate = *(uint32_t*)(pcm_cache + 24);
                        uint16_t bits_per_sample = *(uint16_t*)(pcm_cache + 34);

                        if (wav_sample_rate < 8000 || wav_sample_rate > 96000) {
                            wav_sample_rate = 24000;
                        }

                        // 🌟 精确定位 "data" 块偏移量，彻底杜绝任意长度元数据导致的字节错位与破音
                        size_t data_offset = 0;
                        for (size_t z = 12; z < pcm_cache_len - 4 && z < 256; z++) {
                            if (pcm_cache[z] == 'd' && pcm_cache[z+1] == 'a' && pcm_cache[z+2] == 't' && pcm_cache[z+3] == 'a') {
                                data_offset = z + 8;
                                break;
                            }
                        }
                        if (data_offset == 0 || data_offset >= pcm_cache_len) {
                            data_offset = 44;
                        }

                        Serial.printf("🎯 [TTS-AUDIO] 动态锁定真实音频: 采样率=%u Hz, 声道数=%d, 位宽=%d bit, 数据大小=%u B\r\n",
                                      (unsigned)wav_sample_rate, num_channels, bits_per_sample, (unsigned)(pcm_cache_len - data_offset));

                        // 🌟 3. 动态配置 I2S 硬件锁相环时钟，与音频原生采样率 100% 严密对齐
                        AudioService::initForTTS(wav_sample_rate);
                        AudioService::wake(); // 唤醒 I2S 驱动并锁定时钟

                        // 🌟 4. 硬件防吞字静音前缀 (200ms)
                        const size_t PRE_SILENCE_BYTES = 4096;
                        uint8_t* pre_silence_buf = (uint8_t*)calloc(PRE_SILENCE_BYTES, 1);
                        if (pre_silence_buf) {
                            size_t bw = 0;
                            i2s_write(I2S_NUM_0, pre_silence_buf, PRE_SILENCE_BYTES, &bw, portMAX_DELAY);
                            free(pre_silence_buf);
                        }

                        // 🌟 5. 从 PSRAM 连续无缝推流到 I2S DMA (零延迟、零网络断流、零波波声)
                        uint8_t* pcm_data = pcm_cache + data_offset;
                        size_t pcm_bytes = pcm_cache_len - data_offset;
                        int vol = CustomUiEngine::getTtsVolume();
                        float factor = (vol / 100.0f) * 0.80f;

                        int16_t stereo_chunk[1024]; // 512 stereo frames

                        if (num_channels == 1) {
                            int16_t* mono_ptr = (int16_t*)pcm_data;
                            size_t total_mono_samples = pcm_bytes / sizeof(int16_t);

                            for (size_t s = 0; s < total_mono_samples && !global_interrupt; ) {
                                if (digitalRead(JOY_SW) == LOW) {
                                    Serial.println("TTS播放打断");
                                    global_interrupt = true;
                                    i2s_zero_dma_buffer(I2S_NUM_0);
                                    break;
                                }
                                size_t chunk_samples = min((size_t)512, total_mono_samples - s);
                                for (size_t i = 0; i < chunk_samples; i++) {
                                    int32_t mixed = (int32_t)(mono_ptr[s + i] * factor);
                                    if (mixed > 30000) mixed = 30000;
                                    else if (mixed < -30000) mixed = -30000;
                                    int16_t val = (int16_t)mixed;
                                    stereo_chunk[i * 2]     = val;
                                    stereo_chunk[i * 2 + 1] = val;
                                }
                                s += chunk_samples;
                                size_t bytes_to_write = chunk_samples * sizeof(int16_t) * 2;
                                size_t bw = 0;
                                i2s_write(I2S_NUM_0, stereo_chunk, bytes_to_write, &bw, portMAX_DELAY);
                            }
                        } else {
                            int16_t* stereo_ptr = (int16_t*)pcm_data;
                            size_t total_stereo_samples = pcm_bytes / sizeof(int16_t);
                            for (size_t s = 0; s < total_stereo_samples && !global_interrupt; ) {
                                if (digitalRead(JOY_SW) == LOW) {
                                    Serial.println("TTS播放打断");
                                    global_interrupt = true;
                                    i2s_zero_dma_buffer(I2S_NUM_0);
                                    break;
                                }
                                size_t chunk_samples = min((size_t)1024, total_stereo_samples - s);
                                for (size_t i = 0; i < chunk_samples; i++) {
                                    int32_t mixed = (int32_t)(stereo_ptr[s + i] * factor);
                                    if (mixed > 30000) mixed = 30000;
                                    else if (mixed < -30000) mixed = -30000;
                                    stereo_chunk[i] = (int16_t)mixed;
                                }
                                s += chunk_samples;
                                size_t bytes_to_write = chunk_samples * sizeof(int16_t);
                                size_t bw = 0;
                                i2s_write(I2S_NUM_0, stereo_chunk, bytes_to_write, &bw, portMAX_DELAY);
                            }
                        }
                        played = true;
                    } else {
                        Serial.printf("[ERR] TTS 音频数据无效或非 WAV 格式 (接收: %u 字节)\r\n", (unsigned)pcm_cache_len);
                    }
                    xSemaphoreGive(cache_mutex);
                } else {
                    Serial.printf("[ERR] OSS 音频下载失败: %d (%s)\r\n",
                                  audioCode, HTTPClient::errorToString(audioCode).c_str());
                    audioHttp.end();
                }
            }
            if (played && !global_interrupt) { // 🌟 被打断就不冲刷了，直接静音退出
                size_t silence_chunk = 4096;
                uint8_t* silence_buf = (uint8_t*)calloc(silence_chunk, 1);
                if (silence_buf) {
                    for(int s = 0; s < 12; s++) { size_t bw = 0; i2s_write(I2S_NUM_0, silence_buf, silence_chunk, &bw, portMAX_DELAY); }
                    free(silence_buf);
                }
                showOnScreenSafe("播报", "语音播放完毕。");
            }
        }
    } else {
        Serial.printf("[ERR] TTS 请求失败, HTTP Code: %d\r\n", httpCode);
    }
    AudioService::sleep(); // 🌟 播放结束，切断时钟让功放自动休眠
    http.end();

    // 🌟 音频焦点交接与开播路由（Skill 与语音对话生命周期标准实现）：
    int pendingVoiceTrack = DeviceActionService::getPendingVoiceTrack();
    if (pendingVoiceTrack >= 0) {
        DeviceActionService::clearPendingVoiceTrack();
        DeviceActionService::setVoiceInterrupted(false);
        s_music_paused_for_voice = false;
        Serial.printf("🎵 [VOICE-TURN] 语音播报完毕，正式启动技能点播曲目: #%02d\r\n", pendingVoiceTrack + 1);
        MusicPlayerService::sendPlay(pendingVoiceTrack);
    } else if (DeviceActionService::isVoiceInterrupted() || s_music_paused_for_voice) {
        DeviceActionService::setVoiceInterrupted(false);
        s_music_paused_for_voice = false;
        if (!MusicPlayerService::isPlaying()) {
            Serial.println("🎵 [VOICE-TURN] 语音播报完毕，平滑恢复被打断的音乐播放！");
            MusicPlayerService::sendResume();
        }
    }
}

// ==========================================
// [AI 识图：多模态图像识别与语音流式播报]
// ==========================================
void processVisionRecognitionTask() {
    Serial.println("\r\n========================================");
    Serial.println("[VISION-STAGE 1/6] 检查网络与快照状态...");
    if (!NetworkRuntime::cloudReady()) {
        Serial.println("[VISION-ERR] 网络未就绪或未连接 WiFi");
        CustomUiEngine::setAiVisionResult("网络未连接，请先连接 WiFi！");
        CustomUiEngine::notifyUiNeedsUpdate();
        return;
    }

    uint16_t* frame = CameraService::getFreezeFrame();
    if (!frame) {
        Serial.println("[VISION-ERR] 冻结帧快照为空");
        CustomUiEngine::setAiVisionResult("未获取到图像，请重新连接并开始识图！");
        CustomUiEngine::notifyUiNeedsUpdate();
        return;
    }

    uint32_t f_hash = 0;
    for (int i = 0; i < CameraService::FRAME_W * CameraService::FRAME_H; i += 16) {
        f_hash += frame[i];
    }
    Serial.printf("[VISION-STAGE 1/6 OK] 获得快照特征码: 0x%08X (确认图像唯一性)\r\n", f_hash);

    CustomUiEngine::setAiVisionResult("正在进行极速图像转码...");
    CustomUiEngine::notifyUiNeedsUpdate();

    // 1. 软件编码 RGB565 -> JPEG 二进制
    Serial.println("[VISION-STAGE 2/6] 启动标准正交 2D 矩阵 JPEG 编码 (160x120)...");
    uint32_t t_enc_start = millis();
    std::vector<uint8_t> jpeg_bytes;
    bool encode_ok = JpegEncoder::encodeRgb565(frame, CameraService::FRAME_W, CameraService::FRAME_H, 80, jpeg_bytes);
    uint32_t t_enc_ms = millis() - t_enc_start;

    if (!encode_ok || jpeg_bytes.empty()) {
        Serial.printf("[VISION-ERR] 图像转码失败 (耗时 %lu ms)\r\n", t_enc_ms);
        CustomUiEngine::setAiVisionResult("图像转码失败，请重试！");
        CustomUiEngine::notifyUiNeedsUpdate();
        return;
    }

    Serial.printf("[VISION-STAGE 2/6 OK] JPEG 编码完成: %u 字节 | 纯计算耗时: %lu ms\r\n", 
                  (unsigned int)jpeg_bytes.size(), t_enc_ms);

    // 2. Base64 转码
    Serial.println("[VISION-STAGE 3/6] 开始 Base64 格式化...");
    size_t est_b64_len = (jpeg_bytes.size() * 4 / 3) + 64;
    char* b64_buf = (char*)heap_caps_malloc(est_b64_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!b64_buf) b64_buf = (char*)malloc(est_b64_len);
    if (!b64_buf) {
        Serial.println("[VISION-ERR] 内存不足无法分配 Base64 缓冲");
        CustomUiEngine::setAiVisionResult("内存不足，无法进行 Base64 编码！");
        CustomUiEngine::notifyUiNeedsUpdate();
        return;
    }

    size_t b64_out_len = 0;
    int b64_ret = mbedtls_base64_encode((unsigned char*)b64_buf, est_b64_len, &b64_out_len,
                                        jpeg_bytes.data(), jpeg_bytes.size());
    if (b64_ret != 0) {
        free(b64_buf);
        Serial.println("[VISION-ERR] Base64 转换失败");
        CustomUiEngine::setAiVisionResult("Base64 编码异常！");
        CustomUiEngine::notifyUiNeedsUpdate();
        return;
    }
    b64_buf[b64_out_len] = '\0';
    Serial.printf("[VISION-STAGE 3/6 OK] Base64 字符串生成完成 (长度: %u 字符)\r\n", (unsigned int)b64_out_len);

    // 3. 构建标准 Qwen 3.5 全模态 (qwen3.5-omni-plus) 多模态 JSON 负载
    Serial.println("[VISION-STAGE 4/6] 组装 Qwen3.5-Omni 多模态 JSON 载荷...");
    CustomUiEngine::setAiVisionResult("正在上传云端大模型分析中...");
    CustomUiEngine::notifyUiNeedsUpdate();

    JsonDocument doc;
    doc["model"] = "qwen3.5-omni-plus";
    doc["modalities"].to<JsonArray>().add("text");
    JsonArray messages = doc["messages"].to<JsonArray>();
    JsonObject userMsg = messages.add<JsonObject>();
    userMsg["role"] = "user";
    JsonArray content = userMsg["content"].to<JsonArray>();

    JsonObject textItem = content.add<JsonObject>();
    textItem["type"] = "text";
    textItem["text"] = "这是微型低分辨率摄像头拍摄的实时照片。请忽略低像素与马赛克噪点，重点分析画面正中心的目标物品、手势或场景，直接回答这是什么，给出明确结论，50字以内。";

    JsonObject imgItem = content.add<JsonObject>();
    imgItem["type"] = "image_url";
    imgItem["image_url"]["url"] = String("data:image/jpeg;base64,") + b64_buf;

    free(b64_buf); // 及时释放 Base64 原始缓存

    String payload;
    serializeJson(doc, payload);

    Serial.printf("[VISION-STAGE 4/6 OK] 载荷构建完成 (JSON 总大小: %u 字节)\r\n", (unsigned int)payload.length());

    // 4. HTTPS 安全请求云端
    Serial.println("[VISION-STAGE 5/6] 建立 HTTPS TLS 连接向阿里云发起请求...");
    vTaskDelay(pdMS_TO_TICKS(5)); // 主动让出 CPU，喂狗

    WiFiClientSecure client;
    client.setInsecure();
    client.setTimeout(25000);

    HTTPClient http;
    http.begin(client, "https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions");
    http.setConnectTimeout(10000);
    http.setTimeout(25000);
    http.addHeader("Authorization", String(API_KEY));
    http.addHeader("Content-Type", "application/json");

    uint32_t t_http_start = millis();
    int httpCode = http.POST(payload);
    uint32_t t_http_ms = millis() - t_http_start;
    String ai_reply = "";

    if (httpCode == 200 || httpCode == HTTP_CODE_OK) {
        String resp = http.getString();
        JsonDocument respDoc;
        DeserializationError err = deserializeJson(respDoc, resp);
        if (!err) {
            const char* ans = respDoc["choices"][0]["message"]["content"];
            if (ans && strlen(ans) > 0) {
                ai_reply = String(ans);
                Serial.printf("[VISION-STAGE 5/6 OK] 云端识别成功 (耗时: %lu ms)\r\n", t_http_ms);
            } else {
                ai_reply = "大模型未返回描述内容。";
                Serial.println("[VISION-WARN] 大模型返回空内容");
            }
        } else {
            ai_reply = "云端响应解析失败。";
            Serial.println("[VISION-ERR] JSON 响应解析失败");
        }
    } else {
        Serial.printf("[VISION-ERR] HTTP 请求失败, Code: %d (耗时 %lu ms)\r\n", httpCode, t_http_ms);
        ai_reply = "识别失败 (HTTP " + String(httpCode) + ")";
    }

    http.end();
    client.stop();

    Serial.printf("[VISION] 最终识别文本: \"%s\"\r\n", ai_reply.c_str());

    // 5. 更新 UI 文本展示
    CustomUiEngine::setAiVisionResult(ai_reply);
    CustomUiEngine::notifyUiNeedsUpdate();

    // 6. 语音播报 TTS (如果识别成功)
    if (httpCode == 200 && ai_reply.length() > 0) {
        Serial.println("[VISION-STAGE 6/6] 启动 TTS 语音合成与板载扬声器流式播报...");
        playTTS(ai_reply);
        Serial.println("[VISION-STAGE 6/6 OK] 语音播报流程结束");
    }
    Serial.println("========================================\r\n");
}

// ==========================================
// [大模型交互：第二轮 流式 SSE]
// ==========================================
void requestCompletionPayload(const String& payload) {
    CommunicationGuard communicationGuard;
    if (!NetworkRuntime::cloudReady()) {
        showOnScreenSafe("网络不可用", "WiFi 或系统时间未就绪，无法请求云端服务");
        return;
    }
    showOnScreenSafe("云端思考中", "生成回复与语音播报...");

    std::unique_ptr<WiFiClientSecure> client(new WiFiClientSecure());
    client->setInsecure();
    client->setTimeout(25000);
    if (!client->connect(LLM_HOST, LLM_PORT, 10000)) {
        Serial.println("[ERR] 第二轮连接失败"); return;
    }

    client->print("POST /compatible-mode/v1/chat/completions HTTP/1.1\r\n");
    client->print("Host: " + String(LLM_HOST) + "\r\n");
    client->print("Authorization: " + String(API_KEY) + "\r\n");
    client->print("Content-Type: application/json\r\n");
    client->print("Accept: text/event-stream\r\n");
    client->print("Content-Length: " + String(payload.length()) + "\r\n");
    client->print("Connection: close\r\n\r\n");
    client->print(payload);

    // ========================================================
    // 阶段一：【绝对无脑全速接收】
    // ========================================================
    xSemaphoreTake(cache_mutex, portMAX_DELAY);
    pcm_cache_len = 0;
    raw_cache_len = 0;
    xSemaphoreGive(cache_mutex);

    uint8_t net_buf[2048];
    unsigned long timeout = millis();

    if (!consumeSuccessfulHttpHeaders(*client)) {
        client->stop();
        showOnScreenSafe("云端错误", "服务器拒绝了文本请求");
        return;
    }

    // 极速扒干 TCP 缓冲区
    bool rawCacheOverflow = false;
    while (client->connected() || client->available()) {
        if (global_interrupt || digitalRead(JOY_SW) == LOW || digitalRead(static_cast<int>(AppConfig::RETURN_BUTTON)) == LOW) {
            Serial.println("[NET] 打断切断TCP");
            triggerGlobalInterrupt();
            client->stop();
            client.reset();
            return;
        }
        if (client->available()) {
            timeout = millis();
            int bytes_read = client->read(net_buf, sizeof(net_buf));
            if (bytes_read > 0) {
                xSemaphoreTake(cache_mutex, portMAX_DELAY);
                if (raw_cache_len + bytes_read < MAX_RAW_CACHE_SIZE) {
                    memcpy(raw_cache + raw_cache_len, net_buf, bytes_read);
                    raw_cache_len += bytes_read;
                } else {
                    rawCacheOverflow = true;
                }
                xSemaphoreGive(cache_mutex);
            }
        } else {
            if (millis() - timeout > 15000) break;
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
    client->stop(); // 接收完毕，果断掐断 TCP
    
    // 🌟 核心修复：提前销毁大模型的 HTTP/TLS 上下文，释放 40KB 内部堆内存，给后面的 playTTS 留出空间
    client.reset();
    if (rawCacheOverflow) {
        showOnScreenSafe("云端错误", "回复超过解析缓存上限，请缩短问题后重试");
        return;
    }

    // ========================================================
    // 阶段二：【离线解析 PSRAM 中的数据】
    // ========================================================
    String full_ai_reply = "";
    full_ai_reply.reserve(4096);
    const size_t JSON_BUF_SIZE = 64 * 1024;            // 纯文本SSE每行<1KB
    char* pending_json = (char*)ps_malloc(JSON_BUF_SIZE);
    if (!pending_json) return;
    int pending_idx = 0;
    int line_start = 0;

    for (int i = 0; i < raw_cache_len; i++) {
        if (global_interrupt) {
            free(pending_json);
            return;
        }
        if (raw_cache[i] == '\n') {
            int line_len = i - line_start;
            char saved_cr = 0;
            if (line_len > 0 && raw_cache[i - 1] == '\r') {
                saved_cr = '\r';
                raw_cache[i - 1] = '\0';
                line_len--;
            }
            raw_cache[i] = '\0';

            char* current_line = raw_cache + line_start;

            bool is_hex = (line_len > 0 && line_len <= 8);
            for (int k = 0; k < line_len; k++) {
                char ch = current_line[k];
                if (!((ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f') || (ch >= 'A' && ch <= 'F'))) {
                    is_hex = false; break;
                }
            }

            if (is_hex || line_len == 0 || strcmp(current_line, "data: [DONE]") == 0) {
                raw_cache[i] = '\n';
                if (saved_cr) raw_cache[i - 1] = saved_cr;
                line_start = i + 1;
                continue;
            }

            // 跨行拼接完整的 JSON
            if (pending_idx == 0) {
                if (strncmp(current_line, "data: ", 6) == 0) {
                    memcpy(pending_json, current_line + 6, line_len - 6);
                    pending_idx = line_len - 6;
                    pending_json[pending_idx] = '\0';
                }
            } else {
                if (pending_idx + line_len < JSON_BUF_SIZE - 1) {
                    memcpy(pending_json + pending_idx, current_line, line_len);
                    pending_idx += line_len;
                    pending_json[pending_idx] = '\0';
                } else {
                    pending_idx = 0;
                }
            }

            // 尝试解析，解析成功后一次性解码 Base64
            if (pending_idx > 0) {
                JsonDocument doc;
                DeserializationError err = deserializeJson(doc, pending_json);

                if (err) {
                    // 🌟 核心修复 1：拦截内存溢出等致命错误，防止后续缓冲区雪崩
                    if (err != DeserializationError::IncompleteInput) {
                        Serial.printf("[ERR] JSON解析异常: %s\r\n", err.c_str());
                        pending_idx = 0;
                    }
                } else {
                    JsonObject delta = doc["choices"][0]["delta"];
                    if (delta["content"].is<const char*>()) full_ai_reply += delta["content"].as<const char*>();

                    if (delta["audio"]["data"].is<const char*>()) {
                        const char* b64_audio = delta["audio"]["data"].as<const char*>();
                        size_t b64_len = strlen(b64_audio);
                        size_t max_pcm_len = (b64_len * 3) / 4;
                        uint8_t* pcm_buf = (uint8_t*)ps_malloc(max_pcm_len);

                        if (pcm_buf) {
                            size_t pcm_len = 0;
                            if (mbedtls_base64_decode(pcm_buf, max_pcm_len, &pcm_len, (const unsigned char*)b64_audio, b64_len) == 0) {
                                xSemaphoreTake(cache_mutex, portMAX_DELAY);
                                if (pcm_cache_len + pcm_len < MAX_PCM_CACHE_SIZE) {
                                    memcpy(pcm_cache + pcm_cache_len, pcm_buf, pcm_len);
                                    pcm_cache_len += pcm_len;
                                }
                                xSemaphoreGive(cache_mutex);
                            } else {
                                Serial.println("[ERR] Base64 解码失败！");
                            }
                            free(pcm_buf);
                        } else {
                            Serial.println("[ERR] PSRAM 分配音频缓存失败，丢失一段音频！");
                        }
                    }
                    pending_idx = 0; // 成功解析，重置游标
                }
            }

            raw_cache[i] = '\n';
            if (saved_cr) raw_cache[i - 1] = saved_cr;
            line_start = i + 1;
        }
    }
    free(pending_json);

    // 一次性全局剥离 WAV 头
    if (pcm_cache_len > 12 && pcm_cache[0] == 'R' && pcm_cache[1] == 'I') {
        size_t offset = 0;
        for (size_t z = 12; z < pcm_cache_len - 4 && z < 200; z++) {
            if (pcm_cache[z] == 'd' && pcm_cache[z+1] == 'a' && pcm_cache[z+2] == 't' && pcm_cache[z+3] == 'a') {
                offset = z + 8; break;
            }
        }
        if (offset == 0) offset = 44;

        if (offset > 0 && offset < pcm_cache_len) {
            xSemaphoreTake(cache_mutex, portMAX_DELAY);
            size_t new_len = pcm_cache_len - offset;
            memmove(pcm_cache, pcm_cache + offset, new_len);
            pcm_cache_len = new_len;
            xSemaphoreGive(cache_mutex);
        }
    }

    // ========================================================
    // 阶段三：【解析与存入 PSRAM 聊天池】
    // ========================================================
    static auto parseUserAndAiReply = [](const String& fullText, String& outUserText, String& outAiReply) {
        outUserText = "";
        outAiReply = "";
        if (fullText.length() == 0) return;

        int userIdx = fullText.indexOf("用户：");
        if (userIdx == -1) userIdx = fullText.indexOf("用户:");

        int replyIdx = fullText.indexOf("回答：");
        if (replyIdx == -1) replyIdx = fullText.indexOf("回答:");
        if (replyIdx == -1) replyIdx = fullText.indexOf("AI：");
        if (replyIdx == -1) replyIdx = fullText.indexOf("AI:");

        if (userIdx != -1 && replyIdx != -1 && replyIdx > userIdx) {
            int userStart = userIdx + ((fullText.indexOf("用户：") != -1) ? strlen("用户：") : strlen("用户:"));
            outUserText = fullText.substring(userStart, replyIdx);
            outUserText.trim();

            int replyStart = replyIdx + ((fullText.indexOf("回答：", replyIdx) != -1) ? strlen("回答：") : strlen("回答:"));
            outAiReply = fullText.substring(replyStart);
            outAiReply.trim();
        } else if (replyIdx != -1) {
            int replyStart = replyIdx + ((fullText.indexOf("回答：", replyIdx) != -1) ? strlen("回答：") : strlen("回答:"));
            outAiReply = fullText.substring(replyStart);
            outAiReply.trim();
        } else {
            outAiReply = fullText;
            outAiReply.trim();
        }
    };

    String extractedUser = "";
    String extractedReply = "";
    parseUserAndAiReply(full_ai_reply, extractedUser, extractedReply);

    if (extractedUser.length() > 0) {
        AiChatStore::addMessage("user", extractedUser.c_str());
    }
    if (extractedReply.length() > 0) {
        AiChatStore::addMessage("assistant", extractedReply.c_str());
    } else if (full_ai_reply.length() > 0) {
        AiChatStore::addMessage("assistant", full_ai_reply.c_str());
    }
    currentText = AiChatStore::buildCombinedText();
    CustomUiEngine::notifyAiContentUpdate();

    // ========================================================
    // 阶段四：【集中播放 TTS】
    // ========================================================
    String textToPlay = (extractedReply.length() > 0) ? extractedReply : full_ai_reply;
    if (textToPlay.length() > 0 && !global_interrupt) {
        showOnScreenSafe("千问回复", textToPlay);
        beep(150, 1); // 🌟 收到最终文字，滴一声长音提示
        playTTS(textToPlay);
    } else if (!global_interrupt) {
        showOnScreenSafe("千问回复", "处理完毕。");
    }
}

void requestSecondTurnText(const String& prompt) {
    requestCompletionPayload(buildTextCompletionPayload(prompt, true));
}

void requestToolResultText(
    const String& toolCallId,
    const String& toolName,
    const String& toolArguments,
    const String& toolResult,
    bool enableSearch = false) {
    const String safeCallId = toolCallId.length() ? toolCallId : "call_local_fallback";
    requestCompletionPayload(buildToolResultPayload(
        safeCallId, toolName, toolArguments, toolResult, enableSearch));
}

// ==========================================
// [大模型交互：第一轮 流式 SSE + Function Calling]
// ==========================================
static bool s_is_voice_triggered = false;

static void sendRecordedWavOverSerial() {
    if (recorded_bytes == 0) return;
    
    uint8_t wavHeader[44];
    buildPcmWavHeader(wavHeader, recorded_bytes, 12000);
    
    Serial.printf("\r\n=== BEGIN_WAV_BASE64 SR=12000 ===\r\n");
    
    size_t total_raw = sizeof(wavHeader) + recorded_bytes;
    uint8_t* full_raw = (uint8_t*)ps_malloc(total_raw);
    if (full_raw) {
        memcpy(full_raw, wavHeader, sizeof(wavHeader));
        memcpy(full_raw + sizeof(wavHeader), psram_audio_buffer, recorded_bytes);
        
        size_t b64_len = 4 * ((total_raw + 2) / 3);
        char* b64_buf = (char*)ps_malloc(b64_len + 4);
        if (b64_buf) {
            size_t actual_out = 0;
            if (mbedtls_base64_encode((unsigned char*)b64_buf, b64_len + 4, &actual_out, full_raw, total_raw) == 0) {
                b64_buf[actual_out] = '\0';
                for (size_t i = 0; i < actual_out; i += 1024) {
                    size_t chunk = min((size_t)1024, actual_out - i);
                    Serial.write((const uint8_t*)b64_buf + i, chunk);
                }
            }
            free(b64_buf);
        }
        free(full_raw);
    }
    Serial.printf("\r\n=== END_WAV_BASE64 ===\r\n");
}

void uploadAudioToLLM() {
    CommunicationGuard communicationGuard;
    if (!NetworkRuntime::cloudReady()) {
        showOnScreenSafe("网络不可用", "WiFi 或系统时间未就绪，录音未上传");
        return;
    }
    showOnScreenSafe("云端分析中", "正在流式上传音频...");

    const String prefix = buildAudioRequestPrefix();
    const String suffix = "\"}}]}]}";
    const size_t wavBytes = 44 + recorded_bytes;
    const size_t total_len = prefix.length() + base64EncodedSize(wavBytes) + suffix.length();

    std::unique_ptr<WiFiClientSecure> client(new WiFiClientSecure());
    client->setInsecure();
    client->setTimeout(25000);
    if (!client->connect(LLM_HOST, LLM_PORT, 10000)) {
        Serial.println("[ERR] TCP 连接失败");
        return;
    }

    client->print("POST /compatible-mode/v1/chat/completions HTTP/1.1\r\n");
    client->print("Host: " + String(LLM_HOST) + "\r\n");
    client->print("Authorization: " + String(API_KEY) + "\r\n");
    client->print("Content-Type: application/json\r\n");
    client->print("Accept: text/event-stream\r\n");
    client->print("Content-Length: " + String(total_len) + "\r\n");
    client->print("Connection: close\r\n\r\n");

    bool uploadOk = writeClientFully(
        *client, reinterpret_cast<const uint8_t*>(prefix.c_str()), prefix.length(), &global_interrupt);
    if (uploadOk) {
        uploadOk = streamWavBase64(
            *client, psram_audio_buffer, recorded_bytes, 12000, &global_interrupt);
    }
    if (uploadOk) {
        uploadOk = writeClientFully(
            *client, reinterpret_cast<const uint8_t*>(suffix.c_str()), suffix.length(), &global_interrupt);
    }
    if (!uploadOk) {
        Serial.println(global_interrupt ? "[NET] 音频上传已被用户打断" : "[ERR] 音频流式上传失败");
        client->stop();
        return;
    }

    if (!consumeSuccessfulHttpHeaders(*client)) {
        client->stop();
        showOnScreenSafe("云端错误", "服务器拒绝了音频请求");
        return;
    }

    String full_ai_reply = "";
    String current_tool_call_id = "";
    String current_tool_name = "";
    String current_tool_args = "";
    
    // 🌟 修复: 预分配 String 内存，防止长文本流式拼接时产生大量的 Heap 内存碎片导致系统崩溃重启
    full_ai_reply.reserve(4096);
    current_tool_call_id.reserve(64);
    current_tool_name.reserve(64);
    current_tool_args.reserve(1024);


    // ========================================================
    // 阶段一：【绝对无脑全速接收】
    // ========================================================
    xSemaphoreTake(cache_mutex, portMAX_DELAY);
    pcm_cache_len = 0;
    raw_cache_len = 0;
    xSemaphoreGive(cache_mutex);

    uint8_t net_buf[2048];
    unsigned long timeout = millis();
    bool rawCacheOverflow = false;

    while (client->connected() || client->available()) {
        if (global_interrupt || digitalRead(JOY_SW) == LOW || digitalRead(static_cast<int>(AppConfig::RETURN_BUTTON)) == LOW) {
            Serial.println("[NET] 打断切断TCP");
            triggerGlobalInterrupt();
            client->stop();
            client.reset();
            return;
        }
        if (client->available()) {
            timeout = millis();
            int bytes_read = client->read(net_buf, sizeof(net_buf));
            if (bytes_read > 0) {
                xSemaphoreTake(cache_mutex, portMAX_DELAY);
                if (raw_cache_len + bytes_read < MAX_RAW_CACHE_SIZE) {
                    memcpy(raw_cache + raw_cache_len, net_buf, bytes_read);
                    raw_cache_len += bytes_read;
                } else {
                    rawCacheOverflow = true;
                }
                xSemaphoreGive(cache_mutex);
            }
        } else {
            if (millis() - timeout > 15000) {
                Serial.println("[NET] 接收 15 秒超时！");
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
    client->stop(); // 接收完毕，直接果断掐断 TCP，让服务器释放压力！
    
    // 🌟 核心修复：在这里提前销毁一层大模型的 HTTP/TLS 上下文，彻底释放约 40KB 内部 Heap 内存
    // 否则在后续调用 skillWebSearch 或 playTTS 时，多个 TLS 上下文并存会导致堆内存直接溢出重启！
    client.reset(); 
    
    Serial.printf("[NET] 接收包 %dB\r\n", raw_cache_len);
    if (rawCacheOverflow) {
        showOnScreenSafe("云端错误", "模型回复超过解析缓存上限，请缩短指令后重试");
        return;
    }

    // ========================================================
    // 阶段二：【离线解析 PSRAM 中的数据】
    // ========================================================
    showOnScreenSafe("数据解析中", "正在提取文本与音频...");

    // 🌟 找回我们之前验证过最稳的拼接缓冲池！
    const size_t JSON_BUF_SIZE = 64 * 1024;            // 纯文本SSE每行<1KB
    char* pending_json = (char*)ps_malloc(JSON_BUF_SIZE);
    if (!pending_json) {
        Serial.println("[ERR] 解析缓存分配失败");
        return;
    }
    int pending_idx = 0;

    int line_start = 0;
    for (int i = 0; i < raw_cache_len; i++) {
        if (global_interrupt) {
            free(pending_json);
            return;
        }
        if (raw_cache[i] == '\n') {
            int line_len = i - line_start;

            // 剥离 \r
            char saved_cr = 0;
            if (line_len > 0 && raw_cache[i - 1] == '\r') {
                saved_cr = '\r';
                raw_cache[i - 1] = '\0';
                line_len--;
            }
            raw_cache[i] = '\0';

            char* current_line = raw_cache + line_start;

            // 1. 判断是否十六进制 Chunk 标记
            bool is_hex = (line_len > 0 && line_len <= 8);
            for (int k = 0; k < line_len; k++) {
                char ch = current_line[k];
                if (!((ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f') || (ch >= 'A' && ch <= 'F'))) {
                    is_hex = false; break;
                }
            }

            // 2. 忽略杂音与结束符
            if (is_hex || line_len == 0 || strcmp(current_line, "data: [DONE]") == 0) {
                raw_cache[i] = '\n';
                if (saved_cr) raw_cache[i - 1] = saved_cr;
                line_start = i + 1;
                continue;
            }

            // 🌟 3. 核心修复：完美的跨行 JSON 缝合逻辑！
            if (pending_idx == 0) {
                // 如果是新起的一行 JSON
                if (strncmp(current_line, "data: ", 6) == 0) {
                    memcpy(pending_json, current_line + 6, line_len - 6);
                    pending_idx = line_len - 6;
                    pending_json[pending_idx] = '\0';
                }
            } else {
                // 如果上一行没解析成功，这里就是下半截，继续往后拼！
                if (pending_idx + line_len < JSON_BUF_SIZE - 1) {
                    memcpy(pending_json + pending_idx, current_line, line_len);
                    pending_idx += line_len;
                    pending_json[pending_idx] = '\0';
                } else {
                    pending_idx = 0; // 防溢出保护
                }
            }

            // 4. 尝试解析完整的缝合体
            if (pending_idx > 0) {
                JsonDocument doc;
                DeserializationError err = deserializeJson(doc, pending_json);

                if (err) {
                    // 🌟 核心修复 1：拦截溢出和坏块
                    if (err != DeserializationError::IncompleteInput) {
                        Serial.printf("[ERR] JSON解析异常: %s\r\n", err.c_str());
                        pending_idx = 0;
                    }
                } else {
                    JsonObject delta = doc["choices"][0]["delta"];

                    if (delta["content"].is<const char*>()) {
                        full_ai_reply += delta["content"].as<const char*>();
                    }

                    if (delta["tool_calls"]) {
                        JsonObject toolCall = delta["tool_calls"][0];
                        JsonObject func = toolCall["function"];
                        if (toolCall["id"].is<const char*>()) current_tool_call_id += toolCall["id"].as<String>();
                        if (func["name"].is<const char*>()) current_tool_name += func["name"].as<String>();
                        if (func["arguments"].is<const char*>()) current_tool_args += func["arguments"].as<String>();
                    }

                    if (delta["audio"]["data"].is<const char*>()) {
                        const char* b64_audio = delta["audio"]["data"].as<const char*>();
                        size_t b64_len = strlen(b64_audio);
                        size_t max_pcm_len = (b64_len * 3) / 4;

                        uint8_t* pcm_buf = (uint8_t*)ps_malloc(max_pcm_len);
                        if (pcm_buf) {
                            size_t pcm_len = 0;
                            if (mbedtls_base64_decode(pcm_buf, max_pcm_len, &pcm_len, (const unsigned char*)b64_audio, b64_len) == 0) {
                                xSemaphoreTake(cache_mutex, portMAX_DELAY);
                                if (pcm_cache_len + pcm_len < MAX_PCM_CACHE_SIZE) {
                                    memcpy(pcm_cache + pcm_cache_len, pcm_buf, pcm_len);
                                    pcm_cache_len += pcm_len;
                                }
                                xSemaphoreGive(cache_mutex);
                            } else {
                                Serial.println("[ERR] Base64 解码失败！");
                            }
                            free(pcm_buf);
                        } else {
                            Serial.println("[ERR] PSRAM 分配音频缓存失败，丢失一段音频！");
                        }
                    }
                    pending_idx = 0;
                }
            }

            // 🌟 还原 raw_cache，不破坏供 Web 下载的原始记录
            raw_cache[i] = '\n';
            if (saved_cr) raw_cache[i - 1] = saved_cr;

            line_start = i + 1; // 移动游标到下一行起点
        }
    } // for 循环解析完毕

    free(pending_json);

    static auto parseUserAndAiReply = [](const String& fullText, String& outUserText, String& outAiReply) {
        outUserText = "";
        outAiReply = "";
        if (fullText.length() == 0) return;

        int userIdx = fullText.indexOf("用户：");
        if (userIdx == -1) userIdx = fullText.indexOf("用户:");

        int replyIdx = fullText.indexOf("回答：");
        if (replyIdx == -1) replyIdx = fullText.indexOf("回答:");
        if (replyIdx == -1) replyIdx = fullText.indexOf("AI：");
        if (replyIdx == -1) replyIdx = fullText.indexOf("AI:");

        if (userIdx != -1 && replyIdx != -1 && replyIdx > userIdx) {
            int userStart = userIdx + ((fullText.indexOf("用户：") != -1) ? strlen("用户：") : strlen("用户:"));
            outUserText = fullText.substring(userStart, replyIdx);
            outUserText.trim();

            int replyStart = replyIdx + ((fullText.indexOf("回答：", replyIdx) != -1) ? strlen("回答：") : strlen("回答:"));
            outAiReply = fullText.substring(replyStart);
            outAiReply.trim();
        } else if (replyIdx != -1) {
            int replyStart = replyIdx + ((fullText.indexOf("回答：", replyIdx) != -1) ? strlen("回答：") : strlen("回答:"));
            outAiReply = fullText.substring(replyStart);
            outAiReply.trim();
        } else {
            outAiReply = fullText;
            outAiReply.trim();
        }
    };

    // ========================================================
    // 阶段三：【解析用户语音原文与 AI 回答，分别写入 PSRAM 对话池】
    // ========================================================
    String extractedUser = "";
    String extractedReply = "";
    parseUserAndAiReply(full_ai_reply, extractedUser, extractedReply);


    if (extractedUser.length() > 0) {
        AiChatStore::addMessage("我", extractedUser.c_str());
    }
    if (extractedReply.length() > 0) {
        AiChatStore::addMessage("千问", extractedReply.c_str());
    }
    currentText = AiChatStore::buildCombinedText();
    CustomUiEngine::notifyAiContentUpdate();

    // ========================================================
    // 阶段四：【TTS 语音朗读限制：仅朗读 AI 回答部分，绝不重读用户提问】
    // ========================================================
    String textToPlay = (extractedReply.length() > 0) ? extractedReply : full_ai_reply;
    if (textToPlay.length() > 0 && current_tool_name.length() == 0 && !global_interrupt) {
        showOnScreenSafe("千问回复", textToPlay);
        beep(150, 1); // 🌟 收到非工具的最终文字，滴一声长音提示开始播报
        playTTS(textToPlay);
    } else if (full_ai_reply.length() > 0 && current_tool_name.length() > 0 && !global_interrupt) {
        showOnScreenSafe("工具分析中", textToPlay);
        // 🌟 完美跳过 playTTS，极速去执行下方的 API 逻辑！
    }

    // ========================================
    // ========================================
    // 工具分发调度 (统一走 DeviceActionService 原子动作中台)
    // ========================================
    if (current_tool_name.length() > 0) {
        Serial.printf("[APP] LLM 请求调用工具: %s, 参数: %s\r\n", current_tool_name.c_str(), current_tool_args.c_str());
        JsonDocument argsDoc;
        if (current_tool_args.length() > 0) {
            deserializeJson(argsDoc, current_tool_args);
        }

        if (current_tool_name == "skillScanDevices") {
            showOnScreenSafe("调用技能", "正在扫描局域网...");
            DeviceActionService::ActionResult res = DeviceActionService::Action_ScanDevices();
            requestToolResultText(current_tool_call_id, current_tool_name, current_tool_args, res.message);

        // 🌟 新增：拦截搜索意图，融合开机定位，甩给第二轮带 enable_search:true 的接口
        } else if (current_tool_name == "skillWebSearch") {
            String user_query = argsDoc["query"] | "查询天气";
            showOnScreenSafe("云端检索", "正在搜寻全网资讯...");
            
            // 🌟 核心优化：融合实际用户提问、地理位置与检索词，构建精准联网 Prompt
            String actualQuestion = (extractedUser.length() > 0) ? extractedUser : user_query;
            String locPrefix = (current_city.length() > 0 && current_city != "定位中..." && current_city != "未知城市")
                               ? ("当前设备位置：" + current_city + "；") : "";
            String prompt = locPrefix + "用户实际提问：" + actualQuestion +
                            "；检索目标：" + user_query + "。请基于最新联网搜索数据精炼回答。";
            requestCompletionPayload(buildTextCompletionPayload(prompt, true));

        } else if (current_tool_name == "controlLight") {
            int action = argsDoc["action"] | 0;
            showOnScreenSafe("调用技能", action == 1 ? "正在开灯..." : "正在关灯...");
            DeviceActionService::ActionResult res = DeviceActionService::Action_SetLight(action == 1);
            requestToolResultText(current_tool_call_id, current_tool_name, current_tool_args, res.message);

        } else if (current_tool_name == "controlMusic") {
            String act = argsDoc["action"] | "play";
            String song = argsDoc["song"] | "";
            showOnScreenSafe("调用技能", "正在控制音乐...");
            DeviceActionService::ActionResult res;
            if (song.length() > 0) {
                res = DeviceActionService::Action_PlayTrackByName(song);
            } else if (act == "play") {
                res = DeviceActionService::Action_ControlMusic(DeviceActionService::MusicCmd::PLAY);
            } else if (act == "pause") {
                res = DeviceActionService::Action_ControlMusic(DeviceActionService::MusicCmd::PAUSE);
            } else if (act == "next") {
                res = DeviceActionService::Action_ControlMusic(DeviceActionService::MusicCmd::NEXT);
            } else if (act == "prev") {
                res = DeviceActionService::Action_ControlMusic(DeviceActionService::MusicCmd::PREV);
            } else if (act == "toggle_mode") {
                res = DeviceActionService::Action_ControlMusic(DeviceActionService::MusicCmd::TOGGLE_MODE);
            } else {
                res = { false, false, "未知音乐动作" };
            }
            requestToolResultText(current_tool_call_id, current_tool_name, current_tool_args, res.message);

        } else if (current_tool_name == "setVolume") {
            int vol = argsDoc["volume"] | 80;
            showOnScreenSafe("调用技能", "正在设置音量...");
            DeviceActionService::ActionResult res = DeviceActionService::Action_SetVolume(vol);
            requestToolResultText(current_tool_call_id, current_tool_name, current_tool_args, res.message);

        } else if (current_tool_name == "setBrightness") {
            int level = argsDoc["level"] | 80;
            showOnScreenSafe("调用技能", "正在调节亮度...");
            DeviceActionService::ActionResult res = DeviceActionService::Action_SetBrightness(level);
            requestToolResultText(current_tool_call_id, current_tool_name, current_tool_args, res.message);

        } else if (current_tool_name == "switchPage") {
            String pageStr = argsDoc["page"] | "home";
            pageStr.toLowerCase();
            int targetPage = PAGE_HOME;
            if (pageStr == "weather") targetPage = PAGE_WEATHER;
            else if (pageStr == "ai") targetPage = PAGE_AI;
            else if (pageStr == "music") targetPage = PAGE_MUSIC;
            else if (pageStr == "camera") targetPage = PAGE_CAMERA;
            else if (pageStr == "devices") targetPage = PAGE_DEVICES;
            else if (pageStr == "logs") targetPage = PAGE_LOGS;
            else if (pageStr == "settings") targetPage = PAGE_SETTINGS;
            
            showOnScreenSafe("调用技能", "正在切换界面...");
            DeviceActionService::ActionResult res = DeviceActionService::Action_SwitchPage(targetPage);
            requestToolResultText(current_tool_call_id, current_tool_name, current_tool_args, res.message);

        } else if (current_tool_name == "controlFlash") {
            int action = argsDoc["action"] | 0;
            showOnScreenSafe("调用技能", action == 1 ? "开启补光灯..." : "关闭补光灯...");
            DeviceActionService::ActionResult res = DeviceActionService::Action_SetFlash(action == 1);
            requestToolResultText(current_tool_call_id, current_tool_name, current_tool_args, res.message);

        } else if (current_tool_name == "readEnvironment") {
            showOnScreenSafe("调用技能", "正在读取环境数据...");
            DeviceActionService::ActionResult res = DeviceActionService::Action_ReadEnvironment();
            requestToolResultText(current_tool_call_id, current_tool_name, current_tool_args, res.message);

        } else if (current_tool_name == "readTempProbe") {
            showOnScreenSafe("调用技能", "正在读取探针温湿度...");
            DeviceActionService::ActionResult res = DeviceActionService::Action_ReadTempProbe();
            requestToolResultText(current_tool_call_id, current_tool_name, current_tool_args, res.message);

        } else if (current_tool_name == "readAmbientLight") {
            showOnScreenSafe("调用技能", "正在读取环境光照...");
            DeviceActionService::ActionResult res = DeviceActionService::Action_ReadAmbientLight();
            requestToolResultText(current_tool_call_id, current_tool_name, current_tool_args, res.message);

        } else {
            requestToolResultText(
                current_tool_call_id, current_tool_name, current_tool_args,
                "设备不支持该工具调用");
        }
    } else {
        if (full_ai_reply.length() == 0) showOnScreenSafe("千问回复", "处理完毕。");
    }

    // 🌟 每一轮大模型对话交互结束：硬件全量复位与缓冲区清零
    resetHardwarePostTurn();
}

// ==========================================
// [FreeRTOS 双核任务 & Setup]
// ==========================================
#define AUDIO_BUF_SIZE 1024
uint8_t audio_buffer[AUDIO_BUF_SIZE];

void core0_network_task(void *pvParameters) {
    bool record_stop_requested = false;
    uint32_t lastLocationAttempt = 0;
    uint32_t lastEnvironmentAutoQuery = 0;
    bool environmentAutoQueryStarted = false;

    // 🌟 两段式智能静音与超时时间戳控制器
    static uint32_t s_record_start_ms = 0;
    static uint32_t s_last_voice_ms = 0;
    static uint32_t s_speech_chunk_count = 0;
    static bool s_user_has_spoken = false;

    // 🌟 开机瞬态硬件丢弃计数器
    static int s_record_discard_count = 0;

    while (!app_tasks_started) vTaskDelay(pdMS_TO_TICKS(1));

    for (;;) {
        const uint32_t now = millis();
        const bool firstEnvironmentQueryDue =
            !environmentAutoQueryStarted && now >= 5000;
        const bool periodicEnvironmentQueryDue =
            environmentAutoQueryStarted &&
            now - lastEnvironmentAutoQuery >= AppConfig::ENVIRONMENT_UPDATE_INTERVAL_MS;
        // 三个传感器查询错峰启动: 每次只占约 60ms 射频, 避免 900ms 连续占满
        // 导致射频并发冲突或回包延迟
        static bool tempQueryPending = false;
        static bool lightQueryPending = false;
        static uint32_t tempQueryStartAt = 0;
        static uint32_t lightQueryStartAt = 0;
        if ((firstEnvironmentQueryDue || periodicEnvironmentQueryDue) &&
            now - CustomUiEngine::getLastInteractionTime() > 10000 &&
            !isRecording && !isCommunicating && !environment_request_pending) {
            if (startEnvironmentSensorQuery()) {
                environmentAutoQueryStarted = true;
                lastEnvironmentAutoQuery = now;
                tempQueryPending = true;
                lightQueryPending = true;
                // 从机收到第一条后保持常开 3 秒, 后续查询错峰 700ms 足够
                tempQueryStartAt = now + 700;
                lightQueryStartAt = now + 1400;
            }
        }
        if (tempQueryPending && now >= tempQueryStartAt && !temp_probe_request_pending &&
            now - CustomUiEngine::getLastInteractionTime() > 10000) {
            if (startTempProbeQuery()) {
                tempQueryPending = false;
            }
        }
        if (lightQueryPending && now >= lightQueryStartAt && !light_request_pending &&
            now - CustomUiEngine::getLastInteractionTime() > 10000) {
            if (startAmbientLightQuery()) {
                lightQueryPending = false;
            }
        }
        serviceEnvironmentSensorTimeout();
        serviceTempProbeTimeout();
        serviceAmbientLightTimeout();

        if (NetworkRuntime::consumeReconnectFlag()) {
            Serial.println("[APP] 检测到网络重连/切换，刷新全量网络信息...");
            lastLocationAttempt = millis();
            showOnScreenSafe("网络更新", "网络重连/切换，正在刷新信息...");
            refreshNetworkInfo(false);
        } else if (NetworkRuntime::cloudReady() && current_adcode.length() == 0 &&
            millis() - lastLocationAttempt >= 60000) {
            lastLocationAttempt = millis();
            showOnScreenSafe("网络已恢复", "正在补充定位和天气信息...");
            refreshNetworkInfo(false);
        }

        AppCommand command;
        while (receiveAppCommand(command)) {
            switch (command.type) {
                case AppCommandType::INTERRUPT_AND_RECORD:
                case AppCommandType::START_RECORDING:
                    if (!isRecording) {
                        isRecording = true;
                        record_stop_requested = false;
                        recorded_bytes = 0;
                        s_record_start_ms = millis();
                        s_last_voice_ms = millis();
                        s_user_has_spoken = false;
                        s_record_discard_count = 1800; // 🌟 硬件级丢弃前 150ms (彻底滤除手指按摇杆的机械震动)
                        CustomUiEngine::wakeUpScreen();
                        CustomUiEngine::setCurrentPage(PAGE_AI);
                        AudioService::initForTTS(AppConfig::AUDIO_SAMPLE_RATE);
                        AudioService::wake();
                        i2s_zero_dma_buffer(I2S_NUM_0);
                        size_t dummy_b = 0;
                        uint8_t dummy_buf[512];
                        i2s_read(I2S_NUM_0, dummy_buf, sizeof(dummy_buf), &dummy_b, pdMS_TO_TICKS(10));
                        beep(60, 1);
                        showOnScreenSafe("智能对话", "🔴 正在聆听... 说完自动识别或按摇杆");
                        Serial.println("🎙️ [AI-RECORD] 开始录音... 说完后将自动上传至大模型");
                        CustomUiEngine::notifyUiNeedsUpdate();
                    } else {
                        record_stop_requested = true;
                    }
                    break;

                case AppCommandType::STOP_RECORDING:
                    if (isRecording) {
                        record_stop_requested = true;
                    }
                    break;

                case AppCommandType::PLAY_PCM:
                    trigger_pcm_playback = true;
                    break;

                case AppCommandType::TOGGLE_LIGHT:
                    global_light_state = !global_light_state;
                    light_brightness = global_light_state ? 100 : 20;
                    smart_light_ack_state = -1; // 等待从机回执, 未收到前显示"未检测到"
                    sendRpcCommand(2, 0x01, global_light_state ? 100 : 20);
                    CustomUiEngine::notifyUiNeedsUpdate();
                    break;

                case AppCommandType::READ_ENVIRONMENT:
                    showOnScreenSafe("环境检测", "正在读取温度和气压...");
                    if (queryEnvironmentSensor()) {
                        showOnScreenSafe("环境数据", environmentReadingText());
                    } else {
                        showOnScreenSafe("环境检测", "读取失败，请检查GY-63或被控端连接");
                    }
                    break;

                case AppCommandType::READ_TEMPERATURE_PROBE:
                    showOnScreenSafe("外置温湿度", "正在读取 DHT 温湿度...");
                    if (queryTempProbeSensor()) {
                        showOnScreenSafe("外置温湿度", tempProbeReadingText());
                        environment_read_revision++;
                    } else {
                        showOnScreenSafe("外置温湿度", "读取失败，请检查连线");
                    }
                    break;

                case AppCommandType::READ_AMBIENT_LIGHT:
                    showOnScreenSafe("环境光照", "正在读取 VEML7700...");
                    if (queryAmbientLightSensor()) {
                        showOnScreenSafe("环境光照", ambientLightReadingText());
                    } else {
                        showOnScreenSafe("环境光照", "读取失败，请检查连接");
                    }
                    break;

                case AppCommandType::SCAN_DEVICES:
                    skillScanDevices();
                    break;

                case AppCommandType::START_VISION_AI:
                    processVisionRecognitionTask();
                    break;
            }
            releaseAppCommand(command);
        }

        // 🌟 拦截网页端上传完毕的 PCM 播放请求
        if (trigger_pcm_playback) {
            trigger_pcm_playback = false;
            showOnScreenSafe("Web 调试", "正在播放导入的音频...");

            xSemaphoreTake(cache_mutex, portMAX_DELAY);
            size_t play_len = pcm_cache_len;
            xSemaphoreGive(cache_mutex);

            // 分块推送 I2S，避免长时间阻塞引起看门狗复位
            size_t played = 0;
            AudioService::initForTTS(AppConfig::AUDIO_SAMPLE_RATE);
            AudioService::wake();
            while (played < play_len) {
                size_t chunk = (play_len - played > 4096) ? 4096 : (play_len - played);
                size_t bw = 0;
                i2s_write(I2S_NUM_0, pcm_cache + played, chunk, &bw, portMAX_DELAY);
                played += chunk;
                vTaskDelay(pdMS_TO_TICKS(1)); // 喂狗
            }
            AudioService::sleep();
            showOnScreenSafe("Web 调试", "播放完毕。");
        }

        // 🌟 录音启停控制：支持唤醒后 1.2 秒未说话自动取消、说话后停顿 1.2 秒自动发送、或手动摇杆结束
        bool trigger_send = false;
        bool trigger_cancel = false;

        if (isRecording) {
            uint32_t now = millis();
            // 🌟 1. 唤醒后 1.2 秒内从未开口说话 (!s_user_has_spoken)，直接判定为未发指令，静默取消交互
            if (!s_user_has_spoken && (now - s_record_start_ms >= 1200)) {
                trigger_cancel = true;
            }
            // 🌟 2. 正常录音结束：用户手动按摇杆，或说完话后停顿 1.2 秒，或达到 8 秒最大保护
            else if (record_stop_requested || 
                     (s_user_has_spoken && (now - s_last_voice_ms >= 1200)) || 
                     (now - s_record_start_ms >= 8000) || 
                     (recorded_bytes >= MAX_RECORD_SIZE)) {
                record_stop_requested = false;
                trigger_send = true;
            }
        }

        // 🚫 唤醒后 1.2 秒未说话：自动取消并返回待机主页
        if (isRecording && trigger_cancel) {
            isRecording = false;
            s_is_voice_triggered = false;
            record_stop_requested = false;
            VoiceTriggerService::setCooldown(1500); // 1.5 秒防抖冷却
            Serial.println("ℹ️ [VOICE-TRIGGER] 唤醒后 1.2 秒内未检测到指令，已自动取消交互并返回主页。");
            CustomUiEngine::setCurrentPage(PAGE_HOME);
            i2s_zero_dma_buffer(I2S_NUM_0);
            if (DeviceActionService::isVoiceInterrupted()) {
                DeviceActionService::setVoiceInterrupted(false);
                MusicPlayerService::sendResume();
            }
            CustomUiEngine::notifyUiNeedsUpdate();
        }

        // 🚀 执行自动上传至大模型 (AI 语音交互模式)
        if (isRecording && trigger_send) {
            isRecording = false;
            beep(35, 2);
            
            // 🌟 执行 Stanford CCRMA 1阶隔直滤波器后处理
            int16_t* pcm_ptr = (int16_t*)psram_audio_buffer;
            int total_n = recorded_bytes / 2;
            if (total_n > 200) {
                float x_prev = (float)pcm_ptr[0];
                float y_prev = 0.0f;
                const float R = 0.985f;

                for (int i = 0; i < total_n; i++) {
                    float x_curr = (float)pcm_ptr[i];
                    float y_curr = (x_curr - x_prev) + R * y_prev;
                    x_prev = x_curr;
                    y_prev = y_curr;
                    if (y_curr > 32700.0f) y_curr = 32700.0f;
                    else if (y_curr < -32700.0f) y_curr = -32700.0f;
                    pcm_ptr[i] = (int16_t)y_curr;
                }

                int fade_len = total_n < 240 ? total_n : 240;
                for (int i = 0; i < fade_len; i++) {
                    pcm_ptr[i] = (int16_t)(((int32_t)pcm_ptr[i] * i) / fade_len);
                }
            }

            // 🌟 核心：流式上传至云端大模型并实时语音播报
            uploadAudioToLLM();
            i2s_zero_dma_buffer(I2S_NUM_0);
            CustomUiEngine::notifyUiNeedsUpdate();
        }

        // ==========================================
        // 🎙️ 统一 I2S 音频流处理：麦克风采集与全天候本地离线唤醒
        // ==========================================
        if (!isCommunicating) {
            size_t bytes_read = 0;
            esp_err_t err = i2s_read(I2S_NUM_0, audio_buffer, AUDIO_BUF_SIZE, &bytes_read, pdMS_TO_TICKS(20));
            if (err != ESP_OK || bytes_read < 128) {
                AudioService::initForTTS(AppConfig::AUDIO_SAMPLE_RATE);
                AudioService::wake();
                vTaskDelay(pdMS_TO_TICKS(10));
            } else {
                int num_stereo_frames = (bytes_read / 4);
                int16_t* raw_samples = (int16_t*)audio_buffer;
                int16_t clean_samples[AUDIO_BUF_SIZE / 8];
                int clean_count = 0;
                int64_t sum_sq = 0;
                int32_t sum_val = 0;

                // 提取 12000 Hz 原生无重叠样本 (每 2 帧取 1 帧左声道)，施加 5.0x 高保真纯线性增益
                for (int f = 0; f < num_stereo_frames; f += 2) {
                    int32_t raw_val = (int32_t)raw_samples[f * 2]; // 🌟 硬件板载 INMP441 左声道真实麦克风
                    int32_t val = raw_val * 5;
                    if (val > 32700) val = 32700;
                    else if (val < -32700) val = -32700;
                    int16_t s = (int16_t)val;
                    clean_samples[clean_count++] = s;
                    sum_val += s;
                    sum_sq += (int64_t)s * s;
                }

                float mean = clean_count > 0 ? ((float)sum_val / clean_count) : 0.0f;
                float frame_rms = clean_count > 0 ? sqrtf((float)sum_sq / clean_count - mean * mean) : 0.0f;

                // 1. 录音中 (isRecording) - 保存用户指令音频
                if (isRecording) {
                    for (int i = 0; i < clean_count; i++) {
                        if (s_record_discard_count > 0) {
                            s_record_discard_count--;
                            continue;
                        }
                        if (recorded_bytes + 2 <= MAX_RECORD_SIZE) {
                            memcpy(psram_audio_buffer + recorded_bytes, &clean_samples[i], 2);
                            recorded_bytes += 2;
                        }
                    }

                    if (frame_rms > 1000.0f) {
                        s_speech_chunk_count++;
                        if (s_speech_chunk_count >= 2) {
                            s_user_has_spoken = true;
                        }
                        s_last_voice_ms = millis();
                    }
                }
                // 2. 待命状态 - 全天候本地离线声学关键词唤醒检测！
                else {
                    if (VoiceTriggerService::feedAudioFrame(clean_samples, clean_count)) {
                        isRecording = true;
                        record_stop_requested = false;
                        recorded_bytes = 0;
                        s_record_start_ms = millis();
                        s_last_voice_ms = millis();
                        s_user_has_spoken = false;
                        s_speech_chunk_count = 0;
                        s_record_discard_count = 0;
                        s_is_voice_triggered = true;

                        CustomUiEngine::wakeUpScreen();
                        CustomUiEngine::setCurrentPage(PAGE_AI);

                        // 🌟 若此时音乐正在播放，立刻静音暂停，杜绝功放杂音回灌麦克风
                        if (MusicPlayerService::isPlaying()) {
                            DeviceActionService::setVoiceInterrupted(true);
                            MusicPlayerService::sendPause();
                        }

                        AudioService::initForTTS(AppConfig::AUDIO_SAMPLE_RATE);
                        AudioService::wake();
                        i2s_zero_dma_buffer(I2S_NUM_0);
                        beep(60, 2);
                        showOnScreenSafe("小乐在听", "🟢 正在聆听您的指令...");
                        Serial.println("✨ [LOCAL-KWS] 🎯 唤醒成功！开始录音并准备向云端上传...");
                        CustomUiEngine::notifyUiNeedsUpdate();
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}



void core1_gui_task(void *pvParameters) {
    static uint32_t lastScrollTime = 0;
    static uint32_t lastSensorTime = 0;
    int16_t ax = 0, ay = 0, az = 0; // 全局保存 IMU 数据数据

    while (!app_tasks_started) vTaskDelay(pdMS_TO_TICKS(1));
    
    uint32_t last_loop_start = millis();

    for (;;) {
        uint32_t current_loop_start = millis();
        uint32_t full_loop_time = current_loop_start - last_loop_start;
        last_loop_start = current_loop_start;
        
        static uint32_t max_full_loop_time = 0;
        if (full_loop_time > max_full_loop_time) max_full_loop_time = full_loop_time;
        
        UiMessage message;
        while (receiveUiMessage(message)) {
            applyScreenMessage(message.title ? message.title : "系统", message.content ? message.content : "");
            releaseUiMessage(message);
        }

        // --- ICM42688 屏幕自适应旋转 (I2C) 自然手持灵敏度与防抖滤波 ---
        if (millis() - lastSensorTime > 150) {
            lastSensorTime = millis();
            if (ImuService::readAcceleration(ax, ay, az)) {
                // 尖峰毛刺滤除：计算加速度矢量模长，排除高频干扰读到的溢出脏数据
                int32_t mag_sq = (int32_t)ax * ax + (int32_t)ay * ay + (int32_t)az * az;
                if (mag_sq < 100000000) { // 剔除极端噪声
                    uint8_t candidate_rotation = current_rotation;
                    int16_t threshold = 1200; // 自然灵敏门限，手持倾斜即刻灵敏感应
                    
                    // 手持自然倾角（非完全水平平放）且未锁定旋转时判定
                    if (CustomUiEngine::getAutoRotateEnabled() &&
                        !CustomUiEngine::isRotationLocked() &&
                        az > -3500 && az < 3500) {
                        if (ay > threshold) candidate_rotation = 0;
                        else if (ax > threshold) candidate_rotation = 1;
                        else if (ay < -threshold) candidate_rotation = 2;
                        else if (ax < -threshold) candidate_rotation = 3;
                    }
                    
                    // 🌟 连贯防抖：连续 2 次采样（持续 300ms）一致即平滑触发旋转
                    static uint8_t pending_rotation = 255;
                    static uint8_t pending_count = 0;
                    
                    if (candidate_rotation != current_rotation) {
                        if (candidate_rotation == pending_rotation) {
                            pending_count++;
                            if (pending_count >= 2) {
                                current_rotation = candidate_rotation;
                                pending_count = 0;
                                
                                analogWrite(TFT_BLK, 0); // 重初始化期间关背光, 避免白屏闪烁

                                // 🌟 完整重置 IC 内部状态机，防止 CASET/RASET 错乱
                                tft.startWrite();
                                tft.writecommand(0x01); // SWRESET
                                tft.endWrite();
                                delay(150);
                                tft.startWrite();
                                tft.writecommand(0x11); // SLPOUT
                                tft.endWrite();
                                delay(120);
                                tft.startWrite();
                                tft.writecommand(0x3A); tft.writedata(0x55); // 16bit color
                                tft.endWrite();

                                tft.setRotation(current_rotation);

                                // 手动覆盖 MADCTL 修复面板镜像 (末尾 0x08 = TFT_MAD_BGR 修正偏色)
                                tft.startWrite();
                                tft.writecommand(0x36);
                                if (current_rotation == 0)      tft.writedata(0x48);
                                else if (current_rotation == 1) tft.writedata(0x28);
                                else if (current_rotation == 2) tft.writedata(0x88);
                                else if (current_rotation == 3) tft.writedata(0xE8);
                                tft.writecommand(0x29); // DISPON
                                tft.endWrite();
                                delay(50);

                                tft.invertDisplay(false); // 恢复深黑主题
                                tft.fillScreen(TFT_BLACK);
                                analogWrite(TFT_BLK, CustomUiEngine::getScreenBrightness());

                                CustomUiEngine::notifyUiNeedsUpdate(); // 触发 UI 重绘
                            }
                        } else {
                            pending_rotation = candidate_rotation;
                            pending_count = 1;
                        }
                    } else {
                        pending_count = 0;
                        pending_rotation = current_rotation;
                    }
                }
            }
        }
        
        // 🌟 纯 C++ 手绘 UI 摇杆与红外遥控全局防抖与长按连发引擎
        static uint32_t lastKeyPoll = 0;
        static int last_pressed_key = 0; // 0=none
        static uint32_t key_press_start = 0;
        static uint32_t last_repeat_time = 0;

        IRService::loop();

        if (millis() - lastKeyPoll >= 30) {
            lastKeyPoll = millis();
            // 4次多重平滑采样，滤除 WiFi 射频脉冲引起的 ADC 电源毛刺
            int vrx = 0;
            int vry = 0;
            for (int i = 0; i < 4; i++) {
                vrx += analogRead(JOY_VRX);
                vry += analogRead(JOY_VRY);
            }
            vrx /= 4;
            vry /= 4;

            int sw = digitalRead(JOY_SW);
            int ret = digitalRead(static_cast<int>(AppConfig::RETURN_BUTTON));

            // 🌟 1. 优先独立响应红外遥控脉冲 (IR Remote Pulse: 180ms 单次响应，零卡顿)
            int ir_key = 0;
            if (IRService::isEnter()) ir_key = CUSTOM_KEY_ENTER;
            else if (IRService::isEsc()) ir_key = CUSTOM_KEY_ESC;
            else if (IRService::isLeft()) ir_key = CUSTOM_KEY_LEFT;
            else if (IRService::isRight()) ir_key = CUSTOM_KEY_RIGHT;
            else if (IRService::isDown()) ir_key = CUSTOM_KEY_DOWN;
            else if (IRService::isUp()) ir_key = CUSTOM_KEY_UP;

            if (ir_key != 0) {
                static uint32_t last_ir_trigger_time = 0;
                if (millis() - last_ir_trigger_time > 180) {
                    last_ir_trigger_time = millis();
                    CustomUiEngine::handleKeyInput(ir_key);
                }
            } else {
                // 🌟 2. 独立响应板载双轴摇杆 (Joystick Analog: 保留 400ms 长按连发)
                bool is_enter = (sw == LOW);
                bool is_esc = (ret == LOW);

                // 🌟 拔出断开与引脚悬空保护 (Fail-Safe)：
                // 当拆掉摇杆时，GPIO 2(VRX) 与 GPIO 1(VRY) 悬空，漏电会将电平拉至 0V。
                // 若不进行悬空判定，vrx < 1000 会被误判为“向左”，并触发每 80ms 连发疯狂狂飙。
                // 正常接线摇杆即使被推到斜向极值角落，双轴也不可能同时跌破 150。
                bool joy_disconnected = (vrx < 150 && vry < 150);

                // 放宽死区：由 1000/3000 调整至 700/3300，防止机械回中偏差与 5V 错接引起的误报
                bool is_left = !joy_disconnected && (vrx < 700);
                bool is_right = !joy_disconnected && (vrx > 3300);
                bool is_down = !joy_disconnected && (vry < 700);
                bool is_up = !joy_disconnected && (vry > 3300);

                int joy_key = 0;
                if (is_enter) joy_key = CUSTOM_KEY_ENTER;
                else if (is_esc) joy_key = CUSTOM_KEY_ESC;
                else if (is_left) joy_key = CUSTOM_KEY_LEFT;
                else if (is_right) joy_key = CUSTOM_KEY_RIGHT;
                else if (is_down) joy_key = CUSTOM_KEY_DOWN;
                else if (is_up) joy_key = CUSTOM_KEY_UP;

                if (joy_key != 0) {
                    if (joy_key != last_pressed_key) {
                        CustomUiEngine::handleKeyInput(joy_key);
                        last_pressed_key = joy_key;
                        key_press_start = millis();
                        last_repeat_time = millis();
                    } else {
                        // 🌟 长按连发防误触与平滑加速：单次拨动( <280ms )严格只触发 1 次；持续按住则以 60ms->40ms 单首极速滚屏
                        if (millis() - key_press_start > 280) {
                            uint32_t repeat_interval = 80;
                            if (CustomUiEngine::getCurrentPage() == PAGE_MUSIC && CustomUiEngine::isPlaylistModalOpen()) {
                                uint32_t hold_time = millis() - key_press_start;
                                if (hold_time > 900) repeat_interval = 40; // 🌟 持续按住时每40ms平滑滚1首(25首/秒极速巡航)
                                else if (hold_time > 400) repeat_interval = 60; // 🌟 初始平滑连发
                            }
                            if (millis() - last_repeat_time > repeat_interval) {
                                CustomUiEngine::handleKeyInput(joy_key);
                                last_repeat_time = millis();
                            }
                        }
                    }
                } else {
                    if (last_pressed_key != 0) {
                        CustomUiEngine::handleKeyRelease(last_pressed_key);
                        last_pressed_key = 0;
                    }
                }
            }
        }

        uint32_t ui_loop_start = millis();

        CustomUiEngine::update();
        
        uint32_t ui_update_time = millis() - ui_loop_start;
        
        static uint32_t max_ui_time = 0;
        if (ui_update_time > max_ui_time) max_ui_time = ui_update_time;
        
        static uint32_t last_profiler_time = 0;
        if (CustomUiEngine::getCurrentPage() == PAGE_CAMERA && CameraService::state() == CameraService::State::STREAMING) {
            if (millis() - last_profiler_time > 1000) {
                last_profiler_time = millis();
                Serial.printf("[PROFILER] UI Update 纯耗时 MAX: %lu ms | UI 完整循环 MAX: %lu ms\r\n", max_ui_time, max_full_loop_time);
                max_ui_time = 0;
                max_full_loop_time = 0;
            }
            // 极限满帧刷新：使用任务通知代替死等 1ms，新帧到达时立刻唤醒
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));
        } else {
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
        }
    }
}

// ==========================================
// [Web 调试服务]
// ==========================================
void setup() {
    // 【关键修复】在所有初始化之前，立即钳位 HANDSHAKE=LOW！
    // 防止 ESP32 启动期间 GPIO21 浮空导致 STM32 误判为 HIGH 并提前推 SPI 帧
    pinMode(21, OUTPUT);
    digitalWrite(21, LOW);

    analogSetAttenuation(ADC_11db);
    DebugLog.begin(921600); // 🌟 921600 极速波特率 (比 115200 快 8 倍，秒传音频)
    delay(50);
    // 🌟 原生 USB CDC 接口初始化
    USB.begin();

    if (!appEventsInit()) {
        Serial.println("[FATAL] FreeRTOS 消息队列创建失败");
        fatal_hardware_error = true;
        return;
    }
    buzzer_mutex = xSemaphoreCreateMutex();
    if (!buzzer_mutex) {
        Serial.println("[FATAL] 蜂鸣器互斥锁创建失败");
        fatal_hardware_error = true;
        return;
    }
    esp_now_mutex = xSemaphoreCreateMutex();

    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, HIGH); // 默认高电平静音
    BuzzerService::begin(); // 无源蜂鸣器 (GPIO21) LEDC 初始化
    pinMode(static_cast<int>(AppConfig::RETURN_BUTTON), INPUT_PULLUP);
    if (AMP_EN != GPIO_NUM_NC) {
        pinMode(AMP_EN, OUTPUT);
        digitalWrite(AMP_EN, LOW);
    }

    if (!HardwareValidation::validateMemoryConfiguration()) {
        fatal_hardware_error = true;
        return;
    }
    if (!AudioService::begin()) {
        fatal_hardware_error = true;
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    cache_mutex = xSemaphoreCreateMutex();
    state_mutex = xSemaphoreCreateMutex();
    pcm_cache = (uint8_t*)ps_malloc(MAX_PCM_CACHE_SIZE);
    raw_cache = (char*)ps_malloc(MAX_RAW_CACHE_SIZE);

    if(!cache_mutex || !state_mutex || !pcm_cache || !raw_cache) {
        Serial.println("[FATAL] 缓存或互斥锁申请失败");
        fatal_hardware_error = true;
        return;
    }
    if (AMP_EN != GPIO_NUM_NC) digitalWrite(AMP_EN, HIGH); 

    psram_audio_buffer = (uint8_t*)ps_malloc(MAX_RECORD_SIZE);
    if (!psram_audio_buffer) {
        Serial.println("[FATAL] 录音缓冲区申请失败");
        fatal_hardware_error = true;
        return;
    }
    if (LED_PIN >= 0) {
        pinMode(LED_PIN, OUTPUT);
        digitalWrite(LED_PIN, LOW); 
    }
    pinMode(JOY_SW, INPUT_PULLUP);
    pinMode(static_cast<int>(AppConfig::RETURN_BUTTON), INPUT_PULLUP);
    
    // 🌟 1. 解除休眠状态下所有 RTC 管脚的内部电平锁与控制器重定向，彻底恢复纯净数字 GPIO 速度！
    rtc_gpio_deinit(static_cast<gpio_num_t>(AppConfig::IR_RECV_PIN));
    rtc_gpio_deinit(static_cast<gpio_num_t>(AppConfig::RETURN_BUTTON));
    gpio_num_t bl_pin = static_cast<gpio_num_t>(AppConfig::TFT_BACKLIGHT);
    gpio_hold_dis(bl_pin);
    pinMode(TFT_BLK, OUTPUT);
    digitalWrite(TFT_BLK, LOW);

    // 🌟 全局只初始化一次 IRService，避免重复 init 导致定时器重置脉冲失真
    IRService::init();

    // 🌟 2. 静默开机身份校验（静默监听，不点亮屏幕）
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
        Serial.println("[SYSTEM] 收到红外引脚唤醒 (EXT0)，正在严格校验红色关机按键 (POWER 0x00FFA25D)...");
        uint32_t check_start = millis();
        bool is_power_key = false;
        while (millis() - check_start < 600) {
            IRService::loop();
            uint64_t code = IRService::getLastRecvCode();
            // 严格仅匹配 0x00FFA25D 关机键码，或按住关机键产生的 NEC 重复码 0xFFFFFFFF
            if (code == IRService::getCodePowerOff() || code == 0x00FFA25D || code == 0xFFFFFFFF) {
                is_power_key = true;
                break;
            }
            delay(10);
        }
        if (!is_power_key) {
            Serial.println("[SYSTEM] 校验失败: 其它按键或杂波干涉，静默重新进入深度关机！");
            IRService::enterDeepSleep();
            return;
        }
        Serial.println("[SYSTEM] 红色关机键校验通过！解锁点亮屏幕！");
    } else if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
        Serial.println("[SYSTEM] 收到板载物理按键开机信号(GPIO 18)，正在静默校验 2 秒长按开机...");
        uint32_t press_start = millis();
        bool held_full_2s = true;
        while (millis() - press_start < 2000) {
            if (digitalRead(static_cast<int>(AppConfig::RETURN_BUTTON)) == HIGH) {
                held_full_2s = false; // 提前松开，说明是短按或误触！
                break;
            }
            delay(20);
        }
        if (!held_full_2s) {
            Serial.println("[SYSTEM] 校验失败: 板载按键未长按满 2 秒 (误触)，静默重新进入深度关机！");
            IRService::enterDeepSleep();
            return;
        }
        Serial.println("[SYSTEM] 板载按键 2 秒长按校验成功！准备点亮屏幕！");
    }

    // 🌟 静默开机校验完成，彻底擦除开机瞬间残留的红外脉冲碎片
    IRService::resetRecvBuffer();

    // 🌟 3. 校验通过后，初始化显示屏并点亮背光
    tft.begin(); tft.setRotation(DEFAULT_ROTATION);
    tft.startWrite();
    tft.writecommand(0x01); delay(150); // SWRESET
    tft.writecommand(0x11); delay(120); // SLPOUT 硬件唤醒屏幕
    tft.writecommand(0x3A); tft.writedata(0x55);
    // 根据默认方向设置物理寄存器
    tft.writecommand(0x36);
    if (DEFAULT_ROTATION == 0)      tft.writedata(0x48);
    else if (DEFAULT_ROTATION == 1) tft.writedata(0x28);
    else if (DEFAULT_ROTATION == 2) tft.writedata(0x88);
    else if (DEFAULT_ROTATION == 3) tft.writedata(0xE8);
    tft.writecommand(0x29); delay(50); // DISPON
    tft.endWrite();
    tft.invertDisplay(false); // 🌟 恢复原本正确的深黑主题 (INVOFF)
    tft.fillScreen(TFT_BLACK);
    analogWrite(TFT_BLK, 255); 

    // 🌟 初始化独立的 I2C 陀螺仪与红外遥控接收器
    ImuService::begin();
    if (!LittleFS.begin(true)) Serial.println("[ERR] LittleFS 挂载失败");
    
    // 🌟 每次开机/重启：深度清空 PSRAM 聊天记录池与界面文本，确保开机 UI 绝对干净无杂乱日志
    AiChatStore::init();
    currentText = "";
    // 原生USB初始化已移至 setup 开头，确保 TinyUSB 栈配置正确。

    showOnScreenSafe("启动中", "连接 WiFi...");
    const bool wifiConnected = NetworkRuntime::connect(AppConfig::WIFI_CONNECT_TIMEOUT_MS);
    if (wifiConnected) NetworkRuntime::synchronizeClock();

    // 🌟 初始化 纯 C++ 手绘 UI 引擎
    CameraService::init();
    CustomUiEngine::init(&tft);
    MusicPlayerService::begin(); // 启动 STM32 音乐播放器双向控制串口 UART1 (GPIO 0 / 48)
    DeviceActionService::init(); // 启动统一原子能力中台与状态孪生表
    VoiceTriggerService::init(); // 启动全时自适应声学语音唤醒引擎 (Core 0 后台监听)

    // 🌟 2. 绝对互斥与安全期：在按键线程启动前，强制阻塞获取定位与天气
    if (wifiConnected && NetworkRuntime::clockReady()) {
        showOnScreenSafe("系统初始化", "正在获取设备精准定位与天气...");
        refreshNetworkInfo(false);
    } else {
        showOnScreenSafe("离线模式", "WiFi 或时间服务不可用，云端功能将在重连后恢复");
    }
    
    beep(40, 1); // 短鸣一提示定位成功

    // 🌟 3. 继续原有逻辑
    if (esp_now_init() == ESP_OK) {
        esp_now_register_recv_cb(OnAckRecv);
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, broadcast_mac, 6);
        peerInfo.channel = 0;  peerInfo.encrypt = false;
        esp_now_add_peer(&peerInfo);
    }

    Serial.printf("SRAM: %dKB | PSRAM: %dKB\r\n",
                  ESP.getFreeHeap() / 1024,
                  ESP.getFreePsram() / 1024);

    showOnScreenSafe("系统就绪", "请按住摇杆开始对话！");
    beep(50, 2); // 🌟 开机完全成功音效(滴滴)

    // 🌟 一切就绪后，释放双核任务开始运行
    BaseType_t networkCreated = xTaskCreatePinnedToCore(core0_network_task, "NetworkTask", 16384, NULL, 1, &network_task_handle, 0);
    BaseType_t guiCreated = xTaskCreatePinnedToCore(core1_gui_task, "GUITask", 8192, NULL, 1, &gui_task_handle, 1);
    if (networkCreated != pdPASS || guiCreated != pdPASS) {
        Serial.println("[FATAL] 双核业务任务创建失败");
        fatal_hardware_error = true;
        return;
    }
    app_tasks_started = true;
}

void loop() {
    if (fatal_hardware_error) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        return;
    }
    NetworkRuntime::maintain();

    vTaskDelay(pdMS_TO_TICKS(10)); // 防止看门狗超时
}
