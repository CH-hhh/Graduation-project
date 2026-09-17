#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <math.h>
#include <DHT.h>
#include <WebServer.h>     
#include <Adafruit_VEML7700.h>

// ==========================================
// 🌟 传感器与执行器引脚定义
// ==========================================
#define LED_PIN 6
#define DHTPIN 5     
#define DHTTYPE DHT11   
#define GY63_SDA 11
#define GY63_SCL 12

// ==========================================
// 🌟 屏幕引脚 (0.91寸 SW_I2C)
// ==========================================
#define OLED_SCL 1   
#define OLED_SDA 2   

U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R2, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ U8X8_PIN_NONE);

static DHT dht(DHTPIN, DHTTYPE);
WebServer server(80); 
Adafruit_VEML7700 veml = Adafruit_VEML7700();


// ==========================================
// 🌍 Web 服务器根路径页面
// ==========================================
void handleRoot() {
    String html = "<!DOCTYPE html><html><head><meta charset='utf-8'>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<title>EdgeNode 智能传感器节点</title>";
    html += "<style>body{font-family:sans-serif;text-align:center;padding:40px;background:#1a1a1a;color:#fff;}h2{color:#00ffcc;}</style></head>";
    html += "<body><h2>EdgeNode 智能传感器节点</h2>";
    html += "<p>设备运行正常，正在监听主控通信指令。</p></body></html>";
    server.send(200, "text/html", html);
}


// ==========================================
// 传感器实现占位 (与上文保持一致)
// ==========================================
void skill_dht_init() {
    dht.begin();
    Serial.println("[DHT] 初始化完成");
}
bool skill_dht_read(float &temperature, float &humidity) {
    float h = dht.readHumidity(); float t = dht.readTemperature();
    if (isnan(h) || isnan(t)) return false;
    temperature = t; humidity = h; return true;
}

void skill_veml7700_init() {
    if (veml.begin(&Wire1)) {
        // 恢复默认设置，实际读取时使用 VEML_LUX_AUTO 动态自适应调节
        veml.setGain(VEML7700_GAIN_1);
        veml.setIntegrationTime(VEML7700_IT_100MS);
        Serial.println("[VEML7700] 初始化完成");
    } else {
        Serial.println("[VEML7700] 未找到传感器");
    }
}

bool skill_veml7700_read(float &lux) {
    lux = veml.readLux(VEML_LUX_AUTO);
    if (lux < 0) return false;
    return true;
}

namespace {
    uint8_t sensorAddress = 0;
    uint16_t calibration[8] = {};
    bool sensorReady = false;

    bool sendCommand(uint8_t command) {
        if (sensorAddress == 0) return false;
        Wire1.beginTransmission(sensorAddress);
        Wire1.write(command);
        return Wire1.endTransmission() == 0;
    }

    uint16_t readProm(uint8_t index) {
        Wire1.beginTransmission(sensorAddress);
        Wire1.write(0xA0 + index * 2);
        if (Wire1.endTransmission() != 0) return 0;
        if (Wire1.requestFrom(sensorAddress, static_cast<uint8_t>(2)) != 2) return 0;
        return (static_cast<uint16_t>(Wire1.read()) << 8) | Wire1.read();
    }

    uint32_t readAdc(uint8_t conversionCommand) {
        if (!sendCommand(conversionCommand)) return 0;
        delay(10);  
        Wire1.beginTransmission(sensorAddress);
        Wire1.write(0x00);
        if (Wire1.endTransmission() != 0) return 0;
        if (Wire1.requestFrom(sensorAddress, static_cast<uint8_t>(3)) != 3) return 0;
        uint32_t value = static_cast<uint32_t>(Wire1.read()) << 16;
        value |= static_cast<uint32_t>(Wire1.read()) << 8;
        value |= Wire1.read();
        return value;
    }

    uint8_t calculateCrc4(const uint16_t source[8]) {
        uint16_t prom[8];
        memcpy(prom, source, sizeof(prom));
        prom[7] &= 0xFF00;
        uint16_t remainder = 0;
        for (uint8_t byteIndex = 0; byteIndex < 16; ++byteIndex) {
            remainder ^= (byteIndex & 1) ? (prom[byteIndex >> 1] & 0x00FF) : (prom[byteIndex >> 1] >> 8);
            for (uint8_t bit = 0; bit < 8; ++bit) {
                remainder = (remainder & 0x8000) ? static_cast<uint16_t>((remainder << 1) ^ 0x3000) : static_cast<uint16_t>(remainder << 1);
            }
        }
        return static_cast<uint8_t>((remainder >> 12) & 0x0F);
    }

    bool calculateReading(uint32_t d1, uint32_t d2, float &temperature, float &pressure) {
        if (d1 == 0 || d1 == 0xFFFFFF || d2 == 0 || d2 == 0xFFFFFF) return false;
        const int64_t deltaT = static_cast<int64_t>(d2) - static_cast<int64_t>(calibration[5]) * 256;
        int64_t temp = 2000 + deltaT * calibration[6] / 8388608;
        int64_t offset = static_cast<int64_t>(calibration[2]) * 65536 + deltaT * calibration[4] / 128;
        int64_t sensitivity = static_cast<int64_t>(calibration[1]) * 32768 + deltaT * calibration[3] / 256;
        if (temp < 2000) {
            const int64_t tempDelta = temp - 2000;
            const int64_t t2 = deltaT * deltaT / 2147483648LL;
            int64_t offset2 = 5 * tempDelta * tempDelta / 2;
            int64_t sensitivity2 = 5 * tempDelta * tempDelta / 4;
            if (temp < -1500) {
                const int64_t coldDelta = temp + 1500;
                offset2 += 7 * coldDelta * coldDelta;
                sensitivity2 += 11 * coldDelta * coldDelta / 2;
            }
            temp -= t2; offset -= offset2; sensitivity -= sensitivity2;
        }
        const int64_t pressureHundredthMbar = ((static_cast<int64_t>(d1) * sensitivity / 2097152) - offset) / 32768;
        temperature = temp / 100.0f;
        pressure = pressureHundredthMbar / 100.0f;
        return isfinite(temperature) && isfinite(pressure) && temperature >= -40.0f && temperature <= 85.0f && pressure >= 100.0f && pressure <= 1200.0f;
    }

    bool loadCalibration() {
        const uint8_t addresses[] = {0x76, 0x77};
        for (uint8_t address : addresses) {
            sensorAddress = address;
            if (!sendCommand(0x1E)) continue;
            delay(4);
            uint16_t firstRead[8]; uint16_t secondRead[8];
            for (uint8_t i = 0; i < 8; ++i) firstRead[i] = readProm(i);
            delay(1);
            for (uint8_t i = 0; i < 8; ++i) secondRead[i] = readProm(i);
            const bool stable = memcmp(firstRead, secondRead, sizeof(firstRead)) == 0;
            bool coefficientsValid = true;
            for (uint8_t i = 1; i <= 6; ++i) {
                if (firstRead[i] == 0 || firstRead[i] == 0xFFFF) { coefficientsValid = false; break; }
            }
            const uint8_t storedCrc = firstRead[7] & 0x0F;
            const uint8_t calculatedCrc = calculateCrc4(firstRead);
            if (!stable || !coefficientsValid) continue;
            memcpy(calibration, firstRead, sizeof(calibration));
            if (storedCrc == calculatedCrc) return true;
            if (storedCrc == 0) {
                float testTemperature = 0.0f, testPressure = 0.0f;
                if (calculateReading(readAdc(0x48), readAdc(0x58), testTemperature, testPressure)) return true;
            }
        }
        sensorAddress = 0; return false;
    }
} 

void skill_gy63_init() { 
    sensorReady = loadCalibration(); 
    if (sensorReady) Serial.printf("[GY-63] 模块初始化成功，地址 0x%02X！\n", sensorAddress);
}
bool skill_gy63_read(float &temperature, float &pressure) { 
    if (!sensorReady) sensorReady = loadCalibration();
    if (!sensorReady) return false;
    for (uint8_t attempt = 0; attempt < 3; ++attempt) {
        if (calculateReading(readAdc(0x48), readAdc(0x58), temperature, pressure)) return true;
        delay(5);
    }
    sensorReady = false; return false; 
}


// ==========================================
// 主应用逻辑与全局变量
// ==========================================
typedef struct __attribute__((packed)) { uint8_t device_id; uint8_t action; uint8_t value; uint8_t target_led; uint8_t hue; uint8_t speed; uint8_t brightness; } rpc_cmd_t;
typedef struct __attribute__((packed)) { uint8_t device_id; uint8_t is_success; float sensor_value; float sensor_value2; } rpc_ack_t;

volatile bool task_toggle = false;
volatile bool task_read_gy63 = false;
volatile bool task_read_dht = false;
volatile bool task_read_veml7700 = false;
volatile bool task_discover = false;
volatile uint8_t target_led_value = 0;

volatile unsigned long radio_awake_until = 0; // 收到指令后保持射频常开的时间点

uint8_t master_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
bool master_mac_registered = false;
volatile bool send_done = false;
volatile bool send_success = false;

unsigned long oled_restore_time = 0;
bool is_oled_waiting = false;



#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
#else
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#endif
    send_success = (status == ESP_NOW_SEND_SUCCESS);
    send_done = true;
}

void updateOLED(String title, String oled_msg, String serial_msg = "") {
    u8g2.clearBuffer(); 
    u8g2.setCursor(0, 2); u8g2.print("[ " + title + " ]");
    u8g2.drawLine(0, 16, 128, 16);
    u8g2.setCursor(0, 18); u8g2.print(oled_msg);
    u8g2.sendBuffer(); 
    if (serial_msg != "") Serial.println("[" + title + "] " + serial_msg);
}

void trigger_oled_restore() {
    oled_restore_time = millis() + 1500; 
    is_oled_waiting = true;
}

void send_ack_to_master(uint8_t status_code, float value = 0.0, float value2 = 0.0) {
    rpc_ack_t ack = {2, status_code, value, value2}; 
    if (!master_mac_registered) return;
    int retry = 0;
    while (retry < 20) { 
        send_done = false;
        send_success = false;
        // 发送失败直接重试, 避免永远等不到回调
        if (esp_now_send(master_mac, (uint8_t *) &ack, sizeof(ack)) != ESP_OK) {
            retry++; delay(20);
            continue;
        }
        // 等待送达确认, 单次最多 200ms, 不会长时间占住主循环
        unsigned long wait_start = millis();
        while (!send_done && millis() - wait_start < 200) { delay(1); }
        if (send_success) break; 
        retry++; delay(20); 
    }
}

void toggle_led(uint8_t value) {
    // value: 0~100 亮度, <=20 视为关灯
    if (value <= 20) {
        ledcWrite(0, 0);
    } else {
        ledcWrite(0, (value * 255) / 100);
    }
    delay(10);
    send_ack_to_master(1);
}

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *incomingData, int len) {
    const uint8_t * mac = info->src_addr;
#else
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
#endif
    if (len != sizeof(rpc_cmd_t)) return; 
    if (!master_mac_registered || memcmp(master_mac, mac, 6) != 0) {
        memcpy(master_mac, mac, 6);
        if (esp_now_is_peer_exist(master_mac)) esp_now_del_peer(master_mac);
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, master_mac, 6);
        esp_now_add_peer(&peerInfo);
        master_mac_registered = true;
    }
    rpc_cmd_t cmd; memcpy(&cmd, incomingData, sizeof(cmd));
    // 收到任意指令: 射频保持常开 3 秒, 让主控后续指令与本机回包一次命中
    radio_awake_until = millis() + 3000;
    // 主控对传感器读取指令会重发多次: 500ms 内相同读取只处理一次, 避免重复回传数据
    if (cmd.device_id == 2 && (cmd.action == 0x04 ||
        cmd.action == 0x05 || cmd.action == 0x06)) {
        static unsigned long last_read_time = 0;
        static uint8_t last_read_action = 0;
        unsigned long read_now = millis();
        if (cmd.action == last_read_action && read_now - last_read_time < 500) return;
        last_read_time = read_now;
        last_read_action = cmd.action;
    }
    if (cmd.device_id == 2) {
        if (cmd.action == 0x00) task_discover = true;
        else if (cmd.action == 0x01) { target_led_value = cmd.value; task_toggle = true; }
        else if (cmd.action == 0x04) task_read_gy63 = true;
        else if (cmd.action == 0x05) task_read_dht = true;
        else if (cmd.action == 0x06) task_read_veml7700 = true;
        
    }
}



void setup() {
    Serial.begin(115200); delay(1000);
    
    // 屏幕初始化
    u8g2.setI2CAddress(0x3C * 2);
    u8g2.begin(); u8g2.setContrast(255); 
    u8g2.enableUTF8Print(); u8g2.setFont(u8g2_font_wqy12_t_gb2312); u8g2.setFontPosTop();
    
    // 传感器初始化
    Wire1.begin(11, 12); Wire1.setTimeOut(20); 
    skill_gy63_init(); skill_dht_init(); skill_veml7700_init();
    
    // 灯光 LED: 用 LEDC PWM 控制亮度 (5kHz, 8bit)
    ledcSetup(0, 5000, 8);
    ledcAttachPin(LED_PIN, 0);
    ledcWrite(0, 0);

    // WiFi & ESP-NOW
    WiFi.mode(WIFI_STA);
    WiFi.begin("liukem-2.4G", "ruheneliukem"); 
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    updateOLED("网络就绪", WiFi.localIP().toString());
    
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM); 
    esp_now_init();
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_register_send_cb(OnDataSent);

    // 启动 Web 服务器
    server.on("/", HTTP_GET, handleRoot);
    server.begin();
    Serial.println("HTTP Server 已启动！");
    
    updateOLED("设备待命", "监听控制指令");
}

void loop() {
    // 射频策略: 收到指令后常开 3 秒 (连续指令与回包期间保持常开);
    // 平时按 300ms 醒 / 250ms 睡循环, 让主控任意 300ms 以上的发送窗口必然命中常开段
    static bool slave_radio_awake = false;
    static unsigned long cycle_mark = 0;
    static bool cycle_on = false;
    bool want_awake = (millis() < radio_awake_until);
    if (!want_awake) {
        if (millis() - cycle_mark >= (cycle_on ? 300UL : 250UL)) {
            cycle_mark = millis();
            cycle_on = !cycle_on;
            want_awake = cycle_on;
        } else {
            want_awake = cycle_on;
        }
    }
    if (want_awake != slave_radio_awake) {
        slave_radio_awake = want_awake;
        esp_wifi_set_ps(want_awake ? WIFI_PS_NONE : WIFI_PS_MIN_MODEM);
    }

    // 1. 处理 Web 请求
    server.handleClient();

    // 2. UI 恢复与传感器任务 (非阻塞执行)
    if (is_oled_waiting && millis() > oled_restore_time) {
        is_oled_waiting = false;
        updateOLED("设备待命", "监听控制指令");
    }

    if (task_discover) {
        task_discover = false;
        updateOLED("自检中", "正在检测...");
        send_ack_to_master(0, 0x1D); // 状态0避免被主机误判为灯光回执, 0x1D=传感器掩码 (LED, GY63, DHT, OLED)
        updateOLED("自检完成", "结果已回传");
        trigger_oled_restore();
    }
    if (task_toggle) {
        task_toggle = false;
        toggle_led(target_led_value);
        char ledMsg[32];
        if (target_led_value <= 20) snprintf(ledMsg, sizeof(ledMsg), "灯已关闭");
        else snprintf(ledMsg, sizeof(ledMsg), "灯亮度 %d%%", target_led_value);
        updateOLED("执行结果", ledMsg);
        trigger_oled_restore();
    }
    if (task_read_gy63) {
        task_read_gy63 = false;
        float temp = 0.0, press = 0.0;
        if (skill_gy63_read(temp, press)) {
            send_ack_to_master(4, temp, press);
            updateOLED("气象数据", "T:" + String(temp, 1) + "C P:" + String(press, 0) + "hPa");
        }
        trigger_oled_restore();
    }
    if (task_read_dht) {
        task_read_dht = false;
        float temp = 0.0, hum = 0.0;
        if (skill_dht_read(temp, hum)) {
            send_ack_to_master(5, temp, hum);
            updateOLED("温湿度", "T:" + String(temp, 1) + "C H:" + String(hum, 0) + "%");
        }
        trigger_oled_restore();
    }
    if (task_read_veml7700) {
        task_read_veml7700 = false;
        float lux = 0.0;
        if (skill_veml7700_read(lux)) {
            send_ack_to_master(6, lux);
            updateOLED("环境光照", "Lux: " + String(lux, 1));
        }
        trigger_oled_restore();
    }
    
    // 3. 心跳日志 (让用户知道没死机)
    static unsigned long last_heartbeat = 0;
    if (millis() - last_heartbeat > 5000) {
        Serial.printf("[系统心跳] 运行正常 | 传感器就绪 | IP: %s\n", WiFi.localIP().toString().c_str());
        last_heartbeat = millis();
    }
    
    delay(10); // 防止跑满 CPU
}
