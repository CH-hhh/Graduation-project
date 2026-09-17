#include "network_runtime.h"

#include <WiFi.h>
#include <esp_wifi.h>
#include <time.h>

#include <Preferences.h>
#include "app_config.h"
#include "private_config.h"
#include "debug_log_service.h"
#define Serial DebugLog

// main.cpp: 灯效控制/等待回包期间临时关闭主机休眠
extern bool isRadioAwakeRequested();

namespace NetworkRuntime {
namespace {
bool clockSynchronized = false;
bool reconnectedFlag = false;
String activeSsid;
String activePassword;

void loadCredentials() {
    Preferences prefs;
    if (prefs.begin("wifi_cfg", true)) {
        String savedS = prefs.getString("ssid", "");
        String savedP = prefs.getString("pass", "");
        prefs.end();
        if (savedS.length() > 0) {
            activeSsid = savedS;
            activePassword = savedP;
            return;
        }
    }
    activeSsid = PrivateConfig::WIFI_SSID;
    activePassword = PrivateConfig::WIFI_PASSWORD;
}

void saveCredentials(const String& ssid, const String& pass) {
    Preferences prefs;
    if (prefs.begin("wifi_cfg", false)) {
        prefs.putString("ssid", ssid);
        prefs.putString("pass", pass);
        prefs.end();
    }
    activeSsid = ssid;
    activePassword = pass;
}
}

String getActiveSsid() {
    if (activeSsid.length() == 0) loadCredentials();
    return activeSsid;
}

bool connect(uint32_t timeoutMs) {
    if (activeSsid.length() == 0) loadCredentials();
    WiFi.mode(WIFI_STA);
    WiFi.begin(activeSsid.c_str(), activePassword.c_str());
    const uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) delay(250);
    if (WiFi.status() != WL_CONNECTED) {
        Serial.printf("[ERR] WiFi连接超时(离线): %s\r\n", activeSsid.c_str());
        return false;
    }
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    Serial.printf("[NET] WiFi已连接 IP: %s (SSID: %s)\r\n", WiFi.localIP().toString().c_str(), activeSsid.c_str());
    return true;
}

bool reconnect(const String& newSsid, const String& newPass, uint32_t timeoutMs) {
    Serial.printf("[NET] 准备连接新 WiFi: %s\r\n", newSsid.c_str());
    WiFi.disconnect(false, true);
    delay(150);
    WiFi.mode(WIFI_STA);
    WiFi.begin(newSsid.c_str(), newPass.c_str());
    const uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) delay(250);
    if (WiFi.status() != WL_CONNECTED) {
        Serial.printf("[ERR] 连接新 WiFi 失败: %s，正在恢复旧连接...\r\n", newSsid.c_str());
        if (activeSsid.length() > 0) {
            WiFi.disconnect();
            WiFi.begin(activeSsid.c_str(), activePassword.c_str());
        }
        return false;
    }
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    saveCredentials(newSsid, newPass);
    Serial.printf("[NET] 新 WiFi 连接成功并持久化保存: %s (IP: %s)\r\n", newSsid.c_str(), WiFi.localIP().toString().c_str());
    synchronizeClock(8000);
    reconnectedFlag = true;
    return true;
}

bool synchronizeClock(uint32_t timeoutMs) {
    if (WiFi.status() != WL_CONNECTED) return false;
    configTime(8 * 3600, 0, "ntp.aliyun.com", "pool.ntp.org", "time.cloudflare.com");
    const uint32_t start = millis();
    time_t now = time(nullptr);
    while (now < 1700000000 && millis() - start < timeoutMs) {
        delay(200);
        now = time(nullptr);
    }
    clockSynchronized = now >= 1700000000;
    if (!clockSynchronized) Serial.println("[ERR] NTP校时失败");
    return clockSynchronized;
}

static bool s_scanBusy = false;

void setScanBusy(bool busy) {
    s_scanBusy = busy;
}

bool isScanBusy() {
    return s_scanBusy;
}

void maintain() {
    if (s_scanBusy) return; // 🌟 处于 WiFi 扫描或连接期间，禁止后台自动轮询重连，防止打断射频

    static uint32_t lastRetry = 0;
    static wl_status_t previousStatus = WL_NO_SHIELD;
    const wl_status_t status = WiFi.status();
    if (status == WL_CONNECTED) {
        // 请求-响应或灯效控制期间: 主机射频常开保证回包一次命中, 结束后自动恢复休眠
        static bool radio_awake = false;
        const bool need_awake = isRadioAwakeRequested();
        if (need_awake != radio_awake) {
            radio_awake = need_awake;
            esp_wifi_set_ps(need_awake ? WIFI_PS_NONE : WIFI_PS_MIN_MODEM);
        }
        if (previousStatus != WL_CONNECTED) {
            Serial.printf("[NET] WiFi已恢复 IP: %s\r\n", WiFi.localIP().toString().c_str());
            synchronizeClock(8000);
            reconnectedFlag = true;
        }
    } else if (millis() - lastRetry >= AppConfig::WIFI_RETRY_INTERVAL_MS) {
        lastRetry = millis();
        if (activeSsid.length() == 0) loadCredentials();
        WiFi.disconnect();
        WiFi.begin(activeSsid.c_str(), activePassword.c_str());
    }
    previousStatus = status;
}

bool clockReady() {
    return clockSynchronized;
}

bool cloudReady() {
    return WiFi.status() == WL_CONNECTED && clockSynchronized;
}

bool consumeReconnectFlag() {
    if (reconnectedFlag) {
        reconnectedFlag = false;
        return true;
    }
    return false;
}
}
