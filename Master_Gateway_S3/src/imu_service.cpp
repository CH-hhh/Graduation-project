#include "imu_service.h"

#include <Arduino.h>
#include <Wire.h>

#include "app_config.h"
#include "debug_log_service.h"
#include "i2c_bus_lock.h"
#define Serial DebugLog

namespace ImuService {
namespace {
constexpr uint32_t I2C_FREQUENCY_HZ = 100000;
constexpr uint16_t I2C_TIMEOUT_MS = 50; // 增加超时时间以容忍 USB 阻塞
constexpr uint8_t MAX_CONSECUTIVE_FAILURES = 10; // 增加容错次数

uint8_t address = AppConfig::ICM_I2C_ADDRESS;
uint8_t consecutiveFailures = 0;
bool sensorReady = false;
bool busStarted = false;
bool pollingSuspended = false;

void startBus() {
    if (busStarted) Wire.end();
    pinMode(static_cast<int>(AppConfig::ICM_I2C_SDA), INPUT_PULLUP);
    pinMode(static_cast<int>(AppConfig::ICM_I2C_SCL), INPUT_PULLUP);
    Wire.begin(
        static_cast<int>(AppConfig::ICM_I2C_SDA),
        static_cast<int>(AppConfig::ICM_I2C_SCL),
        I2C_FREQUENCY_HZ);
    Wire.setTimeOut(I2C_TIMEOUT_MS);
    busStarted = true;
}

bool configureAt(uint8_t candidate) {
    Wire.beginTransmission(candidate);
    Wire.write(0x75);
    if (Wire.endTransmission() != 0) return false;

    const size_t received = Wire.requestFrom(
        static_cast<uint16_t>(candidate), static_cast<uint8_t>(1));
    if (received != 1 || Wire.available() < 1) return false;
    const uint8_t whoAmI = Wire.read();
    if (whoAmI != 0x47) return false;

    Wire.beginTransmission(candidate);
    Wire.write(0x4E);
    Wire.write(0x0F);
    if (Wire.endTransmission() != 0) return false;

    address = candidate;
    delay(50);
    return true;
}

bool recoverSensor() {
    startBus();
    sensorReady = configureAt(address);
    if (!sensorReady) {
        const uint8_t alternate = address == 0x68 ? 0x69 : 0x68;
        sensorReady = configureAt(alternate);
    }

    consecutiveFailures = 0;
    if (sensorReady) {
        Serial.printf("[SYS] ICM42688 已连接，地址 0x%02X，I2C 100 kHz\n", address);
    }
    return sensorReady;
}

void recordFailure() {
    if (consecutiveFailures < UINT8_MAX) ++consecutiveFailures;
    if (consecutiveFailures < MAX_CONSECUTIVE_FAILURES) return;

    sensorReady = false;
    pollingSuspended = true;
    Serial.println("[WARN] IMU 通信失败，已停止自动旋转轮询，其他功能继续运行");
}
}

bool begin() {
    if (!i2cBusLock()) return false;
    const bool ready = recoverSensor();
    if (!ready) {
        Serial.println("IMU 未能就绪，扫描 I2C 寻找设备...");
        startBus();
        for (byte address = 1; address < 127; address++) {
            Wire.beginTransmission(address);
            if (Wire.endTransmission() == 0) {
                Serial.printf("发现 I2C 设备, 地址: 0x%02X\n", address);
            }
        }
    }
    pollingSuspended = !ready;
    i2cBusUnlock();
    return ready;
}

bool readAcceleration(int16_t& x, int16_t& y, int16_t& z) {
    if (pollingSuspended || !sensorReady) return false;
    if (!i2cBusLock()) return false;

    Wire.beginTransmission(address);
    Wire.write(0x1F);
    if (Wire.endTransmission() != 0) {
        recordFailure();
        i2cBusUnlock();
        return false;
    }

    const size_t received = Wire.requestFrom(
        static_cast<uint16_t>(address), static_cast<uint8_t>(6));
    if (received != 6 || Wire.available() < 6) {
        recordFailure();
        i2cBusUnlock();
        return false;
    }

    x = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
    y = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
    z = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
    consecutiveFailures = 0;
    i2cBusUnlock();
    return true;
}
}
