#pragma once

#include <cstdint>

namespace RpcProtocol {
inline constexpr uint8_t DEVICE_SMART_SOCKET = 2;
inline constexpr uint8_t ACTION_DISCOVER = 0x00;
inline constexpr uint8_t ACTION_LIGHT = 0x01;
inline constexpr uint8_t ACTION_ENVIRONMENT = 0x04;
inline constexpr uint8_t ACTION_TEMPERATURE_PROBE = 0x05;
inline constexpr uint8_t ACTION_AMBIENT_LIGHT = 0x06;

inline constexpr uint8_t STATUS_FAILURE = 0;
inline constexpr uint8_t STATUS_SUCCESS = 1;
inline constexpr uint8_t STATUS_ENVIRONMENT = 4;
inline constexpr uint8_t STATUS_TEMPERATURE_PROBE = 5;
inline constexpr uint8_t STATUS_AMBIENT_LIGHT = 6;
}

struct __attribute__((packed)) RpcCommand {
    uint8_t device_id;
    uint8_t action;
    uint8_t value;
    uint8_t target_led;  
    uint8_t hue;         
    uint8_t speed;       // 动效速度 (0-250, 主机已放大)
    uint8_t brightness;  // 亮度 (0-100)
};

struct __attribute__((packed)) RpcAcknowledgement {
    uint8_t device_id;
    uint8_t is_success;
    float sensor_value;
    float sensor_value2;
};
