#pragma once

#include <Arduino.h>
#include <IPAddress.h>
#include "rpc_protocol.h"

namespace CameraService {

// Raw TCP frame port, must match the CAM side.
inline constexpr uint16_t FRAME_PORT = 8899;
// Scaled RGB565 frame size pushed to the TFT.
inline constexpr uint16_t FRAME_W = 160;
inline constexpr uint16_t FRAME_H = 120;

enum class State : uint8_t {
    IDLE = 0,
    WAKING,
    CONNECTING,
    STREAMING,
    FAILED
};

// Called once from setup() (after ESP-NOW is up). Creates the camera task.
void init();

// UI-facing commands (only set flags; the camera task does the real work).
void start();
void stop();

// Called from the ESP-NOW receive callback (WiFi task context).
void onAck(const RpcAcknowledgement& ack);

// Keep the master radio awake while waking/streaming.
bool wantsRadioAwake();

State state();
bool hasFrame();
uint32_t frameRevision();
bool hasQueuedFrames();

uint32_t rxFps();
uint32_t rxTotalFps();
uint32_t rxGoodFps();
uint32_t rxBadFps();
String statusText();
String ipText();

// UI: lock the current ready frame while pushing it to the TFT.
uint16_t* lockFrame();
void unlockFrame();

// 冻结帧快照管理 (用于 AI 识图)
bool captureFreezeFrame();
uint16_t* getFreezeFrame();
void clearFreezeFrame();
bool hasFreezeFrame();

void toggleFlash();
bool isFlashOn();

}
