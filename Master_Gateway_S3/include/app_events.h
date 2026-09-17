#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>

enum class AppCommandType : uint8_t {
    START_RECORDING,
    STOP_RECORDING,
    INTERRUPT_AND_RECORD,
    TOGGLE_LIGHT,
    READ_ENVIRONMENT,
    READ_TEMPERATURE_PROBE,
    READ_AMBIENT_LIGHT,
    SCAN_DEVICES,
    PLAY_PCM,
    START_VISION_AI
};

struct AppCommand {
    AppCommandType type;
    char* text;
};

struct UiMessage {
    char* title;
    char* content;
};

bool appEventsInit();

bool postAppCommand(AppCommandType type, const String& text = String());
bool receiveAppCommand(AppCommand& command, TickType_t waitTicks = 0);
void releaseAppCommand(AppCommand& command);

bool postUiMessage(const String& title, const String& content);
bool receiveUiMessage(UiMessage& message, TickType_t waitTicks = 0);
void releaseUiMessage(UiMessage& message);
