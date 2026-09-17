#include "app_events.h"

#include <esp_heap_caps.h>
#include <freertos/queue.h>

namespace {
QueueHandle_t commandQueue = nullptr;
QueueHandle_t uiQueue = nullptr;

char* duplicateText(const String& value) {
    const size_t bytes = value.length() + 1;
    char* copy = static_cast<char*>(heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!copy) copy = static_cast<char*>(malloc(bytes));
    if (!copy) return nullptr;
    memcpy(copy, value.c_str(), bytes);
    return copy;
}
}

bool appEventsInit() {
    if (!commandQueue) commandQueue = xQueueCreate(12, sizeof(AppCommand));
    if (!uiQueue) uiQueue = xQueueCreate(16, sizeof(UiMessage));
    return commandQueue != nullptr && uiQueue != nullptr;
}

bool postAppCommand(AppCommandType type, const String& text) {
    if (!commandQueue) return false;
    AppCommand command{type, nullptr};
    if (text.length() > 0) {
        command.text = duplicateText(text);
        if (!command.text) return false;
    }
    if (xQueueSend(commandQueue, &command, 0) != pdTRUE) {
        free(command.text);
        return false;
    }
    return true;
}

bool receiveAppCommand(AppCommand& command, TickType_t waitTicks) {
    command.text = nullptr;
    return commandQueue && xQueueReceive(commandQueue, &command, waitTicks) == pdTRUE;
}

void releaseAppCommand(AppCommand& command) {
    free(command.text);
    command.text = nullptr;
}

bool postUiMessage(const String& title, const String& content) {
    if (!uiQueue) return false;
    UiMessage message{duplicateText(title), duplicateText(content)};
    if (!message.title || !message.content) {
        free(message.title);
        free(message.content);
        return false;
    }
    if (xQueueSend(uiQueue, &message, 0) != pdTRUE) {
        free(message.title);
        free(message.content);
        return false;
    }
    return true;
}

bool receiveUiMessage(UiMessage& message, TickType_t waitTicks) {
    message.title = nullptr;
    message.content = nullptr;
    return uiQueue && xQueueReceive(uiQueue, &message, waitTicks) == pdTRUE;
}

void releaseUiMessage(UiMessage& message) {
    free(message.title);
    free(message.content);
    message.title = nullptr;
    message.content = nullptr;
}
