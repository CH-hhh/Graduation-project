#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

namespace AiChatStore {
    constexpr size_t MAX_AI_ENTRIES = 20; // 🌟 在 PSRAM 中保存最近 20 条消息 (10 轮完整多轮上下文)

    struct ChatEntry {
        char role[16];     // "user" 或 "assistant"
        char content[512]; // 对话单条文本内容
    };

    void init();
    void clear();
    void addMessage(const char* role, const char* content);
    String buildCombinedText();
    String buildHistoryJson(size_t maxEntries = 8); // 生成已转义的 JSON 片段供流式 Prefix 使用
    void injectRecentContext(JsonArray& messagesArray, size_t maxEntries = 8);
    size_t getCount();
}
