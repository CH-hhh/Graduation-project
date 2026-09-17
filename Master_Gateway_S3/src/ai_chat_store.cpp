#include "ai_chat_store.h"
#include <esp_heap_caps.h>
#include "debug_log_service.h"
#define Serial DebugLog

namespace AiChatStore {
namespace {
    ChatEntry* g_psram_entries = nullptr;
    size_t g_count = 0;
    size_t g_head = 0; // 环形 FIFO 头节点指针

    String escapeJsonString(const char* src) {
        if (!src) return "";
        String out = "";
        out.reserve(strlen(src) + 16);
        while (*src) {
            char c = *src++;
            if (c == '"') {
                out += "\\\"";
            } else if (c == '\\') {
                out += "\\\\";
            } else if (c == '\n') {
                out += "\\n";
            } else if (c == '\r') {
                out += "\\r";
            } else if (c == '\t') {
                out += "\\t";
            } else {
                out += c;
            }
        }
        return out;
    }
}

void init() {
    if (!g_psram_entries) {
        g_psram_entries = (ChatEntry*)heap_caps_malloc(sizeof(ChatEntry) * MAX_AI_ENTRIES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!g_psram_entries) {
            g_psram_entries = (ChatEntry*)malloc(sizeof(ChatEntry) * MAX_AI_ENTRIES);
        }
        if (g_psram_entries) {
            Serial.printf("[SYS] 🧠 PSRAM 对话上下文历史缓冲区就绪 (%u 条容量, %u 字节)\r\n", 
                          (unsigned int)MAX_AI_ENTRIES, (unsigned int)(sizeof(ChatEntry) * MAX_AI_ENTRIES));
        } else {
            Serial.println("[ERR] ❌ PSRAM 对话上下文缓冲区分配失败！");
        }
    }
    clear();
}

void clear() {
    if (g_psram_entries) {
        memset(g_psram_entries, 0, sizeof(ChatEntry) * MAX_AI_ENTRIES);
    }
    g_count = 0;
    g_head = 0;
}

void addMessage(const char* role, const char* content) {
    if (!g_psram_entries || !content || strlen(content) == 0) return;

    size_t write_idx;
    if (g_count < MAX_AI_ENTRIES) {
        write_idx = g_count;
        g_count++;
    } else {
        // FIFO 挤压淘汰机制：新的进，最老的被盖掉
        write_idx = g_head;
        g_head = (g_head + 1) % MAX_AI_ENTRIES;
    }

    memset(&g_psram_entries[write_idx], 0, sizeof(ChatEntry));
    
    // 统一角色标识为标准 API 格式 ("user" / "assistant")
    if (role) {
        if (strcmp(role, "用户") == 0 || strcmp(role, "我") == 0 || strcasecmp(role, "user") == 0) {
            strncpy(g_psram_entries[write_idx].role, "user", sizeof(g_psram_entries[write_idx].role) - 1);
        } else {
            strncpy(g_psram_entries[write_idx].role, "assistant", sizeof(g_psram_entries[write_idx].role) - 1);
        }
    } else {
        strncpy(g_psram_entries[write_idx].role, "user", sizeof(g_psram_entries[write_idx].role) - 1);
    }

    strncpy(g_psram_entries[write_idx].content, content, sizeof(g_psram_entries[write_idx].content) - 1);
}

String buildCombinedText() {
    if (!g_psram_entries || g_count == 0) return "";

    String result = "";
    for (size_t i = 0; i < g_count; i++) {
        size_t actual_idx = (g_head + i) % MAX_AI_ENTRIES;
        if (strlen(g_psram_entries[actual_idx].content) > 0) {
            if (result.length() > 0) result += "\n";
            const char* r = g_psram_entries[actual_idx].role;
            if (strcmp(r, "user") == 0) {
                result += "我: ";
            } else {
                result += "千问: ";
            }
            result += String(g_psram_entries[actual_idx].content);
        }
    }
    return result;
}

String buildHistoryJson(size_t maxEntries) {
    if (!g_psram_entries || g_count == 0) return "";

    size_t send_count = (g_count < maxEntries) ? g_count : maxEntries;
    size_t start_offset = g_count - send_count;

    String json = "";
    for (size_t i = 0; i < send_count; i++) {
        size_t actual_idx = (g_head + start_offset + i) % MAX_AI_ENTRIES;
        const char* r = g_psram_entries[actual_idx].role;
        const char* c = g_psram_entries[actual_idx].content;
        if (strlen(c) > 0) {
            json += "{\"role\":\"";
            json += (strcmp(r, "user") == 0) ? "user" : "assistant";
            json += "\",\"content\":\"";
            json += escapeJsonString(c);
            json += "\"},";
        }
    }
    return json;
}

void injectRecentContext(JsonArray& messagesArray, size_t maxEntries) {
    if (!g_psram_entries || g_count == 0) return;

    size_t send_count = (g_count < maxEntries) ? g_count : maxEntries;
    size_t start_offset = g_count - send_count;

    for (size_t i = 0; i < send_count; i++) {
        size_t actual_idx = (g_head + start_offset + i) % MAX_AI_ENTRIES;
        const char* r = g_psram_entries[actual_idx].role;
        const char* c = g_psram_entries[actual_idx].content;
        if (strlen(c) > 0) {
            JsonObject msg = messagesArray.add<JsonObject>();
            msg["role"] = (strcmp(r, "user") == 0) ? "user" : "assistant";
            msg["content"] = c;
        }
    }
}

size_t getCount() {
    return g_count;
}

} // namespace AiChatStore
