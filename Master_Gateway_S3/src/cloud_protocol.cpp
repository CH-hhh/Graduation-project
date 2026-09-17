#include "cloud_protocol.h"

#include <ArduinoJson.h>
#include <mbedtls/base64.h>
#include "protocol_utils.h"
#include "device_action_service.h"
#include "ai_chat_store.h"
#include "debug_log_service.h"
#define Serial DebugLog

size_t base64EncodedSize(size_t rawBytes) {
    return encodedBase64Size(rawBytes);
}

String buildAudioRequestPrefix() {
    String stateCtx = DeviceActionService::Action_BuildSystemPromptStateContext();
    stateCtx.replace("\"", "\\\"");
    stateCtx.replace("\n", "\\n");

    String sysPrompt = "你是智能硬件中控大脑。必须严格遵循固定输出格式：\\n用户：<识别出的用户语音原文>\\n回答：<你的回答>\\n回答需简明扼要、直奔主题、拒绝客套废话。若回答内容较多，请精炼总结归纳在150字以内，确保回答完整且不被截断。\\n" + stateCtx;

    String json = F("{\"model\":\"qwen3.5-omni-plus\",\"temperature\":0.7,\"stream\":true,\"modalities\":[\"text\"],\"tools\":[");
    json += F("{\"type\":\"function\",\"function\":{\"name\":\"skillWebSearch\",\"description\":\"天气、百科、新闻等联网查询使用此工具\",\"parameters\":{\"type\":\"object\",\"properties\":{\"query\":{\"type\":\"string\",\"description\":\"查询关键词或问题\"}},\"required\":[\"query\"]}}},");
    json += F("{\"type\":\"function\",\"function\":{\"name\":\"controlLight\",\"description\":\"控制智能灯开关\",\"parameters\":{\"type\":\"object\",\"properties\":{\"action\":{\"type\":\"integer\",\"description\":\"1为开，0为关\"}},\"required\":[\"action\"]}}},");
    json += F("{\"type\":\"function\",\"function\":{\"name\":\"controlMusic\",\"description\":\"控制音乐播放\",\"parameters\":{\"type\":\"object\",\"properties\":{\"action\":{\"type\":\"string\",\"description\":\"play(播放/继续), pause(暂停), next(下一首), prev(上一首), toggle_mode(切换模式)\",\"enum\":[\"play\",\"pause\",\"next\",\"prev\",\"toggle_mode\"]},\"song\":{\"type\":\"string\",\"description\":\"可选:指定歌曲名\"}},\"required\":[\"action\"]}}},");
    json += F("{\"type\":\"function\",\"function\":{\"name\":\"setVolume\",\"description\":\"设置音乐音量\",\"parameters\":{\"type\":\"object\",\"properties\":{\"volume\":{\"type\":\"integer\",\"description\":\"音量百分比0-100\"}},\"required\":[\"volume\"]}}},");
    json += F("{\"type\":\"function\",\"function\":{\"name\":\"setBrightness\",\"description\":\"设置屏幕背光亮度\",\"parameters\":{\"type\":\"object\",\"properties\":{\"level\":{\"type\":\"integer\",\"description\":\"亮度百分比10-100\"}},\"required\":[\"level\"]}}},");
    json += F("{\"type\":\"function\",\"function\":{\"name\":\"switchPage\",\"description\":\"切换屏幕显示页面\",\"parameters\":{\"type\":\"object\",\"properties\":{\"page\":{\"type\":\"string\",\"description\":\"home(中控首页), weather(天气预报), ai(语音对话), music(音乐播放器), camera(摄像头), devices(设备列表), logs(系统日志), settings(系统设置)\"}},\"required\":[\"page\"]}}},");
    json += F("{\"type\":\"function\",\"function\":{\"name\":\"controlFlash\",\"description\":\"控制摄像头补光灯\",\"parameters\":{\"type\":\"object\",\"properties\":{\"action\":{\"type\":\"integer\",\"description\":\"1为开，0为关\"}},\"required\":[\"action\"]}}},");
    json += F("{\"type\":\"function\",\"function\":{\"name\":\"readEnvironment\",\"description\":\"读取室内实时温度、湿度和气压\"}},");
    json += F("{\"type\":\"function\",\"function\":{\"name\":\"readTempProbe\",\"description\":\"读取外置探针高精度温湿度\"}},");
    json += F("{\"type\":\"function\",\"function\":{\"name\":\"readAmbientLight\",\"description\":\"读取室内实时环境光照强度Lux\"}},");
    json += F("{\"type\":\"function\",\"function\":{\"name\":\"skillScanDevices\",\"description\":\"查询局域网有哪些智能设备在线\"}}");
    json += F("],\"messages\":[{\"role\":\"system\",\"content\":\"");
    json += sysPrompt;
    json += F("\"},");

    // 🌟 注入 PSRAM 多轮对话上下文历史 (最近 6 条消息 / 3 轮完整对话)
    String historyJson = AiChatStore::buildHistoryJson(6);
    if (historyJson.length() > 0) {
        json += historyJson;
    }

    json += F("{\"role\":\"user\",\"content\":[{\"type\":\"input_audio\",\"input_audio\":{\"format\":\"wav\",\"data\":\"data:audio/wav;base64,");
    return json;
}

String buildTextCompletionPayload(const String& prompt, bool enableSearch) {
    JsonDocument doc;
    doc["model"] = "qwen3.5-omni-plus";
    if (enableSearch) doc["enable_search"] = true;
    doc["temperature"] = 0.7;
    doc["stream"] = true;
    doc["modalities"].to<JsonArray>().add("text");
    JsonArray messages = doc["messages"].to<JsonArray>();
    JsonObject system = messages.add<JsonObject>();
    system["role"] = "system";
    system["content"] = "你是专职联网搜索工具引擎，只负责全网检索并直接回答问题。第一轮已识别用户提问，第二轮无需重复输出用户提问。必须严格遵循固定输出格式：\n回答：<你的搜索回答>\n回答必须极其简明扼要、直奔主题、拒绝客套废话。若搜索内容较多，必须精炼总结归纳在150字以内，确保信息准确完整且不被截断。严禁输出搜索过程、网页链接或无关说明。";

    // 🌟 注入最近 4 条历史上下文
    AiChatStore::injectRecentContext(messages, 4);

    JsonObject user = messages.add<JsonObject>();
    user["role"] = "user";
    user["content"] = prompt;
    String payload;
    serializeJson(doc, payload);
    return payload;
}

String buildToolResultPayload(
    const String& toolCallId,
    const String& toolName,
    const String& toolArguments,
    const String& toolResult,
    bool enableSearch) {
    JsonDocument doc;
    
    doc["model"] = "qwen3.5-omni-plus";
    if (enableSearch) doc["enable_search"] = true;
    doc["modalities"].to<JsonArray>().add("text");
    
    doc["temperature"] = 0.7;
    doc["stream"] = true;

    JsonArray messages = doc["messages"].to<JsonArray>();
    JsonObject system = messages.add<JsonObject>();
    system["role"] = "system";
    system["content"] = "你是智能硬件中控。你必须100%根据工具返回的真实结果（tool content）据实向用户汇报：\n"
                        "1. 若工具返回执行成功，自然精炼地告知用户操作已完成；\n"
                        "2. 若工具返回失败、未找到、超时、离线或报错，必须如实告知用户失败原因（如设备未响应、歌单未找到某歌等），绝对严禁在工具失败时谎称成功！\n"
                        "必须严格遵循固定输出格式：\n用户：<识别出的用户请求>\n回答：<你的回答>\n"
                        "回答需极其简明扼要、直奔主题，50字以内，拒绝客套废话。";

    // 🌟 注入最近 4 条历史上下文
    AiChatStore::injectRecentContext(messages, 4);

    JsonObject user = messages.add<JsonObject>();
    user["role"] = "user";
    user["content"] = "用户通过语音请求执行操作。";

    JsonObject assistant = messages.add<JsonObject>();
    assistant["role"] = "assistant";
    assistant["content"] = ""; // 必须是空字符串，不能是 null，否则 API 解析失败
    JsonObject call = assistant["tool_calls"].to<JsonArray>().add<JsonObject>();
    call["id"] = toolCallId;
    call["type"] = "function";
    call["function"]["name"] = toolName;
    call["function"]["arguments"] = toolArguments.length() ? toolArguments : "{}";

    JsonObject tool = messages.add<JsonObject>();
    tool["role"] = "tool";
    tool["tool_call_id"] = toolCallId;
    tool["content"] = toolResult;

    String payload;
    serializeJson(doc, payload);
    return payload;
}

bool writeClientFully(WiFiClient& client, const uint8_t* data, size_t length, volatile bool* interrupted, uint32_t timeoutMs) {
    size_t sent = 0;
    uint32_t lastProgress = millis();
    while (sent < length) {
        if (interrupted && *interrupted) return false;
        if (!client.connected()) return false;
        const size_t written = client.write(data + sent, length - sent);
        if (written > 0) {
            sent += written;
            lastProgress = millis();
        } else {
            if (millis() - lastProgress > timeoutMs) return false;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    return true;
}

bool streamWavBase64(WiFiClient& client, const uint8_t* pcm, size_t pcmBytes, uint32_t sampleRate, volatile bool* interrupted) {
    uint8_t wavHeader[44];
    buildPcmWavHeader(wavHeader, static_cast<uint32_t>(pcmBytes), sampleRate);

    constexpr size_t RAW_CHUNK = 3072;
    constexpr size_t BASE64_CHUNK = 4096;
    uint8_t raw[RAW_CHUNK];
    uint8_t encoded[BASE64_CHUNK + 1];
    const size_t total = sizeof(wavHeader) + pcmBytes;
    size_t offset = 0;

    while (offset < total) {
        const size_t count = min(RAW_CHUNK, total - offset);
        size_t copied = 0;
        if (offset < sizeof(wavHeader)) {
            const size_t headerBytes = min(count, sizeof(wavHeader) - offset);
            memcpy(raw, wavHeader + offset, headerBytes);
            copied += headerBytes;
        }
        if (copied < count) {
            const size_t pcmOffset = offset + copied - sizeof(wavHeader);
            memcpy(raw + copied, pcm + pcmOffset, count - copied);
        }

        size_t encodedBytes = 0;
        if (mbedtls_base64_encode(encoded, sizeof(encoded), &encodedBytes, raw, count) != 0) return false;
        
        // mbedtls 的 olen 可能会包含末尾的 '\0'，导致发送多余字节并使 Content-Length 失配
        // 这里显式计算不含 '\0' 的精确 Base64 长度
        size_t exactEncodedBytes = encodedBase64Size(count);
        if (!writeClientFully(client, encoded, exactEncodedBytes, interrupted)) return false;
        
        offset += count;
    }
    return true;
}
