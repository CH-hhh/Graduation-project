#pragma once

#include <Arduino.h>
#include <WiFiClient.h>

size_t base64EncodedSize(size_t rawBytes);
String buildAudioRequestPrefix();
String buildTextCompletionPayload(const String& prompt, bool enableSearch = true);
String buildToolResultPayload(
    const String& toolCallId,
    const String& toolName,
    const String& toolArguments,
    const String& toolResult,
    bool enableSearch);

bool writeClientFully(
    WiFiClient& client,
    const uint8_t* data,
    size_t length,
    volatile bool* interrupted = nullptr,
    uint32_t timeoutMs = 15000);

bool streamWavBase64(
    WiFiClient& client,
    const uint8_t* pcm,
    size_t pcmBytes,
    uint32_t sampleRate,
    volatile bool* interrupted = nullptr);
