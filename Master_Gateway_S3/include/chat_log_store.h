#pragma once

#include <Arduino.h>

bool appendChatLog(const String& title, const String& content);
size_t loadChatLogHistory(String* history, size_t capacity, String& combinedText);
