#include "chat_log_store.h"

#include <LittleFS.h>

namespace {
constexpr char LOG_PATH[] = "/chat_log.txt";
// 挤压式日志：超过 MAX_LOG_FILE_BYTES 后，只保留最新 TRIM_KEEP_BYTES，
// 最老的内容会被挤出，而不是整文件删掉重来。
constexpr size_t MAX_LOG_FILE_BYTES = 64 * 1024;
constexpr size_t TRIM_KEEP_BYTES = 32 * 1024;
constexpr size_t LOAD_TAIL_BYTES = 15 * 1024;

void trimLogToTail(size_t keepBytes) {
    File src = LittleFS.open(LOG_PATH, FILE_READ);
    if (!src) return;
    const size_t fileSize = src.size();
    if (fileSize <= keepBytes) {
        src.close();
        return;
    }
    // 跳到保留区起点，先丢掉被切掉一半的那一行，保证从完整行开始
    src.seek(fileSize - keepBytes);
    src.readStringUntil('\n');

    File dst = LittleFS.open("/chat_log.tmp", FILE_WRITE);
    if (!dst) {
        src.close();
        return;
    }
    uint8_t buf[256];
    int n;
    while (src.available() && (n = src.read(buf, sizeof(buf))) > 0) {
        dst.write(buf, (size_t)n);
    }
    src.close();
    dst.close();
    LittleFS.remove(LOG_PATH);
    LittleFS.rename("/chat_log.tmp", LOG_PATH);
}
}

bool appendChatLog(const String& title, const String& content) {
    File file = LittleFS.open(LOG_PATH, FILE_APPEND);
    if (!file) return false;
    file.println("> [" + title + "] " + content);
    file.close();

    File check = LittleFS.open(LOG_PATH, FILE_READ);
    if (!check) return true;
    const size_t fileSize = check.size();
    check.close();
    if (fileSize > MAX_LOG_FILE_BYTES) {
        trimLogToTail(TRIM_KEEP_BYTES);
    }
    return true;
}

size_t loadChatLogHistory(String* history, size_t capacity, String& combinedText) {
    combinedText = "";
    if (!history || capacity == 0) return 0;
    File file = LittleFS.open(LOG_PATH, FILE_READ);
    if (!file) return 0;

    const size_t fileSize = file.size();
    if (fileSize > MAX_LOG_FILE_BYTES) {
        file.close();
        trimLogToTail(TRIM_KEEP_BYTES);
        return 0;
    }
    if (fileSize > LOAD_TAIL_BYTES) {
        file.seek(fileSize - LOAD_TAIL_BYTES);
        file.readStringUntil('\n');
    }

    String* ring = new String[capacity];
    if (!ring) {
        file.close();
        return 0;
    }
    size_t count = 0;
    while (file.available()) {
        String line = file.readStringUntil('\n');
        line.trim();
        line.replace("▶", ">");
        if (line.length()) ring[count++ % capacity] = line;
    }
    file.close();

    const size_t valid = min(count, capacity);
    for (size_t i = 0; i < valid; ++i) {
        history[i] = ring[(count - 1 - i) % capacity];
    }
    for (int i = valid - 1; i >= 0; --i) {
        combinedText += history[i] + "\n";
    }
    delete[] ring;
    return valid;
}
