#pragma once

#include <Arduino.h>

namespace NetworkRuntime {
bool connect(uint32_t timeoutMs);
bool reconnect(const String& newSsid, const String& newPass, uint32_t timeoutMs = 12000);
String getActiveSsid();
bool synchronizeClock(uint32_t timeoutMs = 10000);
void maintain();
bool clockReady();
bool cloudReady();
bool consumeReconnectFlag();
void setScanBusy(bool busy);
bool isScanBusy();
}

extern void refreshNetworkInfo(bool forceClockSync = true);
