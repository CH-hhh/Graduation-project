#include "i2c_bus_lock.h"

namespace {
SemaphoreHandle_t i2cBusMutex = nullptr;
}

bool i2cBusLock(TickType_t waitTicks) {
    if (!i2cBusMutex) i2cBusMutex = xSemaphoreCreateMutex();
    if (!i2cBusMutex) return true;
    return xSemaphoreTake(i2cBusMutex, waitTicks) == pdTRUE;
}

void i2cBusUnlock() {
    if (i2cBusMutex) xSemaphoreGive(i2cBusMutex);
}
