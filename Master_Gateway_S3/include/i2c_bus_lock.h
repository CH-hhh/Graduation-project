#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// 保护全局 Wire 总线：IMU 与罗盘分属不同任务，必须串行化 I2C 事务。
bool i2cBusLock(TickType_t waitTicks = portMAX_DELAY);
void i2cBusUnlock();
