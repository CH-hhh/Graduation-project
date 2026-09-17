#pragma once

#include <Arduino.h>

// OV2640 SCCB (I2C) 默认地址 0x30 (7位地址)
constexpr uint8_t OV2640_SCCB_ADDR = 0x30;

// OV2640 寄存器结构体
struct SensorReg {
    uint8_t reg;
    uint8_t val;
};

// OV2640 基础初始化寄存器
extern const SensorReg OV2640_INIT_REGS[];
extern const size_t OV2640_INIT_REGS_SIZE;

// OV2640 全视野 CIF 60fps QQVGA (160x120) 组合配置矩阵
extern const SensorReg OV2640_QQVGA_60FPS_REGS[];
extern const size_t OV2640_QQVGA_60FPS_REGS_SIZE;

// OV2640 RGB565 配置 (供 ImageWin 和 OutSize 使用)
extern const SensorReg OV2640_RGB565_REGS[];
extern const size_t OV2640_RGB565_REGS_SIZE;

// OV2640 YUV422 配置 (JPEG 引擎的前置输入)
extern const SensorReg OV2640_YUV422_REGS[];
extern const size_t OV2640_YUV422_REGS_SIZE;

// OV2640 JPEG 配置
extern const SensorReg OV2640_JPEG_REGS[];
extern const size_t OV2640_JPEG_REGS_SIZE;

// 窗口设置
uint8_t OV2640_ImageWin_Set(uint16_t offx, uint16_t offy, uint16_t width, uint16_t height);
uint8_t OV2640_OutSize_Set(uint16_t width, uint16_t height);
