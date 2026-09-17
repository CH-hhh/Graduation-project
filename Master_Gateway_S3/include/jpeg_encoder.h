#pragma once

#include <Arduino.h>
#include <vector>

namespace JpegEncoder {

/**
 * @brief 将 RGB565 像素缓冲区编码为标准 JPEG 格式二进制数据
 * @param rgb565 输入的 16 位 RGB565 像素缓冲区
 * @param width 图像宽度 (例如 160)
 * @param height 图像高度 (例如 120)
 * @param quality JPEG 图像质量 (1-100, 推荐 75-85)
 * @param out_jpeg 存放输出 JPEG 二进制数据的动态字节向量
 * @return 编码成功返回 true，否则返回 false
 */
bool encodeRgb565(const uint16_t* rgb565, int width, int height, int quality, std::vector<uint8_t>& out_jpeg);

} // namespace JpegEncoder
