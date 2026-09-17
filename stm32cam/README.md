# STM32 极限性能摄像头协处理器 (stm32cam)

本项目是 AI 智能家居系统的专属高速摄像头协处理器固件，运行在 **STM32H743** 核心板上。
为了分担 ESP32-S3 主网关的视频捕获压力，STM32 承担了 OV2640 的底层驱动、DCMI 并行数据捕获、RGB565 像素端序翻转、以及 40MHz 高速 SPI 封包推流的繁重任务。

## 最新特性：60fps 极限推流架构

为了打破常规开源驱动 15~30fps 的性能天花板，本固件进行了深度的极限榨能重构：

- **硬件级乒乓双缓冲 (Ping-Pong Buffer)**
  在 SRAM1 中分配了整整 2 帧 (76.8KB) 的连续 DMA 缓冲区。开启 DCMI 的 Continuous Mode 后，摄像头硬件将不间断地在后台交替填充这两块内存区，实现 100% 零丢包捕获。
- **极致的零中断轮询**
  主循环 (`loop()`) 摒弃了传统的延迟等待和中断开销，采用极客级的裸机寄存器轮询——死死盯住 `DMA1_Stream1->NDTR` 寄存器。一旦判断 DMA 正在写入前半区，CPU 立即处理后半区的数据并瞬间推上 SPI 总线。
- **OV2640 60fps 硬件级超频**
  突破了标准初始化的 30fps 限制。通过将内部 `CLKRC (0x11)` 时钟寄存器设置为 `0x80`，强制开启摄像头自带的 PLL 内部时钟倍频器 (Double Clock)，使物理帧率飙升至 60fps。
- **SPI 总线毫秒级穿插控制**
  系统采用 40MHz 超高速 SPI。在两帧传输的极短微秒间隙内，依然保留了对 `HANDSHAKE` 引脚的高频检测逻辑，允许主控 ESP32 随时随地插入 2ms 的短脉冲，实现了**推流期间无缝闪光灯控制**的惊艳效果。

## 引脚定义

STM32 端的外设连接引脚如下：

### OV2640 摄像头 (DCMI 接口)
| OV2640 | STM32H7 | 说明 |
| --- | --- | --- |
| D0~D7 | PC6~PC11, PE5, PE6 | 8 位并行数据总线 |
| PCLK | PA6 | 像素同步时钟 |
| HSYNC | PA4 | 行同步信号 |
| VSYNC | PB7 | 帧同步信号 |
| SCL/SDA | PB8/PB9 | SCCB (I2C) 配置总线 |
| RESET / PWDN | PE3 / PE2 | 硬件复位与低功耗控制 |
| XCLK | - | 模组自带 24MHz 有源晶振，无需 STM32 驱动 |

### 主机通信与外设 (SPI 接口)
| 功能 | 引脚 | 说明 |
| --- | --- | --- |
| SPI_SCK | PA9 | 40MHz 极速 SPI 时钟 (从主控来) |
| SPI_MISO | PA10 | 像素数据发送至 ESP32 |
| SPI_NSS | PA15 | 片选信号 (主控发起) |
| HANDSHAKE | PA12 | **核心同步线**。ESP32 用于长脉冲推流请求与短脉冲闪光灯控制 |
| 闪光灯 LED | PA5 | OV2640 模组自带高亮 LED |

## 通信协议与时序

STM32 作为 SPI 发送端，每帧传输一个 `38412` 字节的完整数据包，其结构如下：
```cpp
struct __attribute__((packed)) ImagePacket {
    uint32_t header_magic;   // 固定 0xDEADBEEF
    uint32_t payload_size;   // 固定 38400
    uint16_t pixels[19200];  // 160x120 RGB565 数据
    uint32_t tail_magic;     // 固定 0xFEEDFACE
};
```

1. ESP32-S3 触发底层的 `post_setup_cb` 将 `HANDSHAKE` 引脚精准拉高。
2. STM32 极速轮询检测到高电平后，开始等待 DMA 半区就绪。
3. DMA 半区就绪后，STM32 对 19200 个像素进行高位与低位的端序交换（适配 Little-Endian 屏幕），然后调用 `SPI.transfer()` 传出 38412 字节。
4. 传输完成的瞬间，STM32 会启动极速微秒级死循环阻塞，**直到 ESP32 撤销 (拉低) `HANDSHAKE` 信号**，才结束本轮循环，从而完美避开了连续高频中断重入所引发的误判。

## 构建与烧录

本项目使用 PlatformIO 管理，框架为 Arduino on STM32。

```bash
# 进入目录
cd stm32cam

# 编译并烧录
pio run -t upload
```
烧录后，若系统初始化正常，串口 (115200) 将打印出：
```text
[Init] SCCB 总线启动
[Init] OV2640 硬件复位完毕...
...
[Init] DCMI + DMA 硬件级图像采集已就绪！
```
