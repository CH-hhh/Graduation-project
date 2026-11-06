# STM32H743 媒体协处理器固件 (stm32cam)

本固件运行于 STM32H743VIT6 核心板上，主要承担系统中的图像采集、SD 卡文件系统读取以及 MP3 音频软解码任务，并通过 SPI 高速总线将媒体流推送至 ESP32-S3 主控端。

---

## 硬件外设连接

### 1. OV2640 摄像头 (DCMI 并行接口)
| OV2640 引脚 | STM32H743 引脚 | 功能说明 |
| :--- | :--- | :--- |
| D0 ~ D7 | PC6 ~ PC11, PE5, PE6 | 8 位并行数据总线 |
| PCLK | PA6 | 像素同步时钟 |
| HSYNC | PA4 | 行同步信号 |
| VSYNC | PB7 | 帧同步信号 |
| SCL / SDA | PB8 / PB9 | SCCB (I2C) 传感器配置接口 |
| RESET / PWDN | PE3 / PE2 | 硬件复位与电源控制 |

### 2. 主机通信接口 (SPI Slave & UART RPC)
| 信号线 | 引脚 | 说明 |
| :--- | :--- | :--- |
| SPI_SCK | PA9 | SPI 从机时钟输入 (由 ESP32 主机提供，最高 20MHz) |
| SPI_MISO | PA10 | 图像与音频数据上行发送 |
| SPI_NSS | PA15 | 片选信号 (低电平有效) |
| HANDSHAKE | PA12 | 状态同步与长脉冲推流请求引脚 |
| UART_TX / RX | USART1 | 与主控进行播控指令交互与 LRC 歌词同步 (115200 bps) |

---

## 核心技术实现

### 1. DCMI 连续捕获与 DMA 双缓冲
* 在 SRAM 中分配两块连续的 DMA 帧缓冲区构建乒乓缓冲（Ping-Pong Buffer）。
* DCMI 配置为连续捕获模式，DMA1 自动交替填充缓冲区块，主循环中检测 `NDTR` 计数器状态实现无等待零拷贝推流，避免图像采集掉帧。

### 2. SDMMC 与 Helix MP3 软解码
* 采用硬件 SDMMC 4-bit 模式驱动 MicroSD 卡，挂载 FatFs 文件系统。
* 集成 Helix MP3 定点数解码算法库，将 MP3 音频流实时解码为 16-bit 44.1kHz 双声道 PCM 数据，经由 SPI DMA 推送至主控。
* 扫描 SD 卡目录时读取歌曲同名 `.lrc` 文件，提取时间戳后按播放进度逐行推送给上位机。

### 3. 歌单汉字拼音字母序重排
* 内置约两万字的常用汉字拼音映射索引表。
* 遍历 SD 卡曲目列表时，将中文歌名自动转译为拼音字母，并在微控制器内存中执行快速排序，输出 A-Z 严格字母序索引。

---

## 构建方式

项目基于 PlatformIO 开发：

```bash
# 编译固件
pio run -e genericSTM32H743VI

# 烧录至开发板 (通过 ST-Link)
pio run -e genericSTM32H743VI -t upload
```
