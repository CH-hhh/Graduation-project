# 基于多模态大模型的桌面智能语音终端设计与实现
(Desktop Multimodal Intelligent Voice Terminal)

[![PlatformIO](https://img.shields.io/badge/PlatformIO-Compatible-orange.svg)](https://platformio.org/)
[![Hardware](https://img.shields.io/badge/Hardware-ESP32--S3%20%7C%20STM32H7%20%7C%20ESP32-blue.svg)]()
[![Model](https://img.shields.io/badge/LLM-Qwen3.5--Omni--Plus-green.svg)]()

本项目是一套**基于多硬件协同与多模态大模型的桌面智能语音终端系统**。系统以大语言模型（阿里云通义千问 Qwen-Omni）云端推理与端侧 **Skill（Function Calling）插件化控制体系**为认知中枢，以端侧离线语音唤醒与高响应图形大屏为人机交互触点，融合分布式微环境感知与异构流媒体协同处理，具备连续多轮语音问答、多模态视觉拍照解答、桌面投屏与外设附加控制能力。

---

## 🏛️ 系统架构与子模块划分

```
                    ┌─────────────────────────────────────────┐
                    │   云端大模型认知大脑 & Skill 工具引擎      │
                    │   • Qwen-Omni 视觉与语音多模态大模型     │
                    │   • Function Calling 决策与 TTS 语音播报 │
                    └────────────────────▲────────────────────┘
                                         │ (WiFi HTTPS/SSE)
                                         ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│                     桌面智能语音终端主控 (ESP32-S3)                              │
│                                                                                 │
│   • 负责端侧离线语音唤醒、大模型流式对话、Skill工具调度、2.8寸屏幕10大界面绘制     │
│   • Native USB 20+ FPS 极速桌面投屏                                             │
└───────────────────────┬─────────────────────────────────┬───────────────────────┘
                        │ (20MHz SPI 高速多媒体总线)       │ (ESP-NOW 专用无线协议)
                        ▼                                 ▼
┌───────────────────────────────────────┐ ┌───────────────────────────────────────┐
│     多媒体协处理器 (STM32H743)         │ │     边缘微环境感知从节点 (ESP32)       │
│                                       │ │                                       │
│ • OV2640 摄像头多模态图像抓拍与图传   │ │ • 室内温湿度、大气压强与环境光强检测 │
│ • SD卡音频解码推流与毫秒歌词逐句同步   │ │ • 智能照明开关与 PWM 平滑调光附加执行 │
│ • 海量歌曲基于2万字词库的拼音字母排序 │ │ • 超低功耗射频间歇休眠管理            │
└───────────────────────────────────────┘ └───────────────────────────────────────┘
```

---

## 📂 项目工程目录索引

| 目录/文件 | 硬件平台/说明 | 核心职责 |
| :--- | :--- | :--- |
| [**`Master_Gateway_S3/`**](Master_Gateway_S3/) | ESP32-S3 (16MB Flash, 8MB PSRAM) | 终端主控中枢、端侧语音唤醒、大模型调用、Skill 引擎、10大卡片界面绘制 |
| [**`stm32cam/`**](stm32cam/) | STM32H743VIT6 (480MHz Cortex-M7) | 摄像头采集、视频图传推流、SD卡MP3硬件解码、歌单拼音重排与歌词下发 |
| [**`EdgeNode_A_Sensor/`**](EdgeNode_A_Sensor/) | ESP32 控制器 | 多维微环境感知（温湿度/气压/光强）、单色智能照明调光、ESP-NOW 无线组网 |
| [**`tools/`**](tools/) | PC Python 辅助开发套件 | 唤醒词采集与训练图形化工具、离线机器学习分类训练器 |
| [**`PROJECT_PROGRESS_REPORT.md`**](PROJECT_PROGRESS_REPORT.md) | 全局技术报告 | 详尽技术进展报告、系统交互时序、资源开销统计与 8 大已知工程挑战分析 |

---

## 🚀 快速上手与配置

### 1. 私有密钥配置
在 `Master_Gateway_S3/include/` 目录下，复制模板文件并填写您的 WiFi 与大模型 API 密钥：
```bash
cp Master_Gateway_S3/include/private_config.example.h Master_Gateway_S3/include/private_config.h
```
> **注意**：`private_config.h` 已被 `.gitignore` 严格忽略，绝不会上传至远端仓库。

### 2. 编译与烧录 (PlatformIO)
各子工程均基于 PlatformIO 构建，可在各自目录下直接执行编译：
```bash
# 1. 编译主控终端
cd Master_Gateway_S3
pio run -e esp32s3box -t upload

# 2. 编译环境感知从节点
cd ../EdgeNode_A_Sensor
pio run -e esp32s3 -t upload

# 3. 编译多媒体协处理器
cd ../stm32cam
pio run -e genericSTM32H743VI -t upload
```

---

## 📄 技术进展与已知问题

详细系统架构、各模块功能清单及系统当前存在且尚未解决的 8 项工程挑战（包括移动网络定位偏差、多模态音频截断模型缺陷、音乐偶发掉帧、大音量放歌唤醒限制等），请参阅完整技术文档：  
👉 [**PROJECT_PROGRESS_REPORT.md**](PROJECT_PROGRESS_REPORT.md)
