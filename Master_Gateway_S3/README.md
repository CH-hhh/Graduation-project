# 基于多模态大模型的桌面智能语音终端 (Master Gateway S3)

本项目是一套运行在 ESP32-S3 上的**多模态桌面智能语音终端系统**。设备通过 I2S 麦克风拾音与端侧离线声学唤醒，以阿里云通义千问全模态大模型及标准化 Function Calling（Skill）机制为认知与控制调度中枢，使用云端 TTS 流式播放回答，并具备对环境传感节点、附加智能照明与多媒体协处理器的联动控制能力。系统还提供原生 USB 超低延迟投屏功能。

## 最新特性 (v2.0)

- **多模态认知与 Skill 插件化控制**：基于 `qwen3.5-omni-plus` 实现全模态流式连续对话，注入完备工具契约（Tools Schema），支持联网检索、设备控制、音乐播控与环境感知等一系列 Skill 自适应调度。
- **纯 C++ 极速渲染引擎**：全面移除 LVGL，采用轻量级纯 C++ 原生 UI 引擎。SRAM 占用直降 150KB 以上，支持主题色、平滑字体、滚动条与多层叠屏交互。
- **20+ FPS 极速原生 USB 投屏**：利用 ESP32-S3 的 Native USB CDC，结合自定义的超高速 RLE 硬件级压缩算法与 153KB PSRAM 专属缓冲，实现 240x320 画面以极速实时投屏至电脑端。
- **异构双核 SD 卡 MP3 音乐播放器**：通过 SPI DMA 与 UART RPC 联动 STM32H7 协处理器，实现 SD 卡歌曲无感缓存与本地动态歌词滚动播放。
- **ESP-NOW 微环境无感感知与附加控制**：后台自动轮询收集 DHT 温湿度、GY-63 气压、VEML7700 光照等从机数据，支持语音分发 PWM 灯光调光指令。
- **极限 60fps 异构流媒体相机**：外挂 STM32H7 摄像头协处理器。支持实时图传与一键多模态大模型视觉拍照解答。

## 核心功能

- **语音大模型与 Skill 控制**：`qwen3.5-omni-plus` 语音与视觉理解、标准 Function Calling（Skill 工具调度）；`qwen3-tts-flash` 语音合成。
- **音频引擎**：24 kHz 单声道录音（约 22 秒缓冲）、VAD 自动停录、可打断 TTS 播放（回复限 500 字）、空闲停时钟消除底噪。
- **外设附加控制**：ESP-NOW 设备自检与发现、照明调光、微环境监测与红外学习对码。
- **系统生态**：高德 HTTPS IP 定位及四天天气；ICM42688 自动旋转屏幕、STM32 60fps 极限同步摄像头。
- **十个 UI 面板（标准循序）**：首页 (`PAGE_HOME`)、天气预报 (`PAGE_WEATHER`)、AI 助手 (`PAGE_AI`)、智能家居 (`PAGE_SMART_HOME`)、拓扑设备 (`PAGE_DEVICES`)、摄像头 (`PAGE_CAMERA`)、音乐播放器 (`PAGE_MUSIC`)、系统状态 (`PAGE_SYSTEM` - 倒数第3页)、系统设置 (`PAGE_SETTINGS` - 倒数第2页)、系统日志 (`PAGE_LOGS` - 最后一页)。
- **健壮的人机交互**：
  - 双轴模拟摇杆具备 **拔除悬空断开保护 (Fail-Safe)**、4 次 ADC 过采样滤波与自适应宽死区，彻底杜绝悬空自移或 WiFi 纹波误触。
  - 设置页面 WiFi 扫描具备专用 300ms/信道长超时状态机、SSID 跑马灯平滑滚动与 GB2312 双字库全字符渲染。
  - AI 对话卡片配备基于像素精度的多语言自动折行与严格卡片边界截断。
- **高级设置**：支持系统内动态调节屏幕亮度、休眠时长、TTS 音量，自由开关投屏模式、系统提示音和屏幕自动旋转。

## 硬件要求

- **主控**：ESP32-S3，16 MB Flash，至少 4 MB PSRAM；推荐 N16R8 模组。
- **屏幕**：ST7789 240×320 显示屏。
- **音频**：INMP441 麦克风和 NS4168 功放。
- **其他**：ICM42688 (六轴姿态传感器)、五向摇杆、旋转编码器、返回键、无源蜂鸣器（状态 LED 未使用）。

固件启动时会校验 Flash 和 PSRAM。容量不足时停止业务初始化。完整接线见 [docs/hardware.md](docs/hardware.md)。

## 私有配置

复制 `include/private_config.example.h` 为 `include/private_config.h`，填写 Wi-Fi、DashScope 和高德配置。`private_config.h` 已加入 `.gitignore`，不会进入版本库。

## 构建和体验

### 1. 编译并烧录固件
```powershell
pio run -e esp32s3box -t upload
```

### 2. 体验原生 PC 投屏
通过 Native USB 接口连接电脑，并运行提供的 Python 客户端：
```powershell
python tools/screen_mirror.py
```
> **注意**：投屏功能需在设备“系统设置”页面中确保“USB 电脑投屏”处于开启状态。

## 目录结构

```text
include/
  app_config.h             硬件和运行参数
  private_config.h         本机密钥，不提交
  custom_ui_engine.h       纯 C++ 极速 UI 引擎
  app_events.h             双核命令/UI 消息队列
  cloud_protocol.h         JSON、WAV、流式 Base64 协议
  rpc_protocol.h           ESP-NOW RPC 数据结构
  i2c_bus_lock.h           I2C 总线互斥锁
  music_player_service.h   双核异构 MP3 播放与歌单管理
src/
  main.cpp                 启动和业务编排
  custom_ui_engine.cpp     UI 渲染与动画实现
  audio_service.cpp        I2S 初始化
  imu_service.cpp          ICM42688 驱动
  buzzer_service.cpp       提示音（无源蜂鸣器）
  camera_service.cpp       STM32 相机 SPI 接收
  music_player_service.cpp 本地音乐播放服务实现
  ir_service.cpp           红外遥控与对码
  debug_log_service.cpp    环形缓冲串口日志
  i2c_bus_lock.cpp         I2C 总线互斥锁实现
  network_runtime.cpp      Wi-Fi、NTP 和自动重连
  chat_log_store.cpp       LittleFS 日志
  cloud_protocol.cpp       云端请求协议实现
  app_events.cpp           FreeRTOS 队列实现
tools/
  screen_mirror.py         Python 极速投屏客户端
```

## 系统架构与云端通信

- **双核编排**：GUI 固定在 Core 1；录音、云端请求和 ESP-NOW 业务固定在 Core 0。避免网络 IO 阻塞 UI 刷新。
- **流式音频**：音频上传按 3072 字节原始块实时编码为 Base64，不再创建完整 WAV 和 JSON 副本，极大降低内存浪涌。
- **Function Calling**：工具执行后使用 `assistant.tool_calls`、`tool_call_id` 和 `role: tool` 提交真实结果。
- **HTTPS 加密**：DashScope 和高德均使用 HTTPS 加密传输；为保证响应速度，当前固件跳过证书链校验（`setInsecure`），不校验证书。
