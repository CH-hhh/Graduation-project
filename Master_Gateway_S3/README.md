# ESP32-S3 终端主控工程 (Master_Gateway_S3)

本工程运行在 ESP32-S3 主控芯片上，负责语音输入采集、端侧离线唤醒、大模型流式调用与工具分发、ST7789 屏幕 UI 渲染以及外设协调控制。

---

## 硬件配置与外设引脚

* **主控芯片**：ESP32-S3-WROOM-1 (推荐 N16R8：16MB Flash, 8MB PSRAM)
* **显示屏**：2.8 寸 ST7789 彩色 LCD (240x320, 硬件 SPI 驱动)
* **音频模块**：INMP441 麦克风 (I2S 输入) + NS4168 数字功放 (I2S 输出)
* **交互输入**：双轴模拟摇杆 (ADC 采样 + 软滤波) + 旋转编码器 + 独立按键
* **其他外设**：ICM42688 六轴传感器 (I2C) + 无源蜂鸣器 + 红外收发对管

硬件引脚定义与连线细节参见 [docs/hardware.md](docs/hardware.md)。

---

## 模块架构与功能实现

### 1. 语音前端与离线唤醒 (`voice_trigger_service`)
* 采用本地轻量声学处理管线：16 通道 Mel 滤波器组进行特征提取，配合 8 阶段 DTW（动态时间规整）模板比对。
* 离线唤醒词为“你好小乐”，识别耗时在毫秒级以内，无需连接外网即可触发交互状态机。

### 2. 大模型协议与 Skill 调度 (`cloud_protocol`, `device_action_service`)
* 基于 WiFi HTTPS 连接阿里云 DashScope，以 SSE 流式解析通义千问模型回复。
* 采用标准化 Function Calling 规范向模型注入 Tools Schema：
  * 天气查询：结合高德 IP 定位与天气 API 获取多日预报；
  * 音乐播控：解析点歌词条并向下游协处理器分发播控指令；
  * 环境感知：通过 ESP-NOW 向上位机上报温度、湿度、气压与光照数值；
  * 设备控制：动态生成调光 PWM 占空比并广播给从机节点。

### 3. 轻量化 UI 引擎 (`custom_ui_engine`)
* 自行实现纯 C++ 基础绘图与卡片排版逻辑，避免通用 GUI 框架占用过量 SRAM（节省约 150KB 内存）。
* 系统包含 10 个功能界面：首页、天气预报、AI 问答卡片、智能家居控制、设备拓扑拓扑、摄像头监视、音乐播放、系统状态、系统设置、运行日志。
* 对话文本支持基于像素测宽的中文自动折行与平滑滚动。

### 4. 原生 USB 投屏 (`screen_mirror.py`)
* 基于 ESP32-S3 原生 USB CDC 接口传输屏幕帧缓冲。
* 采用自定义 RLE 行程压缩，配合上位机 Python 脚本实现 20+ FPS 电脑端实时画面投屏。

---

## 本地编译与烧录

基于 PlatformIO 环境开发：

```bash
# 复制并配置密钥
cp include/private_config.example.h include/private_config.h

# 编译并烧录至 ESP32-S3
pio run -e esp32s3box -t upload

# 打开串口监视器
pio device monitor -b 115200
```
