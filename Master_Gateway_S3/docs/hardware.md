# 硬件接线与协议

## GPIO 分配

| 功能 | GPIO | 说明 |
|---|---:|---|
| TFT BLK | 9 | 屏幕背光控制 |
| TFT SC(DC) | 10 | 数据/命令选择 |
| TFT SDO(MISO)| 11 | 屏幕总线数据输入 |
| TFT CS | 12 | 屏幕片选 |
| TFT SCL | 13 | ST7789 SPI 时钟 |
| TFT SDI(MOSI)| 14 | ST7789 SPI 数据输出 |
| 摇杆 VRX | 2 | ADC 输入 |
| 摇杆 VRY | 1 | ADC 输入 |
| 摇杆 SW | 42 | 低电平按下 |
| 返回键 | 18 | 低电平按下 |
| 旋转编码器 A / B / 按键 | 3 / 17 / 8 | EC11 正交解码 |
| 蜂鸣器 I/O | 15 | 无源蜂鸣器（LEDC 驱动） |
| 麦克风 SD | 4 | INMP441 麦克风数据 |
| 共用 BCLK(SCK) | 5 | 麦克风与功放共享 |
| 共用 LRCK(WS) | 6 | 麦克风与功放共享 |
| 功放 DATA | 7 | 喇叭功放音频输出 |
| IMU SDA | 41 | ICM42688 姿态仪 I2C 数据 |
| IMU SCL | 40 | ICM42688 姿态仪 I2C 时钟 |
| 红外 OUT | 16 | IR 遥控器数据接收 |
| STM32 相机 CS / SCK / MISO / MOSI | 47 / 38 / 39 / 45 | 与 STM32H743 SPI 通信 |
| STM32 相机 HANDSHAKE | 21 | 请求发送握手信号 |
| 状态 LED | 未使用 | 原 GPIO 39 已让给相机 SPI MISO |

TFT `RST=-1`，屏幕复位应连接 ESP32 的 EN、3.3V 或按屏幕模块设计处理。ICM42688 使用硬件 I2C 总线（SDA=41, SCL=40），地址为 `0x68`/`0x69`（开机自动扫描）。GPIO 8 为旋转编码器按键，GPIO 9 为屏幕背光；状态 LED 未使用（GPIO 39 已让给 STM32 相机 SPI MISO）。

### 物理五向双轴摇杆电气接线与保护须知
1. **供电电压（严禁接 5V）**：
   - 摇杆模块 VCC **必须接 3.3V**（切勿连接 5V 供电）。
   - 若误接 5V，静止中位分压达 2.5V，在 3.3V 满量程的 ESP32 ADC 上读数高达 ~3100（超过向右判定死区 3000），会导致静止时持续疯狂向右自动走位。
2. **共地极性（严禁 GND / VCC 反接）**：
   - 模块 GND 必须良好连接 ESP32 GND。
   - 若反接，会导致电位器两端电平颠倒（左右方向完全反向），且板载轻触按键 SW 将被短接到 3.3V，导致内部上拉引脚（GPIO 42）电平永远无法被拉低，造成“按键下压毫无反应”。
3. **固件级 Fail-Safe 悬空断开保护**：
   - 固件在 `main.cpp` 中已集成模拟引脚断开防护与 4 次过采样均值滤波。当摇杆被物理拔掉时，双轴 ADC 读数同时跌落接近 0V，系统自动识别为“未接入设备”并进行静默阻断，彻底杜绝悬空引起的无限向左连发卡死。

## ESP-NOW RPC

主网关使用广播 MAC 发送七字节命令：

```cpp
struct RpcCommand {
    uint8_t device_id;
    uint8_t action;
    uint8_t value;
    uint8_t target_led;
    uint8_t hue;
    uint8_t speed;
    uint8_t brightness;
};
```

当前设备 ID 为 `2`：

| action | 含义 | value |
|---:|---|---|
| `0x00` | 设备探测 | `0` |
| `0x01` | 单色智能灯（PWM） | 亮度 `0~100`（`≤20` 视为关灯） |
| `0x03` | 距离读取 | `0` |
| `0x04` | 环境读取（温度/气压） | `0` |
| `0x05` | 外置温湿度读取 | `0` |
| `0x06` | 环境光照读取 | `0` |

从设备回传十二字节确认包：

```cpp
struct RpcAcknowledgement {
    uint8_t device_id;
    uint8_t is_success;
    float sensor_value;
    float sensor_value2;
};
```

`is_success == 1` 表示灯光动作执行成功，`3~6` 分别表示距离、环境（气压/温度）、温湿度、光照数据（第二个传感器值放在 `sensor_value2`）。设备探测应答使用 `status=0` 且 `sensor_value` 为传感器在线掩码。ESP-NOW 与 Wi-Fi 共用射频信道，从设备必须与主网关当前信道一致。
