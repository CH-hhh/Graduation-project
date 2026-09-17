/**
 * =================================================================================
 * STM32H743 DCMI + DMA 硬件级图像采集 → SPI 推流 (stm32cam)
 * 
 * 【根因修复】之前手动轮询 DCMI->DR 读像素，但 DCMI 内部 FIFO 仅 4 个字深
 * (16 字节)，在 PCLK 数兆赫的速率下，FIFO 在几微秒内就会溢出！
 * 必须使用 DMA 硬件通道，让 DMA 控制器以总线速度自动将 DCMI FIFO 中的
 * 像素搬运到内存缓冲区，CPU 完全不参与逐像素搬运。
 *
 * 参考: E:\VScode\h743cam\src\HARDWARE\DCMI\dcmi.c (DevEBox 成功代码)
 * =================================================================================
 */

/**
 * ============================================================================
 * [WORKING VERSION: DCMI+DMA HARDWARE CAPTURE]
 * - 摄像头: OV2640 (硬件 I2C/SCCB 极性修正, HSPOL=LOW, VSPOL=LOW)
 * - 分辨率: 底层 SVGA(800x600) -> DSP 硬件缩放 QQVGA(160x120)
 * - 内存: D2 SRAM1 (0x30000000) 完美映射 DMA
 * - 传输: 5线硬件 SPI 发送，带 RGB565 大小端(Endian)翻转修复 (解决异色问题)
 * ============================================================================
 */
#include <Arduino.h>
#include <SPI.h>
#include "ov2640.h"
#include "mp3_player_backend.h"

// ==================== SPI2 传输接口 (5线) ====================
constexpr int MY_PIN_HANDSHAKE = PB1;
constexpr int PIN_FLASH_LED    = PA5;
constexpr int MY_PIN_SPI_NSS   = PB12;
constexpr int MY_PIN_SPI_SCK   = PB13;
constexpr int MY_PIN_SPI_MISO  = PB14;
constexpr int MY_PIN_SPI_MOSI  = PB15;

// ==================== 原理图 J6 物理映射 ====================
constexpr int PIN_DCMI_SDA   = PB11;
constexpr int PIN_DCMI_SCL   = PB10;
constexpr int PIN_DCMI_RESET = PC4;
constexpr int PIN_DCMI_PWDN  = PA7;

// ==================== 画面参数 ====================
constexpr uint16_t CAM_W = 160;
constexpr uint16_t CAM_H = 120;

constexpr uint32_t PKT_HEADER_MAGIC = 0xDEADBEEF; // 包头
constexpr uint32_t PKT_TAIL_MAGIC   = 0xFEEDFACE; // 包尾

struct __attribute__((packed)) ImagePacket {
    uint32_t header_magic;
    uint32_t payload_size;
    uint16_t pixels[CAM_W * CAM_H];
    uint32_t tail_magic;
};

ImagePacket tx_packet;

struct __attribute__((packed)) CmdPacket {
    uint8_t cmd_type; // 1 = STREAM, 2 = FLASH_TOGGLE
    uint8_t flash_on; // 0 = OFF, 1 = ON
    uint16_t reserved;
};

CmdPacket rx_cmd = {1, 0, 0};
uint8_t g_last_flash_state = 0;

uint32_t g_frame_counter = 0;
bool g_camera_hardware_ready = false;

// ==================== DMA 接收缓冲区 ====================
// 【终极修复】DMA1 无法访问 D1 AXI SRAM (0x24000000)
// 将 DMA 缓冲区强制分配到 D2 域的 SRAM1 (0x30000000)
static uint16_t* dcmi_dma_buf = (uint16_t*)0x30000000;

// ==================== SPI2 TX DMA (Bare-Metal) ====================
void initSpiDma() {
    __HAL_RCC_DMA1_CLK_ENABLE();
    
    // Disable DMA1_Stream2
    DMA1_Stream2->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream2->CR & DMA_SxCR_EN);
    
    // Configure DMAMUX for SPI2_TX (Request 40)
    DMAMUX1_Channel2->CCR = 40;

    // Configure DMA CR:
    // DIR = 01 (Memory to Peripheral)
    // MINC = 1 (Memory increment)
    // PINC = 0
    // MSIZE = 00 (8-bit), PSIZE = 00 (8-bit)
    // PRIO = 10 (High)
    DMA1_Stream2->CR = (1 << 6) | (1 << 10) | (2 << 16);
    
    // Clear all interrupt flags for Stream 2 (bits 16-21)
    DMA1->LIFCR = 0x3F << 16;
}

void startSpiDma(void* data, size_t size) {
    digitalWrite(MY_PIN_SPI_NSS, LOW);
    
    SPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
    
    // Configure DMA
    DMA1_Stream2->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream2->CR & DMA_SxCR_EN);
    DMA1->LIFCR = 0x3F << 16; // Clear flags
    
    DMA1_Stream2->PAR = (uint32_t)&SPI2->TXDR;
    DMA1_Stream2->M0AR = (uint32_t)data;
    DMA1_Stream2->NDTR = size;
    
    // Enable SPI TX DMA
    SPI2->CFG1 |= SPI_CFG1_TXDMAEN;
    
    // Set TSIZE in SPI_CR2
    SPI2->CR2 = size;
    
    // Enable DMA
    DMA1_Stream2->CR |= DMA_SxCR_EN;
    
    // Start SPI Transfer
    SPI2->CR1 |= SPI_CR1_CSTART;
}

void waitAndFinishSpiDma() {
    // Wait for DMA Transfer Complete (TCIF2 is bit 21 in LISR)
    while (!(DMA1->LISR & (1 << 21)));
    
    // Wait for SPI TX Complete (TXC = bit 12 in SPI_SR)
    while (!(SPI2->SR & (1 << 12)));
    
    // Clear SPI EOT (bit 3) and TXTF (bit 11) flags to allow next transaction
    SPI2->IFCR = (1 << 3) | (1 << 11);
    
    // Clean up
    SPI2->CFG1 &= ~SPI_CFG1_TXDMAEN;
    SPI2->CR2 = 0; // Reset TSIZE
    
    SPI.endTransaction();
    digitalWrite(MY_PIN_SPI_NSS, HIGH);
}


// ==================== HAL DMA 句柄 ====================
static DMA_HandleTypeDef hdma_dcmi;

// ==========================================================================
// 1. 配置 DCMI 数据/同步引脚 (AF13)
//    完全对照 E:\VScode\h743cam 参考代码的引脚表
// ==========================================================================
void initDcmiGpioPins() {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();  // 启用 GPIOD 时钟 (D5 = PD3)
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_DCMI_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();  // DMA1 时钟！
    
    // 【关键】必须确保 D2 SRAM1 的时钟被开启，否则 DMA1 写入 SRAM1 会被总线丢弃！
    __HAL_RCC_D2SRAM1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;          // 参考代码用 PULLUP
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;

    // PA4 (HSYNC/HREF), PA6 (PCLK)
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_6;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PB7 (VSYNC)
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // PC6 (D0), PC7 (D1)
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // 【修正】根据参考代码，D5 是 PD3，不是 PB6！
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    // PE0 (D2), PE1 (D3), PE4 (D4), PE5 (D6), PE6 (D7)
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

// ==========================================================================
// 2. 初始化 DMA1_Stream1 用于 DCMI 数据搬运
//    完全对照参考代码 DCMI_DMA_Init() 的配置
// ==========================================================================
void initDmaForDcmi() {
    // 先停掉 DMA1_Stream1
    DMA1_Stream1->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream1->CR & DMA_SxCR_EN) {}

    hdma_dcmi.Instance                 = DMA1_Stream1;
    hdma_dcmi.Init.Request             = 75;  // DMA_REQUEST_DCMI = 75
    hdma_dcmi.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_dcmi.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_dcmi.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_dcmi.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;   // 32位
    hdma_dcmi.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;   // 32位
    hdma_dcmi.Init.Mode                = DMA_CIRCULAR;          // 循环模式
    hdma_dcmi.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
    hdma_dcmi.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
    hdma_dcmi.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_dcmi.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_dcmi.Init.PeriphBurst         = DMA_PBURST_SINGLE;

    HAL_DMA_DeInit(&hdma_dcmi);
    HAL_StatusTypeDef dma_status = HAL_DMA_Init(&hdma_dcmi);

    Serial.print("🔧 [DMA Init] HAL_DMA_Init 返回: ");
    Serial.println(dma_status == HAL_OK ? "OK" : "FAIL!");

    // 关键！参考代码: "在开启DMA之前必须使用__HAL_UNLOCK()先解锁一下DMA"
    __HAL_UNLOCK(&hdma_dcmi);

    // 【双缓冲修改】DMA缓冲区扩大一倍，传输长度翻倍
    HAL_StatusTypeDef start_status = HAL_DMA_Start(&hdma_dcmi, 
        (uint32_t)&DCMI->DR, 
        (uint32_t)dcmi_dma_buf, 
        (CAM_W * CAM_H)); // 每帧是 (CAM_W*CAM_H)/2 个 WORD。双缓冲所以是 CAM_W * CAM_H

    Serial.print("🔧 [DMA Start] HAL_DMA_Start 返回: ");
    Serial.println(start_status == HAL_OK ? "OK" : "FAIL!");

    // 诊断输出
    Serial.print("🔧 [DMA Diag] CR=0x");
    Serial.print(DMA1_Stream1->CR, HEX);
    Serial.print(" PAR=0x");
    Serial.print(DMA1_Stream1->PAR, HEX);
    Serial.print(" M0AR=0x");
    Serial.print(DMA1_Stream1->M0AR, HEX);
    Serial.print(" NDTR=");
    Serial.println(DMA1_Stream1->NDTR);
    Serial.print("🔧 [DMAMUX] Ch1_CCR=0x");
    Serial.println(DMAMUX1_Channel1->CCR, HEX);
}

// ==========================================================================
// 3. 初始化 DCMI 外设寄存器
//    极性完全对照参考代码: VSPOL=LOW, HSPOL=LOW, PCKPOL=RISING
// ==========================================================================
void initDcmiPeripheral() {
    DCMI->CR = 0;
    // 参考代码配置 (必须与 ov2640cfg.h 的数组配合):
    // PCKPolarity=RISING, VSPolarity=LOW, HSPolarity=LOW, ExtendedDataMode=8B
    DCMI->CR = (1 << 5);  // PCKPOL=Rising, HSPOL=LOW(0), VSPOL=LOW(0)
    DCMI->IER = 0x0;

    // 初始化时直接开启连续捕获模式，不再干预
    DCMI->CR |= DCMI_CR_ENABLE; // 必须先使能，否则 CAPTURE 位无效！
    DCMI->CR &= ~DCMI_CR_CM; // CM=0 Continuous Mode
    DCMI->CR |= DCMI_CR_CAPTURE;
}

// 4. 解复位并唤醒摄像头
// ==========================================================================
void powerAndResetCameraSequence() {
    pinMode(PIN_DCMI_PWDN, OUTPUT);
    digitalWrite(PIN_DCMI_PWDN, LOW);
    delay(20);

    pinMode(PIN_DCMI_RESET, OUTPUT);
    digitalWrite(PIN_DCMI_RESET, LOW);
    delay(20);
    digitalWrite(PIN_DCMI_RESET, HIGH);
    delay(50);
}

// ==========================================================================
// 5. 纯物理 Bit-Bang SCCB 驱动 (已验证 PID=0x26)
// ==========================================================================
static inline void sccbDelay() { delayMicroseconds(10); }
static void sccbSdaOutput() { pinMode(PIN_DCMI_SDA, OUTPUT_OPEN_DRAIN); }
static void sccbSdaInput()  { pinMode(PIN_DCMI_SDA, INPUT_PULLUP); }

static void sccbStart() {
    sccbSdaOutput();
    digitalWrite(PIN_DCMI_SDA, HIGH);
    digitalWrite(PIN_DCMI_SCL, HIGH);
    sccbDelay();
    digitalWrite(PIN_DCMI_SDA, LOW);
    sccbDelay();
    digitalWrite(PIN_DCMI_SCL, LOW);
    sccbDelay();
}

static void sccbStop() {
    sccbSdaOutput();
    digitalWrite(PIN_DCMI_SDA, LOW);
    digitalWrite(PIN_DCMI_SCL, HIGH);
    sccbDelay();
    digitalWrite(PIN_DCMI_SDA, HIGH);
    sccbDelay();
}

static void sccbSendByte(uint8_t data) {
    sccbSdaOutput();
    for (int i = 7; i >= 0; i--) {
        digitalWrite(PIN_DCMI_SDA, (data >> i) & 0x01 ? HIGH : LOW);
        sccbDelay();
        digitalWrite(PIN_DCMI_SCL, HIGH);
        sccbDelay();
        digitalWrite(PIN_DCMI_SCL, LOW);
        sccbDelay();
    }
    sccbSdaInput();
    sccbDelay();
    digitalWrite(PIN_DCMI_SCL, HIGH);
    sccbDelay();
    digitalWrite(PIN_DCMI_SCL, LOW);
    sccbDelay();
}

static uint8_t sccbReceiveByte() {
    uint8_t val = 0;
    sccbSdaInput();
    for (int i = 7; i >= 0; i--) {
        digitalWrite(PIN_DCMI_SCL, HIGH);
        sccbDelay();
        if (digitalRead(PIN_DCMI_SDA) == HIGH) {
            val |= (1 << i);
        }
        digitalWrite(PIN_DCMI_SCL, LOW);
        sccbDelay();
    }
    sccbSdaOutput();
    digitalWrite(PIN_DCMI_SDA, HIGH);
    sccbDelay();
    digitalWrite(PIN_DCMI_SCL, HIGH);
    sccbDelay();
    digitalWrite(PIN_DCMI_SCL, LOW);
    sccbDelay();
    return val;
}

bool rawSccbWriteReg(uint8_t slave_addr, uint8_t reg, uint8_t data) {
    sccbStart();
    sccbSendByte(slave_addr << 1);
    sccbSendByte(reg);
    sccbSendByte(data);
    sccbStop();
    return true;
}

uint8_t rawSccbReadReg(uint8_t slave_addr, uint8_t reg) {
    uint8_t val = 0xFF;
    sccbStart();
    sccbSendByte(slave_addr << 1);
    sccbSendByte(reg);
    sccbStop();
    sccbDelay();
    sccbStart();
    sccbSendByte((slave_addr << 1) | 0x01);
    val = sccbReceiveByte();
    sccbStop();
    return val;
}

bool rawSccbWriteArray(uint8_t slave_addr, const SensorReg* reg_array, size_t size) {
    for (size_t i = 0; i < size; i++) {
        rawSccbWriteReg(slave_addr, reg_array[i].reg, reg_array[i].val);
        delayMicroseconds(50);
    }
    return true;
}

// ==========================================================================
// 6. OV2640 完整初始化 + DCMI + DMA 一条龙启动
// ==========================================================================
bool initSchematicOv2640() {
    // 6.1 配置所有 DCMI GPIO 引脚
    initDcmiGpioPins();

    // 6.2 不启动 MCO！参考代码也不启动！板载/模组自带 24MHz 有源晶振。
    //     之前强行开 MCO 导致两个时钟源短路打架！

    // 6.3 上电复位序列
    powerAndResetCameraSequence();

    // 6.4 配置 SCCB (I2C) 引脚
    pinMode(PIN_DCMI_SCL, OUTPUT_OPEN_DRAIN);
    pinMode(PIN_DCMI_SDA, INPUT_PULLUP);
    digitalWrite(PIN_DCMI_SCL, HIGH);
    digitalWrite(PIN_DCMI_SDA, HIGH);
    delay(50);

    // 6.5 读取芯片 ID
    rawSccbWriteReg(0x30, 0xFF, 0x01);
    delay(20);
    uint8_t pid = rawSccbReadReg(0x30, 0x0A);
    uint8_t ver = rawSccbReadReg(0x30, 0x0B);

    Serial.print("🎉 [硬件检测] PID: 0x");
    if (pid < 16) Serial.print("0");
    Serial.print(pid, HEX);
    Serial.print(", VER: 0x");
    if (ver < 16) Serial.print("0");
    Serial.println(ver, HEX);

    if (pid != 0x26) {
        Serial.println("❌ OV2640 未检测到！");
        return false;
    }

    Serial.println("🚀 OV2640 芯片已激活！正在写入寄存器...");

    // 6.6 写入由用户提供的官方 60fps 完整初始化序列 (包含 CIF扫描、DSP 缩放、时钟配置、RGB565 输出)
    // 增加软件复位，防止多次烧录或断电残留导致状态机混乱
    rawSccbWriteReg(0x30, 0xFF, 0x01); // 切换到 Sensor Bank
    rawSccbWriteReg(0x30, 0x12, 0x80); // 软件复位
    delay(20);
    
    rawSccbWriteArray(0x30, OV2640_QQVGA_60FPS_REGS, OV2640_QQVGA_60FPS_REGS_SIZE);
    delay(50);

    // 6.7 初始化 DCMI 外设寄存器 (极性配置)
    initDcmiPeripheral();

    // 6.8 初始化 DMA1_Stream1 用于 DCMI 数据搬运
    initDmaForDcmi();

    Serial.println("✅ DCMI + DMA 硬件级图像采集已就绪！");
    return true;
}

// ==========================================================================
// 8. setup()
// ==========================================================================
void setup() {
    Serial.begin(115200);
    delay(1500);

    // 【诊断】关闭 D-Cache，彻底排除缓存一致性问题
    SCB_DisableDCache();

    Serial.println("\n==========================================================");
    Serial.println("  🎥 STM32H743 DCMI+DMA 硬件级真实画面采集 (stm32cam)");
    Serial.println("  [DIAG] D-Cache DISABLED");
    Serial.println("==========================================================");

    // SPI 和握手引脚
    pinMode(MY_PIN_HANDSHAKE, INPUT_PULLDOWN);
    pinMode(PIN_FLASH_LED, OUTPUT);
    digitalWrite(PIN_FLASH_LED, LOW);
    pinMode(MY_PIN_SPI_NSS, OUTPUT);
    digitalWrite(MY_PIN_SPI_NSS, HIGH);

    SPI.setMOSI(MY_PIN_SPI_MOSI);
    SPI.setMISO(MY_PIN_SPI_MISO);
    SPI.setSCLK(MY_PIN_SPI_SCK);
    SPI.begin();

    // OV2640 + DCMI + DMA 全量初始化
    g_camera_hardware_ready = initSchematicOv2640();
    
    // 初始化 SPI DMA
    initSpiDma();

    // 发送包头尾
    tx_packet.header_magic = PKT_HEADER_MAGIC;
    tx_packet.payload_size = sizeof(tx_packet.pixels);
    tx_packet.tail_magic   = PKT_TAIL_MAGIC;

    // 彻底清空内存缓冲区，确保纯净状态
    memset((void*)tx_packet.pixels, 0, sizeof(tx_packet.pixels));
    memset((void*)dcmi_dma_buf, 0, CAM_W * CAM_H * 2 * sizeof(uint16_t));

    // 初始化 MP3 播放与 USART2 控制后端
    Mp3PlayerBackend::begin();

    Serial.println("==========================================================");
    if (g_camera_hardware_ready) {
        Serial.println("✅ [系统就绪] DCMI+DMA 硬件采集推流中...\n");
    } else {
        Serial.println("❌ [系统异常] 摄像头初始化失败！\n");
    }
}

bool g_camera_streaming = false;

// ==========================================================================
// 9. loop()
// ==========================================================================
void loop() {
    // 🌟 1. 后端 MP3 媒体服务与 USART2 RPC 通信轮询
    Mp3PlayerBackend::update();

    // 🌟 2. 严格 SPI 互斥：只有当已下发 CAM_START 且当前未在播放 MP3 音频时，才允许运行摄像头采集与推流
    if (!g_camera_streaming || Mp3PlayerBackend::getPlayState() == Mp3PlayerBackend::STATE_PLAYING) {
        return;
    }

    static uint32_t last_fps_time = 0;
    static uint32_t frames_captured_sec = 0;
    static uint32_t frames_tx_sec = 0;
    static int last_processed_half = -1; // 0=前一半, 1=后一半

    if (millis() - last_fps_time >= 1000) {
        if (last_fps_time != 0 && (frames_captured_sec > 0 || frames_tx_sec > 0)) {
            Serial.printf("[STM32-FPS] 采集帧率: %lu fps | 推送帧率: %lu fps\r\n", frames_captured_sec, frames_tx_sec);
        }
        frames_captured_sec = 0;
        frames_tx_sec = 0;
        last_fps_time = millis();
    }

    if (digitalRead(MY_PIN_HANDSHAKE) == HIGH) {
        // 轮询等待下一半缓冲区准备就绪 (握手线仅作为 DMA 就绪指示)
        uint32_t ndtr;
        int ready_half;
        
        while (true) {
            ndtr = DMA1_Stream1->NDTR;
            int current_writing_half = (ndtr <= (CAM_W * CAM_H / 2)) ? 1 : 0;
            ready_half = 1 - current_writing_half;
            
            if (ready_half != last_processed_half && digitalRead(MY_PIN_HANDSHAKE) == HIGH) {
                break;
            }
            if (digitalRead(MY_PIN_HANDSHAKE) == LOW) {
                return;
            }
            // 硬件自愈：清理 DCMI 偶发错误标志 (OVR_RIS / ERR_RIS)，防止 DCMI 硬件闭锁
            if (DCMI->MISR != 0) {
                DCMI->ICR = 0x1F;
            }
        }
        
        last_processed_half = ready_half;
        frames_captured_sec++;
        g_frame_counter++;

        // 提取现成的数据并使用 ARM REV16 硬件指令翻转端序 (极速单周期)
        uint16_t* src_buf = (ready_half == 0) ? &dcmi_dma_buf[0] : &dcmi_dma_buf[CAM_W * CAM_H];
        for (int i = 0; i < CAM_W * CAM_H; i++) {
            tx_packet.pixels[i] = __builtin_bswap16(src_buf[i]);
        }

        // SPI 发送图像 (MOSI)，同时从 MISO 接收 ESP32 下发的 CmdPacket 命令
        digitalWrite(MY_PIN_SPI_NSS, LOW);
        delayMicroseconds(2);

        SPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
        // 前 4 字节双向传输：MOSI 发送图像包头，MISO 接收 ESP32 CmdPacket
        SPI.transfer((uint8_t*)&tx_packet, (uint8_t*)&rx_cmd, sizeof(rx_cmd));

        // 解析 MISO 下发的闪光灯指令并同步硬件状态
        if (rx_cmd.flash_on != g_last_flash_state) {
            g_last_flash_state = rx_cmd.flash_on;
            digitalWrite(PIN_FLASH_LED, g_last_flash_state ? HIGH : LOW);
            Serial.printf("  [CMD] MISO 随路指令同步闪光灯: %s\r\n", g_last_flash_state ? "ON" : "OFF");
        }

        uint8_t current_cmd_type = rx_cmd.cmd_type;
        rx_cmd.cmd_type = 0; // 及时清空指令，防止上一轮残留指令误触发

        // 如果 ESP32 发送的是纯命令包 (cmd_type == 2)，只控制灯光，不传输图像
        if (current_cmd_type == 2) {
            SPI.endTransaction();
            digitalWrite(MY_PIN_SPI_NSS, HIGH);
            
            uint32_t wait_low_timeout = 10000;
            while(digitalRead(MY_PIN_HANDSHAKE) == HIGH && wait_low_timeout > 0) { 
                wait_low_timeout--;
                delayMicroseconds(1);
            }
            return; // 结束本轮传输，不吐出 38KB 画面数据
        }

        // 剩余 38408 字节单向发送图像像素 (cmd_type == 1 传输画面)
        SPI.transfer(((uint8_t*)&tx_packet) + sizeof(rx_cmd), nullptr, sizeof(tx_packet) - sizeof(rx_cmd));
        SPI.endTransaction();
        frames_tx_sec++;

        delayMicroseconds(2);
        digitalWrite(MY_PIN_SPI_NSS, HIGH);

        // 等待 ESP32 拉低握手线
        uint32_t wait_low_timeout = 50000;
        while(digitalRead(MY_PIN_HANDSHAKE) == HIGH && wait_low_timeout > 0) { 
            wait_low_timeout--;
            delayMicroseconds(1);
        }
    }
}
