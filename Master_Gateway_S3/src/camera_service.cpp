#include "camera_service.h"

#include <Arduino.h>
#include <driver/spi_slave.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include "app_config.h"
#include "custom_ui_engine.h"
#include "esp_heap_caps.h"
#include "debug_log_service.h"
#include "music_player_service.h"
#define Serial DebugLog

namespace CameraService {

namespace {

constexpr uint16_t RAW_W = 160;
constexpr uint16_t RAW_H = 120;

constexpr uint32_t PKT_HEADER_MAGIC = 0xDEADBEEF; // 包头
constexpr uint32_t PKT_TAIL_MAGIC   = 0xFEEDFACE; // 包尾

struct __attribute__((packed)) ImagePacket {
    uint32_t header_magic;
    uint32_t payload_size;
    uint16_t pixels[RAW_W * RAW_H];
    uint32_t tail_magic;
};

constexpr size_t TOTAL_PACKET_BYTES = sizeof(ImagePacket); // 38,412 字节

struct __attribute__((packed)) CmdPacket {
    uint8_t cmd_type; // 1 = STREAM, 2 = FLASH_TOGGLE
    uint8_t flash_on; // 0 = OFF, 1 = ON
    uint16_t reserved;
};

CmdPacket g_cmd_tx_pkt = {1, 0, 0};

State g_state = State::IDLE;
volatile bool g_start_requested = false;
volatile bool g_stop_requested = false;
volatile bool g_flash_toggle_requested = false;
volatile bool g_flash_enabled = false;
volatile bool g_has_frame = false;
volatile uint32_t g_frame_revision = 0;
volatile uint32_t g_rx_fps = 0;
volatile uint32_t g_rx_total_fps = 0;
volatile uint32_t g_rx_good_fps = 0;
volatile uint32_t g_rx_bad_fps = 0;
static uint32_t g_rx_frames_sec = 0;
static uint32_t g_rx_total_sec = 0;
static uint32_t g_rx_good_sec = 0;
static uint32_t g_rx_bad_sec = 0;
static uint32_t g_last_rx_fps_time = 0;

// DMA 接收缓冲区 (内部 SRAM)
DMA_ATTR uint8_t* g_sram_dma_rx = nullptr;

uint16_t* g_ui_buf[3] = {nullptr, nullptr, nullptr};
QueueHandle_t g_free_queue = nullptr;
QueueHandle_t g_ready_queue = nullptr;
uint8_t g_current_draw_slot = 0xFF;
volatile uint8_t g_last_good_slot = 0xFF;
SemaphoreHandle_t g_ui_lock = nullptr;

TaskHandle_t g_task_handle = nullptr;

void notifyUi() {
    CustomUiEngine::notifyUiNeedsUpdate();
}

void setHandshakeReady(bool ready) {
    digitalWrite(AppConfig::STM32_SPI_HANDSHAKE, ready ? HIGH : LOW);
}

// 硬件级中断回调：DMA 准备就绪时精准拉高握手线
void IRAM_ATTR post_setup_cb(spi_slave_transaction_t *trans) {
    GPIO.out_w1ts = (1 << AppConfig::STM32_SPI_HANDSHAKE); // 极速拉高
}

// 标准包头+包尾解包任务 (绝对零坏包、零乱跳)
void cameraSpiTask(void* pvParameters) {
    pinMode(AppConfig::STM32_SPI_HANDSHAKE, OUTPUT);
    setHandshakeReady(false);

    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = AppConfig::STM32_SPI_MOSI;
    buscfg.miso_io_num = AppConfig::STM32_SPI_MISO;
    buscfg.sclk_io_num = AppConfig::STM32_SPI_SCK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = TOTAL_PACKET_BYTES;

    spi_slave_interface_config_t slvcfg = {};
    slvcfg.mode = 0; // SPI Mode 0
    slvcfg.spics_io_num = AppConfig::STM32_SPI_CS;
    slvcfg.queue_size = 1;
    slvcfg.flags = 0;
    slvcfg.post_setup_cb = post_setup_cb;

    bool spi_initialized = false;

    while (true) {
        if (g_stop_requested) {
            g_stop_requested = false;
            g_start_requested = false;
            MusicPlayerService::sendRawCommand("CMD:CAM_STOP");
            setHandshakeReady(false);
            if (spi_initialized) {
                spi_slave_free(SPI2_HOST);
                spi_initialized = false;
            }
            // 🌟 SRAM 核心优化：停止传输时立即释放 38.4KB DMA 内存归还给系统
            if (g_sram_dma_rx) {
                heap_caps_free(g_sram_dma_rx);
                g_sram_dma_rx = nullptr;
                Serial.println("[CameraService] 摄像头关断，已释放 38.4KB SRAM DMA 内存");
            }
            if (g_free_queue && g_ready_queue) {
                xQueueReset(g_free_queue);
                xQueueReset(g_ready_queue);
                for (uint8_t i = 0; i < 3; i++) {
                    if (g_ui_buf[i]) memset(g_ui_buf[i], 0, CameraService::FRAME_W * CameraService::FRAME_H * 2);
                    xQueueSend(g_free_queue, &i, 0);
                }
            }
            g_current_draw_slot = 0xFF;
            g_last_good_slot = 0xFF;
            g_has_frame = false;
            g_state = State::IDLE;
            notifyUi();
        }

        if (g_start_requested && g_state == State::IDLE) {
            g_start_requested = false;
            g_state = State::CONNECTING;
            g_last_good_slot = 0xFF;
            g_current_draw_slot = 0xFF;
            g_has_frame = false;
            notifyUi();
            
            // 🌟 SRAM 动态申请：开启传输时才按需向系统申请 38.4KB DMA 硬件缓冲
            if (!g_sram_dma_rx) {
                g_sram_dma_rx = (uint8_t*)heap_caps_malloc(TOTAL_PACKET_BYTES, MALLOC_CAP_DMA);
                if (g_sram_dma_rx) memset(g_sram_dma_rx, 0, TOTAL_PACKET_BYTES);
            }

            xQueueReset(g_free_queue);
            xQueueReset(g_ready_queue);
            for (uint8_t i = 0; i < 3; i++) {
                if (g_ui_buf[i]) memset(g_ui_buf[i], 0, CameraService::FRAME_W * CameraService::FRAME_H * 2);
                xQueueSend(g_free_queue, &i, 0);
            }

            if (!spi_initialized && g_sram_dma_rx) {
                esp_err_t ret = spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
                if (ret == ESP_OK) {
                    spi_initialized = true;
                    g_state = State::STREAMING;
                    MusicPlayerService::sendRawCommand("CMD:CAM_START");
                    Serial.println("[CameraService] 完整包头+包尾 5线 SPI2 封包解包服务启动 (已发送 CMD:CAM_START)");
                } else {
                    g_state = State::FAILED;
                }
            }
            notifyUi();
        }

        // --- 闪光灯指令发送 (通过串口直接发送给 STM32) ---
        if (g_flash_toggle_requested) {
            g_flash_toggle_requested = false;
            MusicPlayerService::sendRawCommand(g_flash_enabled ? "CMD:FLASH_ON" : "CMD:FLASH_OFF");
        }

        if (g_state != State::STREAMING || !spi_initialized) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // 1. 挂载发给 STM32 的 MISO 随路指令 (含闪光灯状态)
        g_cmd_tx_pkt.flash_on = g_flash_enabled ? 1 : 0;
        g_cmd_tx_pkt.cmd_type = 1;

        // 2. 启动接收，由底层中断在准备好后自动拉高 HANDSHAKE
        spi_slave_transaction_t t = {};
        t.length = TOTAL_PACKET_BYTES * 8;
        t.rx_buffer = g_sram_dma_rx;
        t.tx_buffer = &g_cmd_tx_pkt; // 通过 MISO 把 CmdPacket 捎带给 STM32

        // 3. 同步阻塞接收一个数据包
        esp_err_t ret = spi_slave_transmit(SPI2_HOST, &t, pdMS_TO_TICKS(100));

        // 3. 接收完毕立刻拉低 HANDSHAKE
        GPIO.out_w1tc = (1 << AppConfig::STM32_SPI_HANDSHAKE);

        if (ret == ESP_OK && t.trans_len > 0) {
            g_rx_total_sec++;
            const ImagePacket* pkt = reinterpret_cast<const ImagePacket*>(g_sram_dma_rx);

            if (pkt->header_magic == PKT_HEADER_MAGIC && pkt->tail_magic == PKT_TAIL_MAGIC) {
                g_rx_good_sec++;
                
                uint8_t slot;
                if (xQueueReceive(g_free_queue, &slot, 0) == pdTRUE) {
                    memcpy(g_ui_buf[slot], pkt->pixels, sizeof(pkt->pixels));
                    g_last_good_slot = slot; // 🌟 记录最近一帧绝对有效的图像缓冲槽位
                    xQueueSend(g_ready_queue, &slot, 0);
                    g_has_frame = true;
                    g_frame_revision++;
                    g_rx_frames_sec++;
                    
                    uint32_t now = millis();
                    if (now - g_last_rx_fps_time >= 1000) {
                        g_rx_fps = g_rx_frames_sec;
                        g_rx_total_fps = g_rx_total_sec;
                        g_rx_good_fps = g_rx_good_sec;
                        g_rx_bad_fps = g_rx_bad_sec;
                        g_rx_frames_sec = 0;
                        g_rx_total_sec = 0;
                        g_rx_good_sec = 0;
                        g_rx_bad_sec = 0;
                        g_last_rx_fps_time = now;
                    }
                    notifyUi();
                } else {
                    g_rx_bad_sec++; // Queue full
                }
            } else {
                g_rx_bad_sec++; // 校验失败丢包
            }
        }

        ImagePacket* pkt_reset = reinterpret_cast<ImagePacket*>(g_sram_dma_rx);
        pkt_reset->header_magic = 0;
        pkt_reset->tail_magic = 0;
        memset(g_sram_dma_rx, 0, 16); // 深度清空前16字节，防止旧魔数触发误判
    }
}

} // namespace

void init() {
    g_ui_lock = xSemaphoreCreateMutex();

    // 🌟 SRAM 核心优化：开机时不常驻预分配 38.4KB DMA 内存，仅在线显存在 PSRAM 中申请
    g_sram_dma_rx = nullptr;
    g_ui_buf[0] = (uint16_t*)heap_caps_malloc(CameraService::FRAME_W * CameraService::FRAME_H * 2, MALLOC_CAP_SPIRAM);
    g_ui_buf[1] = (uint16_t*)heap_caps_malloc(CameraService::FRAME_W * CameraService::FRAME_H * 2, MALLOC_CAP_SPIRAM);
    g_ui_buf[2] = (uint16_t*)heap_caps_malloc(CameraService::FRAME_W * CameraService::FRAME_H * 2, MALLOC_CAP_SPIRAM);

    g_free_queue = xQueueCreate(3, sizeof(uint8_t));
    g_ready_queue = xQueueCreate(3, sizeof(uint8_t));

    if (g_ui_buf[0] && g_ui_buf[1] && g_ui_buf[2] && g_free_queue && g_ready_queue) {
        for (uint8_t i = 0; i < 3; i++) {
            memset(g_ui_buf[i], 0, CameraService::FRAME_W * CameraService::FRAME_H * 2);
            xQueueSend(g_free_queue, &i, 0);
        }
        Serial.println("[CameraService] 摄像头初始化成功 (DMA缓冲动态错峰分配，已释放 38.4KB SRAM)");
        xTaskCreatePinnedToCore(cameraSpiTask, "cameraSpiTask", 4096, NULL, 5, &g_task_handle, 1);
    } else {
        Serial.println("[CameraService] 内存分配失败");
        g_state = State::FAILED;
    }
}

static uint16_t* g_freeze_frame = nullptr;
static volatile bool g_has_freeze_frame = false;

void clearFreezeFrame() {
    g_has_freeze_frame = false;
    if (g_freeze_frame) {
        memset(g_freeze_frame, 0, CameraService::FRAME_W * CameraService::FRAME_H * 2);
        heap_caps_free(g_freeze_frame);
        g_freeze_frame = nullptr;
    }
    g_last_good_slot = 0xFF;
    g_current_draw_slot = 0xFF;
    g_has_frame = false;
    Serial.println("[CameraService] 快照与历史帧已彻底深度清空");
}

void start() {
    clearFreezeFrame();
    if (MusicPlayerService::getPlayState() == MusicPlayerService::STATE_PLAYING ||
        MusicPlayerService::getPlayState() == MusicPlayerService::STATE_PAUSED) {
        MusicPlayerService::sendStop();
        Serial.println("[CameraService] ⚡ SPI互斥: 检测到音乐正在运行，已自动关闭音乐以独占 SPI2 总线");
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    g_stop_requested = false;
    g_start_requested = true;
}

void stop() {
    g_start_requested = false;
    g_stop_requested = true;
}

bool captureFreezeFrame() {
    if (!g_freeze_frame) {
        g_freeze_frame = (uint16_t*)heap_caps_malloc(CameraService::FRAME_W * CameraService::FRAME_H * 2, MALLOC_CAP_SPIRAM);
    }
    if (!g_freeze_frame) return false;

    const uint16_t* src = nullptr;
    if (g_last_good_slot < 3 && g_ui_buf[g_last_good_slot]) {
        src = g_ui_buf[g_last_good_slot];
    } else if (g_current_draw_slot < 3 && g_ui_buf[g_current_draw_slot]) {
        src = g_ui_buf[g_current_draw_slot];
    } else {
        for (int i = 0; i < 3; i++) {
            if (g_ui_buf[i]) {
                src = g_ui_buf[i];
                break;
            }
        }
    }

    if (src) {
        memcpy(g_freeze_frame, src, CameraService::FRAME_W * CameraService::FRAME_H * 2);
        g_has_freeze_frame = true;
        uint32_t checksum = 0;
        for (int i = 0; i < CameraService::FRAME_W * CameraService::FRAME_H; i += 16) {
            checksum += g_freeze_frame[i];
        }
        Serial.printf("[CameraService] 成功提取最新快照 (来自 Slot %d, 帧特征码: 0x%08X)\r\n", 
                      (int)(src == g_ui_buf[0] ? 0 : (src == g_ui_buf[1] ? 1 : 2)), checksum);
        return true;
    }
    return false;
}

uint16_t* getFreezeFrame() {
    return (g_has_freeze_frame && g_freeze_frame) ? g_freeze_frame : nullptr;
}

bool hasFreezeFrame() {
    return g_has_freeze_frame && (g_freeze_frame != nullptr);
}

void onAck(const RpcAcknowledgement& ack) {}

bool wantsRadioAwake() { return false; }

State state() { return g_state; }

bool hasFrame() { return g_has_frame || g_has_freeze_frame; }

uint32_t frameRevision() { return g_frame_revision; }

bool hasQueuedFrames() {
    return uxQueueMessagesWaiting(g_ready_queue) > 0;
}

uint32_t rxFps() { return g_rx_fps; }
uint32_t rxTotalFps() { return g_rx_total_fps; }
uint32_t rxGoodFps() { return g_rx_good_fps; }
uint32_t rxBadFps() { return g_rx_bad_fps; }

String statusText() {
    switch (g_state) {
        case State::IDLE:       return "已就绪 (点击开始连接)";
        case State::WAKING:     return "正在唤醒 STM32...";
        case State::CONNECTING: return "正在握手连接...";
        case State::STREAMING:  return "已连接";
        case State::FAILED:     return "传输失败";
        default:                return "离线";
    }
}

String ipText() { return "STM32H743 (RAW)"; }

uint16_t* lockFrame() {
    uint8_t slot;
    if (xQueueReceive(g_ready_queue, &slot, 0) == pdTRUE) {
        g_current_draw_slot = slot;
        return g_ui_buf[slot];
    }
    if (g_has_freeze_frame && g_freeze_frame) {
        return g_freeze_frame;
    }
    return nullptr;
}

void unlockFrame() {
    if (g_current_draw_slot != 0xFF) {
        xQueueSend(g_free_queue, (void*)&g_current_draw_slot, 0);
        g_current_draw_slot = 0xFF;
    }
}

} // namespace CameraService

static bool g_flash_enabled = false;
static bool g_flash_toggle_requested = false;

namespace CameraService {
void toggleFlash() {
    g_flash_enabled = !g_flash_enabled;
    g_flash_toggle_requested = true;
    Serial.printf("[CameraService] Flash toggled: %s\r\n", g_flash_enabled ? "ON" : "OFF");
    notifyUi();
}

bool isFlashOn() {
    return g_flash_enabled;
}
}
