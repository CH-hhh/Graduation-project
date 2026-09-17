#include "music_player_service.h"
#include "audio_service.h"
#include "buzzer_service.h"
#include "camera_service.h"
#include "custom_ui_engine.h"
#include "app_config.h"
#include <driver/i2s.h>
#include <driver/spi_slave.h>
#include <HardwareSerial.h>
#include <math.h>


#include <soc/gpio_reg.h>
#include <soc/soc.h>

extern volatile bool isRecording;
extern volatile bool isCommunicating;
extern volatile bool isSpeaking;

namespace MusicPlayerService {

static HardwareSerial MusicSerial(1);

struct __attribute__((packed)) AudioPacket {
    uint32_t magic;         // 0xA0D10001
    uint16_t sample_count;  // 1152 采样点 (44.1kHz MPEG-1 Layer 3 原生帧)
    uint16_t sample_rate;   // 🌟 物理采样率 (44100 / 32000 / 24000 / 48000 Hz)
    int16_t  samples[1152]; // 2304 字节 Mono PCM
    uint32_t tail_magic;    // 0xFEEDFACE
};

static TaskHandle_t s_spi_task_handle = nullptr;
static DMA_ATTR uint8_t s_spi_rx_buf[sizeof(AudioPacket) + 32];
static uint32_t s_total_pcm_rx_bytes = 0;

static std::vector<SongItem> s_playlist;
static int s_currentTrack = 0;
static PlayState s_playState = STATE_STOPPED;
static int s_currentSeconds = 0;
static int s_totalSeconds = 0;
static int s_saved_pause_seconds = 0; // 🌟 记录暂停时刻的精确播放秒数
static int s_saved_pause_track = 0;   // 🌟 记录暂停时刻的曲目
static int s_volume = 80;
static PlayMode s_playMode = MODE_SEQUENCE;
static String s_currentLyric = "暂无歌词 · 点击连接获取歌单";
static uint32_t s_last_update_ms = 0;
static uint32_t s_last_spectrum_ms = 0;
static uint32_t s_last_tone_ms = 0;
static uint8_t s_spectrum[16] = {0};
static bool s_connected = false;
static bool s_i2s_active = false;

static const uint16_t MELODY_NOTES[] = {
    523, 587, 659, 698, 784, 880, 988, 1046,
    880, 784, 659, 587, 523, 659, 784, 523,
    659, 587, 523};

// 🌟 16 块 × 32KB 独立槽位环形队列 (共 512KB PSRAM，近 6.0 秒超大抗抖底仓)
// 🌟 224 帧 × 2304 字节 原生帧锁相槽位队列 (共 504KB PSRAM，近 5.85 秒超大抗抖底仓)
// 每个 Slot 精确对应 1 个 MP3 解码帧 (1152 采样点/2304 字节)，与 STM32 SPI 发送包 1:1 绝对严密对齐！
static constexpr size_t NUM_FRAME_SLOTS = 224;
static constexpr size_t MAX_SAMPLES_PER_FRAME = 1152;

struct FrameSlot {
    uint16_t sample_count;                   // 当前帧实际采样点数 (通常为 1152 或 576)
    bool     is_eos;                         // 🌟 真实物理流结束标记 (End Of Stream)
    int16_t  samples[MAX_SAMPLES_PER_FRAME]; // 原生 16-bit 单声道 PCM 样本
};

static FrameSlot* s_frame_slots = nullptr;
static volatile uint16_t s_frame_write_idx = 0;
static volatile uint16_t s_frame_read_idx = 0;
static volatile uint16_t s_frame_ready_count = 0;
static portMUX_TYPE s_frame_mux = portMUX_INITIALIZER_UNLOCKED;

static inline void frame_queue_clear() {
    portENTER_CRITICAL(&s_frame_mux);
    s_frame_write_idx = 0;
    s_frame_read_idx = 0;
    s_frame_ready_count = 0;
    portEXIT_CRITICAL(&s_frame_mux);
}

static inline size_t frame_queue_available_bytes() {
    if (s_frame_slots == nullptr) return 0;
    return (size_t)s_frame_ready_count * (MAX_SAMPLES_PER_FRAME * sizeof(int16_t));
}

// 生产者 (SPI 接收任务)：原子预占写入槽位，写入完成后再原子发布就绪，绝对杜绝高优先级任务打断读取半写脏数据！
static inline bool frame_queue_push(const int16_t* pcm, uint16_t count, bool is_eos = false) {
    if (s_frame_slots == nullptr) return false;
    if (!is_eos && (pcm == nullptr || count == 0)) return false;
    if (count > MAX_SAMPLES_PER_FRAME) count = MAX_SAMPLES_PER_FRAME;

    portENTER_CRITICAL(&s_frame_mux);
    if (s_frame_ready_count >= NUM_FRAME_SLOTS) {
        portEXIT_CRITICAL(&s_frame_mux);
        return false; // 224 槽全部蓄满 (5.85 秒)
    }
    uint16_t w_idx = s_frame_write_idx;
    s_frame_write_idx = (s_frame_write_idx + 1) % NUM_FRAME_SLOTS;
    portEXIT_CRITICAL(&s_frame_mux);

    FrameSlot& slot = s_frame_slots[w_idx];
    slot.sample_count = count;
    slot.is_eos = is_eos;
    if (pcm && count > 0) {
        memcpy(slot.samples, pcm, count * sizeof(int16_t));
    }

    portENTER_CRITICAL(&s_frame_mux);
    s_frame_ready_count++;
    portEXIT_CRITICAL(&s_frame_mux);
    return true;
}

// 消费者 (I2S 推流任务)：原子提取就绪槽位，完整读取后再释放槽位，时序 100% 严密闭环！
static inline bool frame_queue_pop(FrameSlot& out_slot) {
    if (s_frame_slots == nullptr) return false;

    portENTER_CRITICAL(&s_frame_mux);
    if (s_frame_ready_count == 0) {
        portEXIT_CRITICAL(&s_frame_mux);
        return false;
    }
    uint16_t r_idx = s_frame_read_idx;
    s_frame_read_idx = (s_frame_read_idx + 1) % NUM_FRAME_SLOTS;
    portEXIT_CRITICAL(&s_frame_mux);

    out_slot.sample_count = s_frame_slots[r_idx].sample_count;
    out_slot.is_eos = s_frame_slots[r_idx].is_eos;
    if (out_slot.sample_count > 0) {
        memcpy(out_slot.samples, s_frame_slots[r_idx].samples, out_slot.sample_count * sizeof(int16_t));
    }

    portENTER_CRITICAL(&s_frame_mux);
    s_frame_ready_count--;
    portEXIT_CRITICAL(&s_frame_mux);
    return true;
}

// 🌟 16 频段真实 PCM 实时频谱分析器 (256 点 Radix-2 快速傅里叶变换 FFT)
static constexpr int FFT_N = 256;
static float s_fft_re[FFT_N];
static float s_fft_im[FFT_N];
static float s_han_win[FFT_N];
static bool s_fft_inited = false;
static volatile uint8_t s_target_spectrum[16] = {0};
static volatile uint8_t s_spectrum_peaks[16] = {0};
static uint8_t s_peak_hold[16] = {0};
static int16_t s_fft_samples_snapshot[FFT_N] = {0};
static volatile bool s_fft_snapshot_ready = false;

// 16 频段对数中心频率划分 (从 172Hz 次重低音到 16kHz 超高音)
static const uint8_t BAND_SPLITS[17] = {
    1, 2, 3, 4, 6, 8, 11, 14, 18, 23, 29, 37, 47, 60, 75, 90, 105
};

// 🌟 1/f 粉红噪声等响度均衡补偿权重 (中高频增益补偿)
static const float BAND_WEIGHTS[16] = {
    1.2f, // Band 0: ~172Hz
    1.4f, // Band 1: ~344Hz
    1.6f, // Band 2: ~516Hz
    1.8f, // Band 3: ~688Hz
    2.1f, // Band 4: ~1033Hz
    2.4f, // Band 5: ~1378Hz
    2.8f, // Band 6: ~1894Hz
    3.2f, // Band 7: ~2411Hz
    3.7f, // Band 8: ~3100Hz
    4.3f, // Band 9: ~3961Hz
    5.0f, // Band 10: ~4994Hz
    5.8f, // Band 11: ~6373Hz
    6.8f, // Band 12: ~8096Hz
    8.0f, // Band 13: ~10335Hz
    9.5f, // Band 14: ~12920Hz
    11.2f // Band 15: ~15504Hz
};

static void initFftTables() {
    if (s_fft_inited) return;
    for (int i = 0; i < FFT_N; i++) {
        s_han_win[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (float)(FFT_N - 1)));
    }
    s_fft_inited = true;
}

static void computeRealPcmSpectrum(const int16_t* samples, int count) {
    if (samples == nullptr || count < FFT_N) return;
    initFftTables();

    // 1. 直流偏移消除 + 汉宁窗加权
    float dc = 0.0f;
    for (int i = 0; i < FFT_N; i++) dc += (float)samples[i];
    dc /= (float)FFT_N;

    for (int i = 0; i < FFT_N; i++) {
        s_fft_re[i] = (((float)samples[i] - dc) / 32768.0f) * s_han_win[i];
        s_fft_im[i] = 0.0f;
    }

    // 2. 256 点 Radix-2 原位快速傅里叶变换 (Cooley-Tukey DIT)
    int j = 0;
    for (int i = 0; i < FFT_N - 1; i++) {
        if (i < j) {
            float tr = s_fft_re[i]; s_fft_re[i] = s_fft_re[j]; s_fft_re[j] = tr;
            float ti = s_fft_im[i]; s_fft_im[i] = s_fft_im[j]; s_fft_im[j] = ti;
        }
        int k = FFT_N >> 1;
        while (k <= j) {
            j -= k;
            k >>= 1;
        }
        j += k;
    }

    for (int len = 2; len <= FFT_N; len <<= 1) {
        float ang = -2.0f * (float)M_PI / (float)len;
        float wlen_r = cosf(ang);
        float wlen_i = sinf(ang);
        int half = len >> 1;
        for (int i = 0; i < FFT_N; i += len) {
            float wr = 1.0f;
            float wi = 0.0f;
            for (int k = 0; k < half; k++) {
                int u = i + k;
                int v = u + half;
                float vr = s_fft_re[v] * wr - s_fft_im[v] * wi;
                float vi = s_fft_re[v] * wi + s_fft_im[v] * wr;
                s_fft_re[v] = s_fft_re[u] - vr;
                s_fft_im[v] = s_fft_im[u] - vi;
                s_fft_re[u] += vr;
                s_fft_im[u] += vi;
                float next_wr = wr * wlen_r - wi * wlen_i;
                wi = wr * wlen_i + wi * wlen_r;
                wr = next_wr;
            }
        }
    }

    // 3. 计算 16 个频段幅值并归一化为真实物理相对幅值 [0.0, 1.0]
    for (int b = 0; b < 16; b++) {
        int start_bin = BAND_SPLITS[b];
        int end_bin = BAND_SPLITS[b + 1];
        float sum_mag = 0.0f;
        for (int k = start_bin; k < end_bin; k++) {
            float mag = sqrtf(s_fft_re[k] * s_fft_re[k] + s_fft_im[k] * s_fft_im[k]);
            if (mag > sum_mag) sum_mag = mag;
        }
        
        // 🌟 1. 消除 256 点 DIT-FFT 累加增益 (N/4 = 64.0)，将幅值严格归一化到 0.0 ~ 1.0 真实基准
        float raw_mag = sum_mag / 64.0f;

        // 🌟 2. 频段等响度加权
        float weighted_mag = raw_mag * BAND_WEIGHTS[b];

        // 🌟 3. 对数分贝动态范围映射 (-45dB ~ 0dB 映射为 0.0 ~ 1.0)
        float db = 20.0f * log10f(weighted_mag + 1e-4f);
        float u = (db + 45.0f) / 45.0f;
        if (u < 0.0f) u = 0.0f;
        if (u > 1.0f) u = 1.0f;

        // 🌟 4. 核心非线性动态响应 (Gamma=1.3 灵敏弹跳)：
        // 伴奏与人声活跃律动在 25% ~ 60%，高潮与重音瞬间冲顶 90% ~ 100%，起伏鲜明生动！
        float curved = powf(u, 1.3f);
        float normalized = curved * 100.0f;
        if (normalized > 100.0f) normalized = 100.0f;
        if (normalized < 0.0f) normalized = 0.0f;

        s_target_spectrum[b] = (uint8_t)normalized;
    }
}

static TaskHandle_t s_audio_task_handle = nullptr;
static uint32_t s_total_pcm_played_bytes = 0;
static uint32_t s_music_sample_rate = 44100;

// 🌟 片上 SRAM DMA 跳板 (单帧 1152 采样点/2304 字节单声道 -> 4608 字节立体声 DMA 推流，单次 26.12ms)
static int16_t s_stream_stereo_dma_chunk[MAX_SAMPLES_PER_FRAME * 2];
static int16_t s_stream_silence_chunk[MAX_SAMPLES_PER_FRAME * 2] = {0};
static volatile bool s_need_prebuffer = true;
static volatile bool s_is_refilling = true;
static volatile bool s_was_in_silence = true;

struct LyricItem {
    uint32_t timeMs;
    String text;
};
static std::vector<LyricItem> s_local_lyrics;
static volatile bool s_is_muted = false;

void setMuteState(bool mute) {
    s_is_muted = mute;
}

static void musicAudioStreamTask(void* param) {
    static FrameSlot pop_slot;
    static int16_t s_last_sample_val = 0;
    while (true) {
        // 🌟 核心互斥：若 AI 正在录音、思考或播报 TTS，音乐任务彻底释放 I2S 硬件通道
        if (isRecording || isCommunicating || isSpeaking) {
            s_i2s_active = false;
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (s_playState == STATE_PLAYING || s_playState == STATE_PAUSED) {
            if (!s_i2s_active) {
                AudioService::initForMusic(s_music_sample_rate); // 🌟 独立调用音乐 44.1kHz 专用初始化函数
                AudioService::wake(); // 🌟 启动 I2S 硬件时钟引擎
                s_i2s_active = true;
                s_last_sample_val = 0;
                s_was_in_silence = true;
                Serial.printf("🔊 [ESP32-I2S] 音乐播放流式任务已激活并启动 %u Hz 音乐专用输出！\r\n", (unsigned)s_music_sample_rate);
            }

            if (s_playState == STATE_PLAYING && !s_is_muted) {
                // 🌟 换曲首包预缓冲保护：必须蓄满 120 帧 (足足 3.13 秒厚重底仓，仅需 0.35 秒极速装满)
                // 彻底准备完毕后才开播，延迟零点几秒，保证全程运行平稳、零掉帧、零波波杂音！
                if (s_need_prebuffer) {
                    if (s_frame_ready_count >= 120) {
                        s_need_prebuffer = false;
                        s_was_in_silence = true;
                        AudioService::resetDma();
                        Serial.printf("✨ [ESP32-MUSIC] 水库已充盈 %u 帧 (3.13秒底仓)，DMA 清零并平滑开播！\r\n", (unsigned)s_frame_ready_count);
                    } else {
                        vTaskDelay(pdMS_TO_TICKS(10));
                        continue;
                    }
                }

                if (frame_queue_pop(pop_slot)) {
                    if (pop_slot.is_eos) {
                        // 🌟 扬声器已真正播完全部真实 PCM 采样点，物理流自然结束！
                        Serial.printf("🎉 [ESP32-I2S] 扬声器已物理播完全部真实 PCM 采样点，无缝触发切歌！(曲目:#%02d)\r\n", s_currentTrack + 1);
                        if (s_playMode == MODE_SINGLE_REPEAT) {
                            sendPlay(s_currentTrack);
                        } else if (s_playMode == MODE_RANDOM) {
                            int r = (s_playlist.size() > 1) ? (rand() % (int)s_playlist.size()) : 0;
                            sendPlay(r);
                        } else {
                            sendNext();
                        }
                        s_need_prebuffer = true;
                        continue;
                    }

                    int samples_count = pop_slot.sample_count;

                    // 🌟 极速快照采样数据给 UI 线程执行 FFT (耗时 < 1 微秒)
                    if (!s_fft_snapshot_ready && samples_count >= FFT_N) {
                        memcpy(s_fft_samples_snapshot, pop_slot.samples, sizeof(s_fft_samples_snapshot));
                        s_fft_snapshot_ready = true;
                    }

                    // 🌟 扩展为标准立体声帧 (L+R)：
                    // 仅在刚从静音/切歌启动的首个音频帧前 32 点做平滑淡入；
                    // 连续播放期间原生音频帧已由 MDCT 完美相交连续，直接 100% 原生输出，零失真零断裂！
                    float vol_scale = (s_volume / 100.0f);
                    if (s_was_in_silence) {
                        for (int i = 0; i < samples_count; i++) {
                            int16_t sample_val = (int16_t)(pop_slot.samples[i] * vol_scale);
                            if (i < 32) {
                                sample_val = (int16_t)((int32_t)sample_val * (i + 1) / 32);
                            }
                            s_stream_stereo_dma_chunk[i * 2]     = sample_val;
                            s_stream_stereo_dma_chunk[i * 2 + 1] = sample_val;
                        }
                        s_was_in_silence = false;
                    } else {
                        for (int i = 0; i < samples_count; i++) {
                            int16_t sample_val = (int16_t)(pop_slot.samples[i] * vol_scale);
                            s_stream_stereo_dma_chunk[i * 2]     = sample_val;
                            s_stream_stereo_dma_chunk[i * 2 + 1] = sample_val;
                        }
                    }
                    if (samples_count > 0) {
                        s_last_sample_val = (int16_t)(pop_slot.samples[samples_count - 1] * vol_scale);
                    }

                    size_t dma_bytes_to_write = samples_count * sizeof(int16_t) * 2;
                    size_t bytes_written = 0;
                    esp_err_t err = i2s_write(I2S_NUM_0, s_stream_stereo_dma_chunk, dma_bytes_to_write, &bytes_written, portMAX_DELAY);
                    if (err == ESP_OK) {
                        s_total_pcm_played_bytes += (samples_count * sizeof(int16_t));
                    }
                } else {
                    // 🌟 若水库彻底为 0 帧 (抽空脱节)：
                    // 立即置位 s_need_prebuffer = true，等待蓄满至少 20 帧再重新开播，彻底杜绝单帧断续抖动与跳帧！
                    s_need_prebuffer = true;
                    s_was_in_silence = true;
                    size_t bytes_written = 0;
                    i2s_write(I2S_NUM_0, s_stream_silence_chunk, sizeof(s_stream_silence_chunk), &bytes_written, pdMS_TO_TICKS(50));
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            } else {
                // 🌟 暂停状态下向 I2S 持续输送静音 PCM 帧 (全0)，维持硬件 BCLK/LRCK 连续稳定震荡
                s_last_sample_val = 0;
                s_was_in_silence = true;
                size_t bytes_written = 0;
                i2s_write(I2S_NUM_0, s_stream_silence_chunk, sizeof(s_stream_silence_chunk), &bytes_written, pdMS_TO_TICKS(50));
                vTaskDelay(pdMS_TO_TICKS(5));
            }
        } else {
            // 🌟 仅在完全停止 (STATE_STOPPED) 时才关闭时钟
            s_last_sample_val = 0;
            s_was_in_silence = true;
            if (s_i2s_active) {
                AudioService::sleep();
                s_i2s_active = false;
            }
            vTaskDelay(pdMS_TO_TICKS(40));
        }
    }
}

static void IRAM_ATTR music_post_setup_cb(spi_slave_transaction_t *trans) {
    WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1 << AppConfig::STM32_SPI_HANDSHAKE)); // 硬件 DMA 挂载完毕，瞬时拉高握手线
}

static void IRAM_ATTR music_post_trans_cb(spi_slave_transaction_t *trans) {
    WRITE_PERI_REG(GPIO_OUT_W1TC_REG, (1 << AppConfig::STM32_SPI_HANDSHAKE)); // 硬件中断内传输完成瞬时拉低握手线
}

static void musicSpiRxTask(void* pvParameters) {
    pinMode(AppConfig::STM32_SPI_HANDSHAKE, OUTPUT);
    digitalWrite(AppConfig::STM32_SPI_HANDSHAKE, LOW);

    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = AppConfig::STM32_SPI_MOSI;
    buscfg.miso_io_num = AppConfig::STM32_SPI_MISO;
    buscfg.sclk_io_num = AppConfig::STM32_SPI_SCK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = sizeof(AudioPacket);

    spi_slave_interface_config_t slvcfg = {};
    slvcfg.mode = 0;
    slvcfg.spics_io_num = AppConfig::STM32_SPI_CS;
    slvcfg.queue_size = 1;
    slvcfg.flags = 0;
    slvcfg.post_setup_cb = music_post_setup_cb;
    slvcfg.post_trans_cb = music_post_trans_cb;

    bool spi_ready = false;

    while (true) {
        if (s_playState == STATE_PLAYING || s_playState == STATE_PAUSED) {
            if (!spi_ready) {
                esp_err_t ret = spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
                if (ret == ESP_OK) {
                    spi_ready = true;
                    Serial.println("🚀 [ESP32-MUSIC] 5线 20MHz SPI2 硬件握手从机 DMA 音频总线启动就绪！");
                }
            }

            if (spi_ready && s_playState == STATE_PLAYING) {
                // 🌟 经典弹性和缓平滑流控 (Elastic Equilibrium Pacing)：
                // 单帧物理播发周期为 26.12ms：
                // 1. 绝对防溢出硬闸：若水位达到 205 帧 (留 19 帧安全余量)，坚决不挂载 SPI，等待播放消耗，100% 杜绝满载丢帧！
                // 2. 水库 >= 190 帧 (4.96秒高位)：让出 30ms (> 26.12ms，水位缓缓回落)
                // 3. 水库 >= 170 帧 (4.44秒充盈位)：让出 24ms (加 SPI 传输 ~26ms，达成完美 1:1 动态平衡)
                // 4. 水库 >= 140 帧 (3.65秒安全位)：让出 10ms (< 26.12ms，水位缓缓上升补水)
                // 5. 水库 < 140 帧：0ms 极速接收充能
                // 水位形成完美的负反馈闭环，死死锁定在 165 ~ 185 帧黄金带，永不触碰 224 满载边界，永不丢帧！
                if (s_frame_ready_count >= 205) {
                    vTaskDelay(pdMS_TO_TICKS(20));
                    continue;
                } else if (s_frame_ready_count >= 190) {
                    vTaskDelay(pdMS_TO_TICKS(30));
                } else if (s_frame_ready_count >= 170) {
                    vTaskDelay(pdMS_TO_TICKS(24));
                } else if (s_frame_ready_count >= 140) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }

                memset(s_spi_rx_buf, 0, sizeof(s_spi_rx_buf));
                spi_slave_transaction_t trans = {};
                trans.length = sizeof(AudioPacket) * 8;
                trans.rx_buffer = s_spi_rx_buf;

                esp_err_t ret = spi_slave_transmit(SPI2_HOST, &trans, pdMS_TO_TICKS(1000));

                if (ret == ESP_OK && trans.trans_len >= sizeof(AudioPacket) * 8) {
                    const AudioPacket* pkt = (const AudioPacket*)s_spi_rx_buf;
                    if ((pkt->magic == 0xA0D10001 || pkt->magic == 0xA0D100EE) && pkt->tail_magic == 0xFEEDFACE) {
                        if (pkt->magic == 0xA0D100EE) {
                            // 🌟 收到真实物理流结束标记 (End Of Stream)
                            frame_queue_push(nullptr, 0, true);
                            Serial.printf("🏁 [ESP32-SPI] 接收到 STM32 下发的 EOS 流结束标记，已排入 PSRAM 水库！\r\n");
                        } else if (pkt->sample_count > 0 && pkt->sample_count <= MAX_SAMPLES_PER_FRAME) {
                            if (pkt->sample_rate >= 8000 && pkt->sample_rate <= 96000 && pkt->sample_rate != s_music_sample_rate) {
                                s_music_sample_rate = pkt->sample_rate;
                                if (s_i2s_active) {
                                    AudioService::setSampleRate(pkt->sample_rate);
                                }
                                Serial.printf("🎯 [ESP32-SPI] 硬件实时同步音频帧采样率: %u Hz\r\n", (unsigned)pkt->sample_rate);
                            }
                            frame_queue_push(pkt->samples, pkt->sample_count, false);
                            s_total_pcm_rx_bytes += (pkt->sample_count * sizeof(int16_t));
                        }
                    } else {
                        Serial.printf("⚠️ [ESP32-SPI] 校验失败数据包 (Magic: 0x%08X, Tail: 0x%08X, Len: %d)\r\n", 
                                      pkt->magic, pkt->tail_magic, (int)trans.trans_len);
                    }
                }
            } else {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        } else {
            if (spi_ready) {
                digitalWrite(AppConfig::STM32_SPI_HANDSHAKE, LOW);
                spi_slave_free(SPI2_HOST);
                spi_ready = false;
                Serial.println("💤 [ESP32-MUSIC] 音乐停止，已释放 SPI2_HOST 归还给系统/摄像头");
            }
            vTaskDelay(pdMS_TO_TICKS(30));
        }
    }
}

void loadMockPlaylistIfEmpty() {
    // 纯动态模式：不填入任何伪造示例歌曲，初始保持纯净占位状态
}

void begin() {
    // 🌟 在 4MB PSRAM 中动态开辟 224 帧 × 2304 字节 原生帧锁相槽位队列 (共 504KB, 约 5.85 秒)
    if (!s_frame_slots) {
        if (psramFound()) {
            s_frame_slots = (FrameSlot*)ps_malloc(NUM_FRAME_SLOTS * sizeof(FrameSlot));
            if (s_frame_slots) {
                Serial.printf("✨ [ESP32-MUSIC] 成功在 4MB PSRAM 中开辟 224 帧原生帧锁相队列 (共 %u KB, 约 5.85 秒)！\r\n", 
                              (unsigned)(NUM_FRAME_SLOTS * sizeof(FrameSlot) / 1024));
            }
        }
        if (!s_frame_slots) {
            s_frame_slots = (FrameSlot*)malloc(NUM_FRAME_SLOTS * sizeof(FrameSlot));
            Serial.printf("ℹ️ [ESP32-MUSIC] PSRAM 未使能，回退分配内部 SRAM %u KB 帧队列\r\n", 
                          (unsigned)(NUM_FRAME_SLOTS * sizeof(FrameSlot) / 1024));
        }
    }
    frame_queue_clear();

    MusicSerial.setRxBufferSize(16384);
    MusicSerial.begin(115200, SERIAL_8N1, static_cast<int>(AppConfig::MUSIC_UART_RX), static_cast<int>(AppConfig::MUSIC_UART_TX));
    Serial.printf("[ESP32-MUSIC] 启动控制总线 UART1 115200 Baud (RX:%d, TX:%d)\r\n", 
                  static_cast<int>(AppConfig::MUSIC_UART_RX), static_cast<int>(AppConfig::MUSIC_UART_TX));

    if (!s_audio_task_handle) {
        // 🌟 音频推流任务设为 Priority 7 (最高级，杜绝任何 I2S DMA 饥饿与断流)
        xTaskCreatePinnedToCore(musicAudioStreamTask, "music_audio_task", 8192, nullptr, 7, &s_audio_task_handle, 0);
        Serial.println("[ESP32-MUSIC] 启动专属 I2S DMA 连续流式音频任务 (Core 0, Priority 7, 8KB 充裕栈空间)");
    }

    if (!s_spi_task_handle) {
        // 🌟 SPI 从机接收任务设为 Priority 5 (与音频任务共享 Core 0 L1 Cache)
        xTaskCreatePinnedToCore(musicSpiRxTask, "music_spi_task", 8192, nullptr, 5, &s_spi_task_handle, 0);
        Serial.println("[ESP32-MUSIC] 启动 5线 20MHz SPI2 硬件从机 DMA 接收任务 (Core 0, Priority 5)");
    }

    // 🌟 开机双向握手与防误播同步：
    // 强制发送 CMD:STOP 停止 STM32 可能在 ESP32 关机期间残留的后台推流，并刷新 SD 卡歌单
    MusicSerial.println("CMD:STOP");
    vTaskDelay(pdMS_TO_TICKS(30));
    MusicSerial.println("CMD:GET_LIST");
}

static void prepareForNewTrack() {
    s_need_prebuffer = true;
    s_is_refilling = true;
    s_total_pcm_played_bytes = 0;
    s_currentSeconds = 0;
    frame_queue_clear();
    memset(s_stream_stereo_dma_chunk, 0, sizeof(s_stream_stereo_dma_chunk));
    AudioService::resetDma();
    Serial.println("🔄 [ESP32-MUSIC] 新曲目 224 帧槽位与硬件彻底深度清空重置，DMA 与 APLL 时钟复位就绪！");
}

static void processIncomingLine(const String& line) {
    if (line == "DISCONNECTED" || line.startsWith("DISCONNECTED")) {
        s_connected = false;
        s_playState = STATE_STOPPED;
        Serial.println("🛑 [ESP32-MUSIC] 收到 STM32 断开确认，状态已切换为【连接】");
        return;
    }

    if (line == "CONNECTED" || line.startsWith("CONNECTED")) {
        s_connected = true;
        Serial.println("✅ [ESP32-MUSIC] 与 STM32 建立连接握手成功！状态已设为【已连】");
        return;
    }

    if (line.startsWith("LIST_START") || line.startsWith("ITEM:") || line.startsWith("LIST_END")) {
        s_connected = true;
    }

    if (line.startsWith("LIST_START")) {
        s_playlist.clear();
        Serial.println("📋 [ESP32-MUSIC] 正在接收 SD 卡歌单列表...");
    } else if (line.startsWith("ITEM:") || line.indexOf("ITEM:") >= 0) {
        // 🌟 强健防御：若由于极端串口抖动导致单行粘连了多个 "ITEM:"，逐个拆解，绝不丢失任何一首歌！
        String remaining = line;
        while (remaining.length() > 0) {
            int itemPos = remaining.indexOf("ITEM:");
            if (itemPos < 0) break;
            int nextItemPos = remaining.indexOf("ITEM:", itemPos + 5);
            String itemStr = (nextItemPos > 0) ? remaining.substring(itemPos, nextItemPos) : remaining.substring(itemPos);
            remaining = (nextItemPos > 0) ? remaining.substring(nextItemPos) : "";

            String content = itemStr.substring(5);
            int id = 0, dur = 240;
            String title = "", artist = "未知歌手";
            
            int t1 = content.indexOf('\t');
            if (t1 > 0) {
                int t2 = content.indexOf('\t', t1 + 1);
                int t3 = (t2 > 0) ? content.indexOf('\t', t2 + 1) : -1;
                id = content.substring(0, t1).toInt();
                if (t2 > t1) {
                    title = content.substring(t1 + 1, t2);
                    if (t3 > t2) {
                        dur = content.substring(t2 + 1, t3).toInt();
                        artist = content.substring(t3 + 1);
                    } else {
                        dur = content.substring(t2 + 1).toInt();
                    }
                } else {
                    title = content.substring(t1 + 1);
                }
            } else {
                // 回退逗号解析 (从两端提取，中间全归歌名，杜绝歌名含逗号错位)
                int firstComma = content.indexOf(',');
                int lastComma = content.lastIndexOf(',');
                if (firstComma > 0 && lastComma > firstComma) {
                    id = content.substring(0, firstComma).toInt();
                    artist = content.substring(lastComma + 1);
                    int secondLastComma = content.lastIndexOf(',', lastComma - 1);
                    if (secondLastComma > firstComma) {
                        title = content.substring(firstComma + 1, secondLastComma);
                        dur = content.substring(secondLastComma + 1, lastComma).toInt();
                    } else {
                        title = content.substring(firstComma + 1, lastComma);
                    }
                }
            }
            title.trim();
            artist.trim();
            if (title.length() > 0) {
                s_playlist.push_back({id, title, artist, dur});
            }
        }
    } else if (line.startsWith("LIST_END")) {
        if (s_currentTrack >= (int)s_playlist.size()) s_currentTrack = 0;
        if (!s_playlist.empty()) {
            s_totalSeconds = s_playlist[s_currentTrack].durationSec;
            s_currentSeconds = 0;
        }
        Serial.printf("🎉 [ESP32-MUSIC] SD卡歌单同步完毕 (共 %d 首歌曲)\r\n", (int)s_playlist.size());
    } else if (line.startsWith("STATUS:")) {
        // STATUS:state,cur_sec,total_sec,vol,mode,track_id
        String params = line.substring(7);
        int p0 = params.indexOf(',');
        int p1 = params.indexOf(',', p0 + 1);
        int p2 = params.indexOf(',', p1 + 1);
        int p3 = params.indexOf(',', p2 + 1);
        int p4 = params.indexOf(',', p3 + 1);
        if (p0 > 0 && p1 > p0 && p2 > p1) {
            PlayState oldState = s_playState;
            int oldTrack = s_currentTrack;
            s_playState = static_cast<PlayState>(params.substring(0, p0).toInt());
            s_totalSeconds = params.substring(p1 + 1, p2).toInt();
            if (p3 > 0) s_volume = params.substring(p2 + 1, p3).toInt();
            if (p4 > 0) s_playMode = static_cast<PlayMode>(params.substring(p3 + 1, p4).toInt());
            if (p4 > 0) {
                int track = params.substring(p4 + 1).toInt();
                if (track >= 0 && track < (int)s_playlist.size()) s_currentTrack = track;
            }
            if (oldTrack != s_currentTrack) {
                prepareForNewTrack();
            }
            if (oldState != s_playState || oldTrack != s_currentTrack) {
                Serial.printf("📊 [ESP32-MUSIC] 状态变更 -> 状态:%d, 进度:%02d:%02d/%02d:%02d, 曲目:#%02d\r\n", 
                              static_cast<int>(s_playState), s_currentSeconds / 60, s_currentSeconds % 60, 
                              s_totalSeconds / 60, s_totalSeconds % 60, s_currentTrack + 1);
            }
        }
    } else if (line.startsWith("LRC:CLEAR")) {
        s_local_lyrics.clear();
        Serial.println("📜 [ESP32-MUSIC] 开始接收 STM32 完整本地歌词表 (LRC:CLEAR)");
    } else if (line.startsWith("LRC:DONE")) {
        CustomUiEngine::notifyUiNeedsUpdate();
        Serial.printf("📜 [ESP32-MUSIC] 成功接收并缓存整首完整歌词 (共 %d 行)！\r\n", (int)s_local_lyrics.size());
    } else if (line.startsWith("LRC:")) {
        // LRC:<time_ms>,<text>
        int comma = line.indexOf(',');
        if (comma > 4) {
            uint32_t t_ms = (uint32_t)line.substring(4, comma).toInt();
            String text = line.substring(comma + 1);
            s_local_lyrics.push_back({t_ms, text});
        }
    } else if (line.startsWith("RATE:")) {
        uint32_t rate = line.substring(5).toInt();
        if (rate >= 8000 && rate <= 96000) {
            s_music_sample_rate = rate;
            if (s_i2s_active) {
                AudioService::setSampleRate(rate);
            }
            Serial.printf("🎯 [ESP32-MUSIC] 收到 STM32 音频采样率更新: %u Hz (APLL 锁相环已同步锁定)\r\n", (unsigned)rate);
        }
    } else {
        bool printable = true;
        for (size_t i = 0; i < line.length() && i < 20; i++) {
            if ((uint8_t)line[i] < 32 && line[i] != '\t' && line[i] != '\r' && line[i] != '\n') {
                printable = false;
                break;
            }
        }
        if (printable && line.length() > 0) {
            Serial.printf("📥 [ESP32-MUSIC-RX] %s\r\n", line.c_str());
        }
    }
}

void update() {
    const uint32_t now = millis();

    // 1. 干净高效的 UART1 文本控制行解析器 (115200 Baud)
    static char rx_buf[512];
    static size_t rx_idx = 0;

    while (MusicSerial.available() > 0) {
        char c = (char)MusicSerial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            if (rx_idx > 0) {
                rx_buf[rx_idx] = '\0';
                processIncomingLine(String(rx_buf));
                rx_idx = 0;
            }
        } else {
            if (rx_idx < sizeof(rx_buf) - 1) {
                rx_buf[rx_idx++] = c;
            }
        }
    }

    // 2. 本地平滑模拟计时与动态频谱律动 (保持 UI 丝滑高帧率响应)
    if (s_playState == STATE_PLAYING) {
        static uint32_t last_spi_rx_log_ms = 0;
        if (now - last_spi_rx_log_ms >= 2000) {
            last_spi_rx_log_ms = now;
            Serial.printf("🔊 [ESP32-NS4168] 累计接收PCM: %u KB, 当前水库余量: %u 字节 (%d/%d帧), 累计硬件播发: %u KB (曲目:#%02d, 进度:%02d:%02d/%02d:%02d, 音量:%d%%)\r\n",
                          s_total_pcm_rx_bytes / 1024, (unsigned)frame_queue_available_bytes(), (int)s_frame_ready_count, (int)NUM_FRAME_SLOTS,
                          s_total_pcm_played_bytes / 1024,
                          s_currentTrack + 1, s_currentSeconds / 60, s_currentSeconds % 60, 
                          s_totalSeconds / 60, s_totalSeconds % 60, s_volume);
        }

        // 🌟 真实物理采样时钟：直接根据 I2S DMA 实际写入扬声器的 PCM 采样总字节数计算物理播放进度 (秒)
        uint32_t real_played_sec = s_total_pcm_played_bytes / (sizeof(int16_t) * (s_music_sample_rate > 0 ? s_music_sample_rate : 44100));
        s_currentSeconds = (int)real_played_sec;

        // 🌟 动态跳动频谱生成：实时响应真实 PCM 音频 FFT 能量（极速上升，敏捷弹跳）
        if (now - s_last_spectrum_ms >= 20) {
            s_last_spectrum_ms = now;
            if (s_fft_snapshot_ready) {
                s_fft_snapshot_ready = false;
                computeRealPcmSpectrum(s_fft_samples_snapshot, FFT_N);
            }
            for (int i = 0; i < 16; i++) {
                uint8_t target = s_target_spectrum[i];
                if (target > s_spectrum[i]) {
                    // 极速上升 (Attack)：瞬间跟上音乐节拍与重音
                    s_spectrum[i] = target;
                } else {
                    // 敏捷弹性回落 (Decay / 鲜明律动)：每周期下落 12，起伏生动自然，绝不迟滞
                    if (s_spectrum[i] > 12) s_spectrum[i] -= 12;
                    else s_spectrum[i] = 0;
                }

                // 峰值悬停顶针 (Peak Hold & Falloff)
                if (s_spectrum[i] >= s_spectrum_peaks[i]) {
                    s_spectrum_peaks[i] = s_spectrum[i];
                    s_peak_hold[i] = 3; // 悬停 3 周期 (约 60ms)
                } else {
                    if (s_peak_hold[i] > 0) {
                        s_peak_hold[i]--;
                    } else if (s_spectrum_peaks[i] > 4) {
                        s_spectrum_peaks[i] -= 4; // 平滑缓慢下落
                    } else {
                        s_spectrum_peaks[i] = 0;
                    }
                }
            }
        }
    } else {
        // 暂停/停止时频谱快速归零
        if (now - s_last_spectrum_ms >= 20) {
            s_last_spectrum_ms = now;
            for (int i = 0; i < 16; i++) {
                if (s_spectrum[i] > 15) s_spectrum[i] -= 15;
                else s_spectrum[i] = 0;
                if (s_spectrum_peaks[i] > 10) s_spectrum_peaks[i] -= 10;
                else s_spectrum_peaks[i] = 0;
                s_target_spectrum[i] = 0;
            }
        }
    }
}

PlayState getPlayState() { return s_playState; }
bool isPlaying() { return s_playState == STATE_PLAYING; }
bool isPaused() { return s_playState == STATE_PAUSED; }
int getCurrentTrackIndex() { return s_currentTrack; }

const SongItem* getCurrentSong() {
    static const SongItem s_placeholder = {0, "歌曲名", "歌手名", 0};
    if (s_playlist.empty()) return &s_placeholder;
    if (s_currentTrack < 0 || s_currentTrack >= (int)s_playlist.size()) return &s_playlist[0];
    return &s_playlist[s_currentTrack];
}

int getCurrentSeconds() { return s_currentSeconds; }
int getTotalSeconds() { return s_totalSeconds; }
int getSavedPauseSeconds() { return s_saved_pause_seconds; }
int getVolume() { return s_volume; }
PlayMode getPlayMode() { return s_playMode; }

bool getPlayingReferenceSamples(int16_t out_samples[256]) {
    if (s_playState != STATE_PLAYING || !s_i2s_active) return false;
    memcpy(out_samples, s_fft_samples_snapshot, sizeof(int16_t) * 256);
    return true;
}

const char* getPlayModeName() {
    switch (s_playMode) {
        case MODE_SEQUENCE: return "顺序";
        case MODE_SINGLE_REPEAT: return "单曲";
        case MODE_RANDOM: return "随机";
        default: return "顺序";
    }
}

String getCurrentLyric() { return s_currentLyric; }

void getLyrics3LinesForTime(int seconds, String& outPrev, String& outCurr, String& outNext) {
    if (s_local_lyrics.empty()) {
        outPrev = "";
        outCurr = (s_playlist.empty() ? "暂无歌词 · 点击连接获取歌单" : "正在载入歌词...");
        outNext = "";
        return;
    }

    uint32_t ms = (seconds < 0 ? 0 : (uint32_t)seconds) * 1000;
    int count = (int)s_local_lyrics.size();
    int idx = 0;

    if (count <= 5) {
        // 动态短占位歌词
        uint32_t curMs = (seconds % 20) * 1000;
        for (int i = 0; i < count; i++) {
            if (curMs >= s_local_lyrics[i].timeMs) idx = i;
            else break;
        }
        int p_idx = (idx - 1 + count) % count;
        int n_idx = (idx + 1) % count;
        outPrev = s_local_lyrics[p_idx].text;
        outCurr = s_local_lyrics[idx].text;
        outNext = s_local_lyrics[n_idx].text;
        return;
    }

    // 真实完整多行时间轴歌词 (毫秒级精准对齐)
    for (int i = 0; i < count; i++) {
        if (ms >= s_local_lyrics[i].timeMs) {
            idx = i;
        } else {
            break;
        }
    }

    outPrev = (idx > 0) ? s_local_lyrics[idx - 1].text : "";
    outCurr = s_local_lyrics[idx].text;
    outNext = (idx + 1 < count) ? s_local_lyrics[idx + 1].text : "";
}

void getLyrics3Lines(String& outPrev, String& outCurr, String& outNext) {
    getLyrics3LinesForTime(s_currentSeconds, outPrev, outCurr, outNext);
}

const std::vector<SongItem>& getPlaylist() { return s_playlist; }
int getPlaylistCount() { return (int)s_playlist.size(); }

const SongItem* getSong(int index) {
    if (index < 0 || index >= (int)s_playlist.size()) return nullptr;
    return &s_playlist[index];
}

void getSpectrumBars(uint8_t outBars[16]) {
    for (int i = 0; i < 16; i++) outBars[i] = s_spectrum[i];
}

void getSpectrumPeaks(uint8_t outPeaks[16]) {
    for (int i = 0; i < 16; i++) outPeaks[i] = s_spectrum_peaks[i];
}

// ----------------- 控制指令下发 -----------------

void sendPlay(int trackId) {
    if (CameraService::state() != CameraService::State::IDLE) {
        CameraService::stop();
        Serial.println("📷 [ESP32-MUSIC] ⚡ SPI互斥: 检测到摄像头正在运行，已自动关闭摄像头以抢占 SPI2 总线");
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (!s_playlist.empty() && trackId >= 0 && trackId < (int)s_playlist.size()) {
        s_currentTrack = trackId;
        s_totalSeconds = s_playlist[trackId].durationSec;
        s_currentSeconds = 0;
    }
    s_playState = STATE_PLAYING;
    prepareForNewTrack();
    MusicSerial.printf("CMD:PLAY,%d\n", trackId);
    CustomUiEngine::notifyUiNeedsUpdate();
    Serial.printf("📤 [ESP32-MUSIC-TX] 发送播放指定曲目指令: CMD:PLAY,%d\r\n", trackId);
}

void sendPlay() {
    if (CameraService::state() != CameraService::State::IDLE) {
        CameraService::stop();
        Serial.println("📷 [ESP32-MUSIC] ⚡ SPI互斥: 检测到摄像头正在运行，已自动关闭摄像头以抢占 SPI2 总线");
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    s_playState = STATE_PLAYING;
    MusicSerial.println("CMD:PLAY");
    CustomUiEngine::notifyUiNeedsUpdate();
    Serial.println("📤 [ESP32-MUSIC-TX] 发送播放指令: CMD:PLAY");
}

void sendTogglePlayPause() {
    if (s_playState == STATE_PLAYING) {
        sendPause();
    } else if (s_playState == STATE_PAUSED) {
        sendResume();
    } else {
        sendPlay(s_currentTrack);
    }
}

void sendPause() {
    s_playState = STATE_PAUSED;
    // 🌟 记录当前暂停时刻的秒数与曲目
    s_saved_pause_seconds = s_currentSeconds;
    s_saved_pause_track = s_currentTrack;

    // 🌟 深度清空残余旧水库与 DMA 描述符，杜绝唤醒或后续恢复时播发残存的旧杂音
    frame_queue_clear();
    AudioService::resetDma();
    s_need_prebuffer = true;
    s_was_in_silence = true;

    MusicSerial.println("CMD:PAUSE");
    CustomUiEngine::notifyUiNeedsUpdate();
    Serial.printf("📤 [ESP32-MUSIC-TX] 发送暂停指令: CMD:PAUSE (已记录进度 %02d:%02d 并深度清空旧水库)\r\n", 
                  s_saved_pause_seconds / 60, s_saved_pause_seconds % 60);
}

void sendStop() {
    s_playState = STATE_STOPPED;
    s_saved_pause_seconds = 0;
    prepareForNewTrack();
    MusicSerial.println("CMD:STOP");
    CustomUiEngine::notifyUiNeedsUpdate();
    Serial.println("📤 [ESP32-MUSIC-TX] 发送停止指令: CMD:STOP");
}

void sendResume() {
    if (CameraService::state() != CameraService::State::IDLE) {
        CameraService::stop();
        Serial.println("📷 [ESP32-MUSIC] ⚡ SPI互斥: 检测到摄像头正在运行，已自动关闭摄像头以抢占 SPI2 总线");
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    s_playState = STATE_PLAYING;

    // 🌟 重新开辟纯净水库，并置位预缓冲标志，蓄满 120 帧 (3.13秒) 后平滑淡入开播
    frame_queue_clear();
    AudioService::resetDma();
    s_need_prebuffer = true;
    s_was_in_silence = true;

    MusicSerial.println("CMD:RESUME");
    CustomUiEngine::notifyUiNeedsUpdate();
    Serial.printf("📤 [ESP32-MUSIC-TX] 发送恢复播放指令: CMD:RESUME (从 %02d:%02d 重新加载水库开播)\r\n",
                  s_currentSeconds / 60, s_currentSeconds % 60);
}

void sendNext() {
    if (CameraService::state() != CameraService::State::IDLE) {
        CameraService::stop();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (!s_playlist.empty()) {
        s_currentTrack = (s_currentTrack + 1) % (int)s_playlist.size();
        s_totalSeconds = s_playlist[s_currentTrack].durationSec;
        s_currentSeconds = 0;
    }
    s_playState = STATE_PLAYING;
    prepareForNewTrack();
    MusicSerial.println("CMD:NEXT");
    CustomUiEngine::notifyUiNeedsUpdate();
    Serial.printf("📤 [ESP32-MUSIC-TX] 发送下一首指令: CMD:NEXT (已切换至 #%02d)\r\n", s_currentTrack + 1);
}

void sendPrev() {
    if (CameraService::state() != CameraService::State::IDLE) {
        CameraService::stop();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (!s_playlist.empty()) {
        s_currentTrack = (s_currentTrack - 1 + (int)s_playlist.size()) % (int)s_playlist.size();
        s_totalSeconds = s_playlist[s_currentTrack].durationSec;
        s_currentSeconds = 0;
    }
    s_playState = STATE_PLAYING;
    prepareForNewTrack();
    MusicSerial.println("CMD:PREV");
    CustomUiEngine::notifyUiNeedsUpdate();
    Serial.printf("📤 [ESP32-MUSIC-TX] 发送上一首指令: CMD:PREV (已切换至 #%02d)\r\n", s_currentTrack + 1);
}

void sendSelectTrack(int index) {
    if (CameraService::state() != CameraService::State::IDLE) {
        CameraService::stop();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (!s_playlist.empty() && index >= 0 && index < (int)s_playlist.size()) {
        s_currentTrack = index;
        s_totalSeconds = s_playlist[s_currentTrack].durationSec;
        s_currentSeconds = 0;
    }
    s_playState = STATE_PLAYING;
    prepareForNewTrack();
    MusicSerial.printf("CMD:SELECT,%d\n", index);
    CustomUiEngine::notifyUiNeedsUpdate();
    Serial.printf("📤 [ESP32-MUSIC-TX] 发送切歌指令: CMD:SELECT,%d\r\n", index);
}

void sendSeek(int seconds) {
    if (seconds < 0) seconds = 0;
    if (s_totalSeconds > 0 && seconds > s_totalSeconds) seconds = s_totalSeconds;
    s_currentSeconds = seconds;
    prepareForNewTrack(); // 🌟 瞬间清空 ESP32 现有的 224 帧旧音频槽位水库并重置预缓冲门限，无缝秒接新时间点！
    MusicSerial.printf("CMD:SEEK,%d\n", seconds);
    Serial.printf("📤 [ESP32-MUSIC-TX] 发送快进快退: CMD:SEEK,%d\r\n", seconds);
}

void sendVolume(int volume) {
    if (volume < 0) volume = 0;
    if (volume > 100) volume = 100;
    s_volume = volume;
    MusicSerial.printf("CMD:VOL,%d\n", volume);
    Serial.printf("📤 [ESP32-MUSIC-TX] 发送音量调节: CMD:VOL,%d\r\n", volume);
}

void sendTogglePlayMode() {
    s_playMode = static_cast<PlayMode>((static_cast<int>(s_playMode) + 1) % 3);
    MusicSerial.printf("CMD:MODE,%d\n", static_cast<int>(s_playMode));
    Serial.printf("📤 [ESP32-MUSIC-TX] 发送播放模式切换: CMD:MODE,%d (%s)\r\n", static_cast<int>(s_playMode), getPlayModeName());
}

void sendRequestPlaylist() {
    MusicSerial.println("CMD:GET_LIST");
    Serial.println("📤 [ESP32-MUSIC-TX] 发送获取歌单指令: CMD:GET_LIST");
}

void sendRawCommand(const String& cmd) {
    MusicSerial.println(cmd);
    Serial.printf("📤 [ESP32-CMD-TX] 发送通用控制指令: %s\r\n", cmd.c_str());
}

bool isConnected() {
    return s_connected;
}

void sendConnect() {
    // 🌟 若此前曾断开释放过水库，点击连接时动态重新开辟 224 帧原生帧锁相队列
    if (!s_frame_slots) {
        if (psramFound()) {
            s_frame_slots = (FrameSlot*)ps_malloc(NUM_FRAME_SLOTS * sizeof(FrameSlot));
            if (s_frame_slots) {
                Serial.printf("✨ [ESP32-MUSIC] 连接激活: 在 4MB PSRAM 中动态开辟 224 帧原生帧锁相队列 (共 %u KB)！\r\n", 
                              (unsigned)(NUM_FRAME_SLOTS * sizeof(FrameSlot) / 1024));
            }
        }
        if (!s_frame_slots) {
            s_frame_slots = (FrameSlot*)malloc(NUM_FRAME_SLOTS * sizeof(FrameSlot));
        }
    }
    frame_queue_clear();
    s_need_prebuffer = true;

    MusicSerial.println("CMD:CONNECT");
    Serial.println("📤 [ESP32-MUSIC-TX] 点击【连接】，已向 STM32 下发 CMD:CONNECT 指令！");
}

void sendDisconnect() {
    // 1. 向 STM32 下发断开与停止播放指令
    MusicSerial.println("CMD:DISCONNECT");
    MusicSerial.println("CMD:STOP");
    Serial.println("📤 [ESP32-MUSIC-TX] 点击【断开】，已向 STM32 下发 CMD:DISCONNECT 指令！");

    // 2. 彻底重置播放状态与所有业务数据
    s_connected = false;
    s_playState = STATE_STOPPED;
    s_currentTrack = 0;
    s_currentSeconds = 0;
    s_totalSeconds = 0;
    s_currentLyric = "暂无歌词 · 点击连接获取歌单";
    memset((void*)s_spectrum, 0, sizeof(s_spectrum));
    memset((void*)s_target_spectrum, 0, sizeof(s_target_spectrum));
    s_playlist.clear();
    s_playlist.shrink_to_fit();

    // 3. 复位硬件 DMA 与音频时钟引擎
    AudioService::resetDma();
    AudioService::sleep();
    s_i2s_active = false;

    // 4. 彻底释放 224 帧 (504KB) PSRAM 槽位队列，归还给系统
    if (s_frame_slots) {
        free(s_frame_slots);
        s_frame_slots = nullptr;
        Serial.println("🧹 [ESP32-MUSIC] 成功释放 224 帧 (共 504KB) PSRAM 原生帧锁相队列！");
    }
    frame_queue_clear();

    // 5. 暂存跳板内存彻底清零
    memset(s_stream_stereo_dma_chunk, 0, sizeof(s_stream_stereo_dma_chunk));
    memset(s_spi_rx_buf, 0, sizeof(s_spi_rx_buf));
    s_need_prebuffer = true;

    Serial.println("🛑 [ESP32-MUSIC] 音乐播放功能已完全断开并清理所有占用内存！");
}

} // namespace MusicPlayerService
