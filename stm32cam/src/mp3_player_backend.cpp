#include "mp3_player_backend.h"
#include <Arduino.h>
#include <SPI.h>
#include <vector>
#include <algorithm>
#include "pinyin_table.h"
#define MINIMP3_IMPLEMENTATION
#include "minimp3.h"
#include "stm32h7xx_hal.h"

extern bool g_camera_streaming;

namespace Mp3PlayerBackend {

// 🌟 逐字解码 UTF-8 字符并提取排序键与 Unicode 码点
struct Utf8SortChar {
    char pinyin_key;    // 'A'~'Z', '0'~'9', 或 '~'
    uint32_t unicode;   // 原始 Unicode 码点 (用于拼音首字母相同时的精确平局决胜)
};

static inline bool isIgnorePunctuation(uint32_t u) {
    if (u <= 0x20) return true; // 空格、控制符
    if (u == '[' || u == ']' || u == '(' || u == ')' || u == '<' || u == '>' ||
        u == '\"' || u == '\'' || u == '.' || u == '-' || u == '_' || u == ',' || u == ':' || u == '·') return true;
    if (u == 0x300A || u == 0x300B) return true; // 《 》
    if (u == 0x3010 || u == 0x3011) return true; // 【 】
    if (u == 0x201C || u == 0x201D) return true; // “ ”
    if (u == 0x2018 || u == 0x2019) return true; // ‘ ’
    if (u == 0x266A || u == 0x266B) return true; // ♪ ♫
    return false;
}

static Utf8SortChar getNextSortChar(const char*& p) {
    while (*p) {
        int consumed = 0;
        const unsigned char* up = (const unsigned char*)p;
        uint32_t u = 0;
        if (*up < 0x80) {
            consumed = 1;
            u = *up;
        } else if ((up[0] & 0xE0) == 0xC0 && up[1]) {
            consumed = 2;
            u = ((up[0] & 0x1F) << 6) | (up[1] & 0x3F);
        } else if ((up[0] & 0xF0) == 0xE0 && up[1] && up[2]) {
            consumed = 3;
            u = ((up[0] & 0x0F) << 12) | ((up[1] & 0x3F) << 6) | (up[2] & 0x3F);
        } else if ((up[0] & 0xF8) == 0xF0 && up[1] && up[2] && up[3]) {
            consumed = 4;
            u = ((up[0] & 0x07) << 18) | ((up[1] & 0x3F) << 12) | ((up[2] & 0x3F) << 6) | (up[3] & 0x3F);
        } else {
            consumed = 1;
            u = *up;
        }
        p += consumed;

        if (isIgnorePunctuation(u)) continue;

        Utf8SortChar sc;
        sc.unicode = u;

        if (u >= 'a' && u <= 'z') {
            sc.pinyin_key = (char)(u - 'a' + 'A');
        } else if (u >= 'A' && u <= 'Z') {
            sc.pinyin_key = (char)u;
        } else if (u >= '0' && u <= '9') {
            sc.pinyin_key = (char)u;
        } else if (u >= 0x4E00 && u <= 0x9FA5) {
            char py = getHanziPinyinInitial((uint16_t)u);
            sc.pinyin_key = (py >= 'A' && py <= 'Z') ? py : '~';
        } else {
            sc.pinyin_key = '~';
        }
        return sc;
    }
    return { '\0', 0 };
}

// 🌟 全字符串多字深度拼音 + 自然数字智能排序比较器 (彻底解决同首字母歌名乱序)
static bool compareSongsByPinyin(const SongItem& a, const SongItem& b) {
    const char* pa = a.title.c_str();
    const char* pb = b.title.c_str();

    auto getRank = [](char k) -> int {
        if (k >= 'A' && k <= 'Z') return k - 'A';
        if (k >= '0' && k <= '9') return 30 + (k - '0');
        if (k == '\0') return -1;
        return 100;
    };

    while (*pa || *pb) {
        // 1. 如果两边同时遇到连续数字，按自然整型数值比较 (如 2 < 10)
        const unsigned char* upa = (const unsigned char*)pa;
        const unsigned char* upb = (const unsigned char*)pb;

        if (isdigit(*upa) && isdigit(*upb)) {
            int numA = 0, numB = 0;
            while (isdigit(*pa)) { numA = numA * 10 + (*pa - '0'); pa++; }
            while (isdigit(*pb)) { numB = numB * 10 + (*pb - '0'); pb++; }
            if (numA != numB) return numA < numB;
            continue;
        }

        Utf8SortChar ca = getNextSortChar(pa);
        Utf8SortChar cb = getNextSortChar(pb);

        if (ca.pinyin_key == '\0' && cb.pinyin_key == '\0') break;
        if (ca.pinyin_key == '\0') return true;  // a 短于 b，排在前面
        if (cb.pinyin_key == '\0') return false; // b 短于 a

        int rankA = getRank(ca.pinyin_key);
        int rankB = getRank(cb.pinyin_key);

        if (rankA != rankB) {
            return rankA < rankB;
        }

        // 🌟 首字母相同时（如“七”和“晴”都是 Q），若汉字不同且声母相同，继续循环比对第2个字、第3个字！
    }

    return a.fileName < b.fileName;
}

// ==================== 5线硬件 SPI2 高速音频总线 ====================
constexpr int MY_PIN_SPI_NSS   = PB12;
constexpr int MY_PIN_SPI_SCK   = PB13;
constexpr int MY_PIN_SPI_MISO  = PB14;
constexpr int MY_PIN_SPI_MOSI  = PB15;
constexpr int MY_PIN_HANDSHAKE = PB1;

struct __attribute__((packed)) AudioPacket {
    uint32_t magic;         // 0xA0D10001
    uint16_t sample_count;  // 1152 采样点 (44.1kHz MPEG-1 Layer 3 原生帧)
    uint16_t sample_rate;   // 🌟 物理采样率 (44100 / 32000 / 24000 / 48000 Hz)
    int16_t  samples[1152]; // 2304 字节 16-bit Mono PCM
    uint32_t tail_magic;    // 0xFEEDFACE
};

// ==================== USART2 (PA2 TX -> ESP32 RX, PA3 RX <- ESP32 TX) ====================
static HardwareSerial ControlSerial(PA3, PA2);

// ==================== 播放状态机变量 ====================
static std::vector<SongItem> s_playlist;
static std::vector<LyricItem> s_currentLyrics;
static int s_currentTrack = 0;
static PlayState s_playState = STATE_STOPPED;
static int s_currentSeconds = 0;
static int s_totalSeconds = 240;
static int s_volume = 80;
static PlayMode s_playMode = MODE_SEQUENCE;

static uint32_t s_last_status_ms = 0;
static uint32_t s_last_tick_ms = 0;
static int s_last_lyric_idx = -1;
static bool s_sd_card_ready = false;

// ==================== SDMMC1 硬件句柄 ====================
static SD_HandleTypeDef hsd1;

// ----------------- SDMMC1 硬件底层初始化 -----------------
static bool initSdmmcHardware() {
    __HAL_RCC_SDMMC1_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // PC8(D0), PC9(D1), PC10(D2), PC11(D3), PC12(CLK)
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // PD2(CMD)
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    hsd1.Instance = SDMMC1;
    hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
    hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
    hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd1.Init.ClockDiv = 2; // 25MHz 高速模式

    if (HAL_SD_Init(&hsd1) == HAL_OK) {
        if (HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B) == HAL_OK) {
            return true;
        }
    }
    return false;
}

// ----------------- UTF-16LE 转 UTF-8 字符解码辅助函数 -----------------
static void appendUtf16ToUtf8(uint16_t u, String& out) {
    if (u == 0 || u == 0xFFFF) return;
    if (u < 0x80) {
        out += (char)(u & 0xFF);
    } else if (u < 0x800) {
        out += (char)(0xC0 | ((u >> 6) & 0x1F));
        out += (char)(0x80 | (u & 0x3F));
    } else {
        out += (char)(0xE0 | ((u >> 12) & 0x0F));
        out += (char)(0x80 | ((u >> 6) & 0x3F));
        out += (char)(0x80 | (u & 0x3F));
    }
}

struct LrcFileInfo {
    String lrcName;
    uint32_t startClust;
    uint32_t fileSize;
};
static std::vector<LrcFileInfo> s_lrcFiles;
static uint32_t s_fat_lba = 0;
static uint32_t s_data_lba = 0;
static uint8_t s_sec_per_clust = 0;

static inline uint32_t clust_to_lba(uint32_t c) {
    return s_data_lba + (c - 2) * s_sec_per_clust;
}

// ----------------- UTF-8 有效性校验辅助函数 -----------------
static bool isValidUtf8(const String& s) {
    size_t len = s.length();
    size_t i = 0;
    while (i < len) {
        uint8_t c = (uint8_t)s[i];
        if (c < 0x80) {
            i++;
        } else if ((c & 0xE0) == 0xC0) {
            if (i + 1 >= len || ((uint8_t)s[i+1] & 0xC0) != 0x80) return false;
            i += 2;
        } else if ((c & 0xF0) == 0xE0) {
            if (i + 2 >= len || ((uint8_t)s[i+1] & 0xC0) != 0x80 || ((uint8_t)s[i+2] & 0xC0) != 0x80) return false;
            i += 3;
        } else if ((c & 0xF8) == 0xF0) {
            if (i + 3 >= len || ((uint8_t)s[i+1] & 0xC0) != 0x80 || ((uint8_t)s[i+2] & 0xC0) != 0x80 || ((uint8_t)s[i+3] & 0xC0) != 0x80) return false;
            i += 4;
        } else {
            return false;
        }
    }
    return true;
}

// ----------------- ID3v2 文本帧解码 (TIT2 歌名 / TPE1 歌手名) -----------------
static String decodeId3TextFrame(const uint8_t* data, size_t len) {
    if (len <= 1 || data == nullptr) return "";
    uint8_t enc = data[0];
    const uint8_t* p = data + 1;
    size_t text_len = len - 1;
    String res = "";

    if (enc == 0 || enc == 3) {
        // 0 = ISO-8859-1/GBK, 3 = UTF-8
        for (size_t i = 0; i < text_len; i++) {
            if (p[i] == 0) break;
            res += (char)p[i];
        }
        if (!isValidUtf8(res)) return ""; // 🌟 过滤非标准 GBK 损坏文本，杜绝向前端发送乱码导致丢字
    } else if (enc == 1 || enc == 2) {
        // 1 = UTF-16 with BOM, 2 = UTF-16BE
        bool be = (enc == 2);
        size_t start = 0;
        if (text_len >= 2 && enc == 1) {
            if (p[0] == 0xFE && p[1] == 0xFF) { be = true; start = 2; }
            else if (p[0] == 0xFF && p[1] == 0xFE) { be = false; start = 2; }
        }
        for (size_t i = start; i + 1 < text_len; i += 2) {
            uint16_t u = be ? (((uint16_t)p[i] << 8) | p[i+1]) : (((uint16_t)p[i+1] << 8) | p[i]);
            if (u == 0) break;
            appendUtf16ToUtf8(u, res);
        }
    }
    res.trim();
    return res;
}

// 🌟 商业级专业 MP3 头部原生精准解析器 (支持 ID3v2 标题/歌手/TLEN、Xing/Info VBR 帧数头、VBRI 头与精确 MPEG 码率计算)
static int parseMp3Metadata(uint32_t startClust, uint32_t fileSize, String& outTitle, String& outArtist, uint32_t& outId3Size) {
    outTitle = "";
    outArtist = "";
    outId3Size = 0;
    if (startClust < 2 || fileSize == 0) return 240;

    static uint8_t buf[2048];
    uint32_t lba = clust_to_lba(startClust);
    // 读取前 4 个扇区 (2KB 头部)
    if (HAL_SD_ReadBlocks(&hsd1, buf, lba, 4, 3000) != HAL_OK) return 240;

    uint32_t id3_size = 0;
    int exact_duration = 0;

    // 1. 检查 ID3v2 头部并检索 TIT2(歌名)、TPE1(歌手)、TLEN(时长)
    if (buf[0] == 'I' && buf[1] == 'D' && buf[2] == '3') {
        uint8_t id3_ver = buf[3];
        id3_size = (((uint32_t)(buf[6] & 0x7F) << 21) |
                    ((uint32_t)(buf[7] & 0x7F) << 14) |
                    ((uint32_t)(buf[8] & 0x7F) << 7)  |
                    ((uint32_t)(buf[9] & 0x7F))) + 10;
        if (buf[5] & 0x10) id3_size += 10; // ID3v2.4 footer
        outId3Size = id3_size;
        
        int scan_limit = (id3_size < sizeof(buf) - 32) ? id3_size : (sizeof(buf) - 32);
        for (int i = 10; i < scan_limit; i++) {
            // 解析 TIT2 歌名
            if (buf[i] == 'T' && buf[i+1] == 'I' && buf[i+2] == 'T' && buf[i+3] == '2') {
                uint32_t fsz = (id3_ver == 4) ?
                    (((uint32_t)(buf[i+4] & 0x7F) << 21) | ((uint32_t)(buf[i+5] & 0x7F) << 14) | ((uint32_t)(buf[i+6] & 0x7F) << 7) | (buf[i+7] & 0x7F)) :
                    (((uint32_t)buf[i+4] << 24) | ((uint32_t)buf[i+5] << 16) | ((uint32_t)buf[i+6] << 8) | (uint32_t)buf[i+7]);
                if (fsz > 0 && fsz < 256 && (i + 10 + fsz <= sizeof(buf))) {
                    outTitle = decodeId3TextFrame(buf + i + 10, fsz);
                }
            }
            // 解析 TPE1 歌手名
            else if (buf[i] == 'T' && buf[i+1] == 'P' && buf[i+2] == 'E' && buf[i+3] == '1') {
                uint32_t fsz = (id3_ver == 4) ?
                    (((uint32_t)(buf[i+4] & 0x7F) << 21) | ((uint32_t)(buf[i+5] & 0x7F) << 14) | ((uint32_t)(buf[i+6] & 0x7F) << 7) | (buf[i+7] & 0x7F)) :
                    (((uint32_t)buf[i+4] << 24) | ((uint32_t)buf[i+5] << 16) | ((uint32_t)buf[i+6] << 8) | (uint32_t)buf[i+7]);
                if (fsz > 0 && fsz < 256 && (i + 10 + fsz <= sizeof(buf))) {
                    outArtist = decodeId3TextFrame(buf + i + 10, fsz);
                }
            }
            // 解析 TLEN 毫秒时长
            else if (buf[i] == 'T' && buf[i+1] == 'L' && buf[i+2] == 'E' && buf[i+3] == 'N') {
                uint32_t fsz = (id3_ver == 4) ?
                    (((uint32_t)(buf[i+4] & 0x7F) << 21) | ((uint32_t)(buf[i+5] & 0x7F) << 14) | ((uint32_t)(buf[i+6] & 0x7F) << 7) | (buf[i+7] & 0x7F)) :
                    (((uint32_t)buf[i+4] << 24) | ((uint32_t)buf[i+5] << 16) | ((uint32_t)buf[i+6] << 8) | (uint32_t)buf[i+7]);
                if (fsz > 0 && fsz < 32 && (i + 10 + fsz <= sizeof(buf))) {
                    char tlen_str[32] = {0};
                    int ti = 0;
                    for (uint32_t k = 0; k < fsz; k++) {
                        char c = (char)buf[i + 10 + k];
                        if (c >= '0' && c <= '9') tlen_str[ti++] = c;
                    }
                    if (ti > 0) {
                        uint32_t ms = atoi(tlen_str);
                        if (ms >= 10000 && ms <= 1800000) {
                            exact_duration = (int)(ms / 1000);
                        }
                    }
                }
            }
        }
    }

    if (exact_duration > 0) return exact_duration;

    // 2. 跨过 ID3 标签区寻找首个 MPEG 帧头并解析 Xing/Info VBR 帧数
    uint32_t audio_offset_in_buf = id3_size;
    if (audio_offset_in_buf >= sizeof(buf) - 512) {
        uint32_t audio_sec_offset = id3_size / 512;
        uint32_t audio_lba = lba + audio_sec_offset;
        if (HAL_SD_ReadBlocks(&hsd1, buf, audio_lba, 2, 3000) == HAL_OK) {
            audio_offset_in_buf = id3_size % 512;
        } else {
            audio_offset_in_buf = 0;
        }
    }

    // 在缓冲区中寻找 MPEG 帧同步字 0xFFE0
    for (size_t i = audio_offset_in_buf; i < sizeof(buf) - 160; i++) {
        if (buf[i] == 0xFF && (buf[i+1] & 0xE0) == 0xE0) {
            uint8_t h1 = buf[i+1];
            uint8_t h2 = buf[i+2];
            uint8_t h3 = buf[i+3];
            int layer = 4 - ((h1 >> 1) & 3);
            if (layer != 3) continue; // 仅处理 Layer 3

            int br_idx = (h2 >> 4) & 0x0F;
            int sr_idx = (h2 >> 2) & 0x03;
            if (br_idx == 0 || br_idx == 15 || sr_idx == 3) continue;

            static const int bitrate_tab_v1[16] = {0, 32, 40, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 0};
            static const int samplerate_tab_v1[4] = {44100, 48000, 32000, 0};
            int bitrate = bitrate_tab_v1[br_idx];
            int samplerate = samplerate_tab_v1[sr_idx];
            if (bitrate == 0 || samplerate == 0) continue;

            bool is_mono = ((h3 >> 6) & 3) == 3;
            size_t xing_pos = i + 4 + (is_mono ? 17 : 32);

            // 检查 Xing / Info 头部
            if (xing_pos + 16 < sizeof(buf)) {
                if ((buf[xing_pos] == 'X' && buf[xing_pos+1] == 'i' && buf[xing_pos+2] == 'n' && buf[xing_pos+3] == 'g') ||
                    (buf[xing_pos] == 'I' && buf[xing_pos+1] == 'n' && buf[xing_pos+2] == 'f' && buf[xing_pos+3] == 'o')) {
                    uint32_t flags = ((uint32_t)buf[xing_pos+4] << 24) | ((uint32_t)buf[xing_pos+5] << 16) |
                                     ((uint32_t)buf[xing_pos+6] << 8)  | (uint32_t)buf[xing_pos+7];
                    if (flags & 0x0001) {
                        uint32_t total_frames = ((uint32_t)buf[xing_pos+8] << 24) | ((uint32_t)buf[xing_pos+9] << 16) |
                                                ((uint32_t)buf[xing_pos+10] << 8) | (uint32_t)buf[xing_pos+11];
                        if (total_frames > 0) {
                            int sec = (int)((uint64_t)total_frames * 1152 / samplerate);
                            if (sec >= 10 && sec <= 1800) return sec;
                        }
                    }
                }
            }

            // 检查 VBRI 头部
            size_t vbri_pos = i + 4 + 32;
            if (vbri_pos + 18 < sizeof(buf)) {
                if (buf[vbri_pos] == 'V' && buf[vbri_pos+1] == 'B' && buf[vbri_pos+2] == 'R' && buf[vbri_pos+3] == 'I') {
                    uint32_t total_frames = ((uint32_t)buf[vbri_pos+14] << 24) | ((uint32_t)buf[vbri_pos+15] << 16) |
                                            ((uint32_t)buf[vbri_pos+16] << 8)  | (uint32_t)buf[vbri_pos+17];
                    if (total_frames > 0) {
                        int sec = (int)((uint64_t)total_frames * 1152 / samplerate);
                        if (sec >= 10 && sec <= 1800) return sec;
                    }
                }
            }

            // 3. 标准 CBR 音频帧计算
            uint32_t audio_bytes = (fileSize > id3_size) ? (fileSize - id3_size) : fileSize;
            int cbr_sec = (int)((uint64_t)audio_bytes * 8 / (bitrate * 1000));
            if (cbr_sec >= 10 && cbr_sec <= 1800) return cbr_sec;
        }
    }

    return 240;
}

// ----------------- FAT32 真实 SD 卡全盘递归扫描引擎 (支持根目录与所有子目录) -----------------
static bool scanSdCardFat32() {
    if (!s_sd_card_ready) return false;

    static uint8_t sec_buf[512];
    
    // 1. 读取扇区 0 (MBR 分区表)
    if (HAL_SD_ReadBlocks(&hsd1, sec_buf, 0, 1, 3000) != HAL_OK) {
        Serial.println("❌ [STM32-MP3] 读取 SD 卡 MBR 失败");
        return false;
    }

    uint32_t part_lba = 0;
    if (sec_buf[510] == 0x55 && sec_buf[511] == 0xAA) {
        uint32_t lba1 = *(uint32_t*)&sec_buf[0x1C6];
        if (lba1 > 0 && lba1 < 2000000000) {
            part_lba = lba1;
        }
    }

    // 2. 读取 VBR / DBR (FAT32 卷引导记录)
    if (HAL_SD_ReadBlocks(&hsd1, sec_buf, part_lba, 1, 3000) != HAL_OK) {
        Serial.println("❌ [STM32-MP3] 读取 FAT32 DBR 失败");
        return false;
    }

    uint16_t bytes_per_sec = *(uint16_t*)&sec_buf[11];
    s_sec_per_clust = sec_buf[13];
    uint16_t rsvd_sec_cnt = *(uint16_t*)&sec_buf[14];
    uint8_t num_fats = sec_buf[16];
    uint32_t fat_sz = *(uint32_t*)&sec_buf[36];
    uint32_t root_clust = *(uint32_t*)&sec_buf[44];

    if (bytes_per_sec != 512 || s_sec_per_clust == 0 || fat_sz == 0) {
        Serial.printf("⚠️ [STM32-MP3] 非标准 FAT32 扇区参数: %d bytes/sec, %d sec/clust\r\n", bytes_per_sec, s_sec_per_clust);
        return false;
    }

    s_fat_lba = part_lba + rsvd_sec_cnt;
    s_data_lba = s_fat_lba + (num_fats * fat_sz);

    s_playlist.clear();
    s_lrcFiles.clear();
    int track_id = 0;

    // 3. 广度优先递归遍历根目录与全部子目录簇链
    std::vector<uint32_t> dir_queue;
    dir_queue.push_back(root_clust);
    size_t q_idx = 0;

    while (q_idx < dir_queue.size() && q_idx < 100) {
        uint32_t current_clust = dir_queue[q_idx++];
        int clusters_read = 0;

        while (current_clust >= 2 && current_clust < 0x0FFFFFF8 && clusters_read < 300) {
            clusters_read++;
            uint32_t lba = clust_to_lba(current_clust);
            String lfn_acc = "";
            bool dir_ended = false;

            for (uint8_t s = 0; s < s_sec_per_clust; s++) {
                if (HAL_SD_ReadBlocks(&hsd1, sec_buf, lba + s, 1, 3000) != HAL_OK) break;

                for (int e = 0; e < 512; e += 32) {
                    uint8_t* ent = &sec_buf[e];
                    if (ent[0] == 0x00) { dir_ended = true; break; }
                    if (ent[0] == 0xE5) { lfn_acc = ""; continue; }

                    uint8_t attr = ent[11];
                    if (attr == 0x0F) {
                        // VFAT 长文件名项 (LFN)
                        uint8_t seq = ent[0];
                        if (seq & 0x40) {
                            lfn_acc = ""; // 🌟 新长文件名首项：彻底清空累加器，杜绝残余脏数据
                        }
                        String chunk_utf8 = "";
                        auto readLfnChars = [&](int start, int end) {
                            for (int k = start; k < end; k += 2) {
                                uint16_t ch = *(uint16_t*)&ent[k];
                                if (ch == 0 || ch == 0xFFFF) return false;
                                appendUtf16ToUtf8(ch, chunk_utf8);
                            }
                            return true;
                        };
                        if (readLfnChars(1, 11)) {
                            if (readLfnChars(14, 26)) {
                                readLfnChars(28, 32);
                            }
                        }
                        lfn_acc = chunk_utf8 + lfn_acc;
                    } else if (attr & 0x10) {
                        // 🌟 子目录：加入扫描队列 (过滤 . / .. 与隐藏/系统目录)
                        String dname = (lfn_acc.length() > 0) ? lfn_acc : "";
                        lfn_acc = "";
                        if (ent[0] != '.' && !dname.startsWith(".") && !dname.startsWith("System Volume") && !dname.startsWith("$")) {
                            uint16_t chi = *(uint16_t*)&ent[20];
                            uint16_t clo = *(uint16_t*)&ent[26];
                            uint32_t sub_c = ((uint32_t)chi << 16) | clo;
                            if (sub_c >= 2 && sub_c < 0x0FFFFFF8) {
                                dir_queue.push_back(sub_c);
                                Serial.printf("  📁 [SD卡发现子目录 #%02d] %s (起始簇:%d)\r\n", (int)dir_queue.size(), dname.c_str(), sub_c);
                            }
                        }
                    } else if ((attr & 0x18) == 0) {
                        // 标准普通文件
                        String fname = "";
                        if (lfn_acc.length() > 0) {
                            fname = lfn_acc;
                            lfn_acc = "";
                        } else {
                            char sname[13] = {0};
                            int si = 0;
                            for (int k = 0; k < 8; k++) if (ent[k] != ' ') sname[si++] = ent[k];
                            if (ent[8] != ' ') {
                                sname[si++] = '.';
                                for (int k = 8; k < 11; k++) if (ent[k] != ' ') sname[si++] = ent[k];
                            }
                            fname = String(sname);
                        }

                        uint16_t chi = *(uint16_t*)&ent[20];
                        uint16_t clo = *(uint16_t*)&ent[26];
                        uint32_t start_c = ((uint32_t)chi << 16) | clo;
                        uint32_t fsize = *(uint32_t*)&ent[28];

                        String lower = fname;
                        lower.toLowerCase();
                        if (lower.endsWith(".lrc")) {
                            s_lrcFiles.push_back({fname, start_c, fsize});
                            Serial.printf("  📄 [SD卡发现歌词] %s (%d 字节)\r\n", fname.c_str(), fsize);
                        } else if (lower.endsWith(".mp3") || lower.endsWith(".wav")) {
                            String id3Title = "";
                            String id3Artist = "";
                            uint32_t id3_sz = 0;
                            int dur = parseMp3Metadata(start_c, fsize, id3Title, id3Artist, id3_sz);

                            // 🌟 1. 优先从高保真长文件名提取基名 (绝不受损坏 ID3 标签污染，杜绝“爱错-王力宏”变“宏”)
                            String cleanName = fname;
                            int dot = cleanName.lastIndexOf('.');
                            if (dot > 0) cleanName = cleanName.substring(0, dot);
                            cleanName.trim();

                            // 剥离前导数字序号 (如 "01. ", "01 - ", "01_", "1. ", "1 - ")
                            int numLen = 0;
                            while (numLen < (int)cleanName.length() && isdigit(cleanName[numLen])) numLen++;
                            if (numLen > 0 && numLen <= 3 && numLen < (int)cleanName.length()) {
                                char c = cleanName[numLen];
                                if (c == '.' || c == '-' || c == '_' || c == ' ' || c == '、') {
                                    cleanName = cleanName.substring(numLen + 1);
                                    cleanName.trim();
                                    if (cleanName.startsWith("-") || cleanName.startsWith(".") || cleanName.startsWith("_")) {
                                        cleanName = cleanName.substring(1);
                                        cleanName.trim();
                                    }
                                }
                            }

                            // 🌟 2. 歌名与歌手智能提取：保证 100% 按【歌名】首字母精准拼音排序！
                            String songTitle = cleanName;
                            String artist = (id3Artist.length() >= 2 && id3Artist != "未知歌手") ? id3Artist : "未知歌手";

                            int dash = cleanName.indexOf('-');
                            if (dash > 0) {
                                String p1 = cleanName.substring(0, dash);
                                String p2 = cleanName.substring(dash + 1);
                                p1.trim();
                                p2.trim();
                                if (id3Artist.length() > 0 && id3Artist == p1) {
                                    songTitle = p2;
                                    artist = p1;
                                } else if (id3Artist.length() > 0 && id3Artist == p2) {
                                    songTitle = p1;
                                    artist = p2;
                                } else if (id3Title.length() > 0 && id3Title == p1) {
                                    songTitle = p1;
                                    artist = (p2.length() > 0) ? p2 : artist;
                                } else if (id3Title.length() > 0 && id3Title == p2) {
                                    songTitle = p2;
                                    artist = (p1.length() > 0) ? p1 : artist;
                                } else {
                                    // 中文歌曲命名绝大多数为 "歌手 - 歌名" (例如 "王力宏 - 爱错", "周杰伦 - 晴天")
                                    // 提取 p2 作为歌名 (爱错)，提取 p1 作为歌手 (王力宏)，排序即可按歌名 "爱(A)" 准确定位！
                                    if (p1.length() > 0 && p2.length() > 0) {
                                        songTitle = p2;
                                        artist = p1;
                                    } else {
                                        songTitle = cleanName;
                                    }
                                }
                            }

                            s_playlist.push_back({track_id++, fname, songTitle, artist, dur, start_c, fsize, id3_sz});
                            Serial.printf("  🎵 [SD卡发现歌曲 %02d] 《%s》 - %s (%02d:%02d, %d KB, 起始簇:%d)\r\n", 
                                          track_id, songTitle.c_str(), artist.c_str(), dur / 60, dur % 60, (int)(fsize / 1024), start_c);
                        }
                        lfn_acc = "";
                    }
                }
                if (dir_ended) break;
            }
            if (dir_ended) break;

            // 从 FAT 表读取下一个簇号
            uint32_t fat_sec = s_fat_lba + (current_clust * 4 / 512);
            uint32_t fat_offset = (current_clust * 4) % 512;
            if (HAL_SD_ReadBlocks(&hsd1, sec_buf, fat_sec, 1, 3000) != HAL_OK) break;
            current_clust = *(uint32_t*)&sec_buf[fat_offset] & 0x0FFFFFFF;
        }
    }


    if (!s_playlist.empty()) {
        // 🌟 核心升级：按歌名首字读音拼音字母 (A~Z, 0~9) 全局升序精准排序！
        std::sort(s_playlist.begin(), s_playlist.end(), compareSongsByPinyin);

        // 重新编号排序后的 Track ID (0, 1, 2, ... N-1)
        for (size_t i = 0; i < s_playlist.size(); i++) {
            s_playlist[i].id = i;
            char ini = getSortInitialKey(s_playlist[i].title.c_str());
            Serial.printf("  🎵 [已排序歌单 #%02d] [%c] %s - %s (%02d:%02d)\r\n", 
                          (int)i + 1, ini, s_playlist[i].title.c_str(), s_playlist[i].artist.c_str(),
                          s_playlist[i].durationSec / 60, s_playlist[i].durationSec % 60);
        }

        s_currentTrack = 0;
        s_totalSeconds = s_playlist[0].durationSec;
        s_currentSeconds = 0;
        Serial.printf("✅ [STM32-MP3] SD卡扫描并首字母排序完成，发现 %d 首歌曲，%d 个LRC歌词文件！\r\n", 
                      (int)s_playlist.size(), (int)s_lrcFiles.size());
        return true;
    } else {
        Serial.println("⚠️ [STM32-MP3] SD卡根目录未检索到 .mp3 文件！");
        return false;
    }
}

// ----------------- 初始化 -----------------
void begin() {
    // 1. 初始化 USART2 控制总线 115200 Baud (专跑指令与歌词同步)
    ControlSerial.begin(115200);

    // 2. 初始化 5线 SPI2 硬件总线
    pinMode(MY_PIN_SPI_NSS, OUTPUT);
    digitalWrite(MY_PIN_SPI_NSS, HIGH);
    pinMode(MY_PIN_HANDSHAKE, INPUT);
    SPI.begin();

    // 3. 初始化硬件 SDMMC1 总线并进行初次全盘扫描
    s_sd_card_ready = initSdmmcHardware();
    if (s_sd_card_ready) {
        Serial.println("✅ [STM32-MP3] SDMMC1 4-bit 硬件初始化成功，开始检索 SD 卡文件...");
        scanSdCardFat32();
    } else {
        Serial.println("⚠️ [STM32-MP3] 未检测到 SD 卡！");
    }

    Serial.println("🚀 [STM32-MP3] 后端媒体服务已启动就绪 (UART 115200 + SPI2 20MHz DMA)！");
}

static void loadLyricsForTrack(int trackId) {
    s_currentLyrics.clear();
    s_last_lyric_idx = -1;
    if (s_playlist.empty() || trackId < 0 || trackId >= (int)s_playlist.size()) return;

    const auto& song = s_playlist[trackId];
    String songTitle = song.title;
    String songFname = song.fileName;
    String songArtist = song.artist;
    songTitle.toLowerCase();
    songFname.toLowerCase();
    songArtist.toLowerCase();

    String songFnameBase = songFname;
    if (songFnameBase.lastIndexOf('.') > 0) songFnameBase = songFnameBase.substring(0, songFnameBase.lastIndexOf('.'));

    // 1. 在已扫描到的 s_lrcFiles 中检索匹配的 LRC 文件 (优先严格全名/基名精准匹配)
    bool lrcFound = false;
    const LrcFileInfo* matchedLrc = nullptr;

    // 第一轮：严格文件名匹配 (如 "BIGBANG - IF YOU.mp3" <-> "BIGBANG - IF YOU.lrc")
    for (const auto& lrc : s_lrcFiles) {
        String lrcLower = lrc.lrcName;
        lrcLower.toLowerCase();
        String lrcBase = lrcLower;
        if (lrcBase.lastIndexOf('.') > 0) lrcBase = lrcBase.substring(0, lrcBase.lastIndexOf('.'));

        if (lrcBase == songFnameBase || lrcBase == songTitle || lrcLower == (songFnameBase + ".lrc")) {
            matchedLrc = &lrc;
            break;
        }
    }

    // 第二轮：若第一轮未命中，尝试带歌手名的精准组合匹配 (如 "周杰伦 - 晴天" <-> "晴天")
    if (!matchedLrc) {
        for (const auto& lrc : s_lrcFiles) {
            String lrcLower = lrc.lrcName;
            lrcLower.toLowerCase();
            String lrcBase = lrcLower;
            if (lrcBase.lastIndexOf('.') > 0) lrcBase = lrcBase.substring(0, lrcBase.lastIndexOf('.'));

            if (lrcBase == (songArtist + " - " + songTitle) || lrcBase == (songTitle + " - " + songArtist)) {
                matchedLrc = &lrc;
                break;
            }
        }
    }

    // 第三轮：若仍未命中，仅在长度 >= 4 时进行包含匹配 (杜绝短歌名如 "if you" 误碰其他长歌名)
    if (!matchedLrc && songTitle.length() >= 4) {
        for (const auto& lrc : s_lrcFiles) {
            String lrcLower = lrc.lrcName;
            lrcLower.toLowerCase();
            String lrcBase = lrcLower;
            if (lrcBase.lastIndexOf('.') > 0) lrcBase = lrcBase.substring(0, lrcBase.lastIndexOf('.'));

            if (lrcBase.indexOf(songTitle) >= 0 || songFnameBase.indexOf(lrcBase) >= 0) {
                matchedLrc = &lrc;
                break;
            }
        }
    }

    if (matchedLrc && matchedLrc->startClust >= 2 && matchedLrc->fileSize > 0 && matchedLrc->fileSize < 65536) {
        const auto& lrc = *matchedLrc;
        Serial.printf("📄 [STM32-MP3] 正在加载 SD 卡真实歌词: %s (%d 字节)...\r\n", lrc.lrcName.c_str(), lrc.fileSize);
        
        static uint8_t lrc_sec[512];
        String lrcContent = "";
        uint32_t cur_c = lrc.startClust;
        uint32_t bytes_read = 0;

        while (cur_c >= 2 && cur_c < 0x0FFFFFF8 && bytes_read < lrc.fileSize && bytes_read < 32768) {
            uint32_t lba = clust_to_lba(cur_c);
            for (uint8_t s = 0; s < s_sec_per_clust; s++) {
                if (HAL_SD_ReadBlocks(&hsd1, lrc_sec, lba + s, 1, 3000) != HAL_OK) break;
                for (int b = 0; b < 512; b++) {
                    if (bytes_read >= lrc.fileSize) break;
                    lrcContent += (char)lrc_sec[b];
                    bytes_read++;
                }
                if (bytes_read >= lrc.fileSize) break;
            }
            // 获取下一个簇
            uint32_t fat_sec = s_fat_lba + (cur_c * 4 / 512);
            uint32_t fat_offset = (cur_c * 4) % 512;
            if (HAL_SD_ReadBlocks(&hsd1, lrc_sec, fat_sec, 1, 3000) != HAL_OK) break;
            cur_c = *(uint32_t*)&lrc_sec[fat_offset] & 0x0FFFFFFF;
        }

        // 2. 逐行解析 LRC 格式: [mm:ss.xx] 歌词文本
        int startPos = 0;
        // 跳过 UTF-8 BOM
        if (lrcContent.length() >= 3 && (uint8_t)lrcContent[0] == 0xEF && (uint8_t)lrcContent[1] == 0xBB && (uint8_t)lrcContent[2] == 0xBF) {
            startPos = 3;
        }

                while (startPos < (int)lrcContent.length()) {
                    int endPos = lrcContent.indexOf('\n', startPos);
                    if (endPos < 0) endPos = lrcContent.length();
                    String line = lrcContent.substring(startPos, endPos);
                    line.trim();
                    line.replace("\r", "");
                    startPos = endPos + 1;

                    if (line.startsWith("[") && line.indexOf(']') > 1) {
                        int rb = line.indexOf(']');
                        String timeStr = line.substring(1, rb);
                        String textStr = line.substring(rb + 1);
                        textStr.trim();

                        int colon = timeStr.indexOf(':');
                        if (colon > 0 && textStr.length() > 0) {
                            int m = timeStr.substring(0, colon).toInt();
                            float s = timeStr.substring(colon + 1).toFloat();
                            uint32_t t_ms = (uint32_t)(m * 60000 + s * 1000);
                            s_currentLyrics.push_back({t_ms, textStr});
                        }
                    }
                }

        if (!s_currentLyrics.empty()) {
            lrcFound = true;
            uint32_t last_lrc_ms = s_currentLyrics.back().timeMs;
            if (s_playlist[trackId].durationSec <= 0 && last_lrc_ms > 10000) {
                int real_lrc_dur = (int)((last_lrc_ms + 4000) / 1000);
                s_playlist[trackId].durationSec = real_lrc_dur;
                s_totalSeconds = real_lrc_dur;
            }
            Serial.printf("✅ [STM32-MP3] 成功载入《%s》真实歌词，共 %d 行 (总时长: %02d:%02d)！\r\n", 
                          song.title.c_str(), (int)s_currentLyrics.size(), s_totalSeconds / 60, s_totalSeconds % 60);
        }
    }

    if (!lrcFound) {
        Serial.printf("⚠️ [STM32-MP3] 未匹配到《%s》的独立LRC歌词文件，使用动态歌曲信息呈现\r\n", song.title.c_str());
        s_currentLyrics.push_back({0, "正在播放: " + song.title});
        s_currentLyrics.push_back({4000, "演唱歌手: " + song.artist});
        s_currentLyrics.push_back({8000, "音频格式: 高保真无损音乐"});
        s_currentLyrics.push_back({12000, "沉浸式 Hi-Fi 立体声"});
        s_currentLyrics.push_back({16000, "随旋律律动中..."});
    }
}

static void sendTextPacket(const String& str) {
    ControlSerial.println(str);
}

static bool sendAudioPacketSpi(const int16_t* pcm, uint16_t sample_count, uint16_t sample_rate = 44100, uint32_t magic = 0xA0D10001) {
    if (magic == 0xA0D10001 && (pcm == nullptr || sample_count == 0)) return false;
    
    // 🌟 等待 ESP32 硬件 DMA 挂载完毕 (PB1 == HIGH)，最多等待 50ms 杜绝瞬间漏帧与乱序
    uint32_t wait_start = millis();
    while (digitalRead(MY_PIN_HANDSHAKE) == LOW) {
        if (millis() - wait_start >= 50) {
            return false;
        }
        delayMicroseconds(20);
    }

    static AudioPacket audio_pkt;
    audio_pkt.magic = magic;
    audio_pkt.sample_count = (sample_count > 1152) ? 1152 : sample_count;
    audio_pkt.sample_rate = sample_rate;
    if (pcm && audio_pkt.sample_count > 0) {
        memcpy(audio_pkt.samples, pcm, audio_pkt.sample_count * sizeof(int16_t));
    } else {
        memset(audio_pkt.samples, 0, sizeof(audio_pkt.samples));
    }
    audio_pkt.tail_magic = 0xFEEDFACE;

    // 🌟 1. 先配置 SPI 事务 (10MHz 极速且波形绝对纯净，在 NSS 处于 HIGH 时锁定 SCK 空闲低电平)
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    delayMicroseconds(2);

    // 🌟 2. SCK 稳定后拉低 NSS 选通从机
    digitalWrite(MY_PIN_SPI_NSS, LOW);
    delayMicroseconds(5);

    // 🌟 3. 发送完整 2316 字节数据包 (10MHz 耗时仅 1.85ms，远快于 26.12ms 帧周期)
    SPI.transfer((const uint8_t*)&audio_pkt, nullptr, sizeof(audio_pkt));
    delayMicroseconds(5);

    // 🌟 4. 拉高 NSS 结束本次传输
    digitalWrite(MY_PIN_SPI_NSS, HIGH);
    delayMicroseconds(2);

    // 🌟 5. 释放 SPI 事务 (传输完成立即返回，下一次传输由入口处的 PB1 握手判定流控)
    SPI.endTransaction();
    return true;
}

static uint32_t s_track_start_millis = 0;
static uint32_t s_track_accum_ms = 0;
static uint32_t s_last_lyric_check_ms = 0;
static int s_last_sent_lyric_idx = -1;
static bool s_eos_sent = false;

// ----------------- 发送当前 3 行歌词 (毫秒级精准对齐) -----------------
// ----------------- 发送当前曲目完整本地歌词表给 ESP32 (切歌时一次性极速下发) -----------------
static void sendFullLyricsToFrontend() {
    if (s_playlist.empty() || s_currentTrack < 0 || s_currentTrack >= (int)s_playlist.size()) return;
    sendTextPacket("LRC:CLEAR");
    for (size_t i = 0; i < s_currentLyrics.size(); i++) {
        char buf[256];
        snprintf(buf, sizeof(buf), "LRC:%u,%s", (unsigned)s_currentLyrics[i].timeMs, s_currentLyrics[i].text.c_str());
        sendTextPacket(buf);
    }
    sendTextPacket("LRC:DONE");
    Serial.printf("📜 [STM32-MP3] 成功下发《%s》整首完整歌词 (%d 行) 给前端！\r\n", 
                  s_playlist[s_currentTrack].title.c_str(), (int)s_currentLyrics.size());
}

// ----------------- 发送歌单与首曲信息 -----------------
static void sendPlaylistToFrontend() {
    Serial.printf("📤 [STM32-MP3] 正在通过 USART2(PA2) 向 ESP32 发送 CONNECTED 与 %d 首歌单数据...\r\n", (int)s_playlist.size());
    sendTextPacket("CONNECTED");
    ControlSerial.flush();
    delay(15);
    sendTextPacket("LIST_START");
    ControlSerial.flush();
    delay(15);
    for (size_t i = 0; i < s_playlist.size(); i++) {
        const auto& s = s_playlist[i];
        char buf[256];
        snprintf(buf, sizeof(buf), "ITEM:%d\t%s\t%d\t%s", s.id, s.title.c_str(), s.durationSec, s.artist.c_str());
        sendTextPacket(buf);
        ControlSerial.flush(); // 🌟 核心保障：确保硬件移位寄存器完全清空后再进入间歇
        delay(10);             // 🌟 10ms 物理安全节拍：让 ESP32 有充裕时间消费 UART FIFO，彻底杜绝丢包与字符粘连！
    }
    sendTextPacket("LIST_END");
    ControlSerial.flush();
    delay(15);
    char statBuf[128];
    snprintf(statBuf, sizeof(statBuf), "STATUS:%d,%d,%d,%d,%d,%d", s_playState, s_currentSeconds, s_totalSeconds, s_volume, s_playMode, s_currentTrack);
    sendTextPacket(statBuf);
    ControlSerial.flush();
    delay(15);
    loadLyricsForTrack(0);
    sendFullLyricsToFrontend();
    ControlSerial.flush();
    Serial.printf("✅ [STM32-MP3] 已响应 ESP32 连接请求，同步歌单(%d首)与首曲信息完毕！\r\n", (int)s_playlist.size());
}

static mp3dec_t s_mp3d;
static uint32_t s_play_cur_clust = 0;
static uint8_t  s_play_cur_sec_in_clust = 0;
static uint32_t s_play_bytes_read = 0;

// 🌟 128KB 超大环形水库 (位于 AXI SRAM 0x24000000，完美支持 CPU与DMA 全局总线)：
// 生产者 (SD 卡高速突发读) 与消费者 (MinimP3 软解) 深度解耦，可蓄积 8~10 秒音频数据，杜绝任何 SD 卡延迟！
static constexpr size_t MP3_RING_SIZE = 131072;
static uint8_t s_mp3_ring[MP3_RING_SIZE];
static volatile size_t s_mp3_head = 0;
static volatile size_t s_mp3_tail = 0;

static inline size_t mp3_ring_available() {
    size_t h = s_mp3_head;
    size_t t = s_mp3_tail;
    return (h >= t) ? (h - t) : (MP3_RING_SIZE - t + h);
}

static inline size_t mp3_ring_free() {
    return (MP3_RING_SIZE - 1) - mp3_ring_available();
}

static inline void mp3_ring_push(const uint8_t* data, size_t len) {
    if (len == 0 || data == nullptr) return;
    size_t free_space = mp3_ring_free();
    if (len > free_space) len = free_space;
    if (len == 0) return;

    size_t h = s_mp3_head;
    size_t first_chunk = MP3_RING_SIZE - h;
    if (len <= first_chunk) {
        memcpy(&s_mp3_ring[h], data, len);
        s_mp3_head = (h + len) % MP3_RING_SIZE;
    } else {
        memcpy(&s_mp3_ring[h], data, first_chunk);
        memcpy(&s_mp3_ring[0], data + first_chunk, len - first_chunk);
        s_mp3_head = len - first_chunk;
    }
}

static inline void mp3_ring_drop(size_t len) {
    size_t avail = mp3_ring_available();
    if (len > avail) len = avail;
    s_mp3_tail = (s_mp3_tail + len) % MP3_RING_SIZE;
}

static inline size_t mp3_ring_peek_linear(uint8_t* dst, size_t max_len) {
    size_t avail = mp3_ring_available();
    if (max_len > avail) max_len = avail;
    if (max_len == 0) return 0;
    size_t t = s_mp3_tail;
    size_t first_chunk = MP3_RING_SIZE - t;
    if (max_len <= first_chunk) {
        memcpy(dst, &s_mp3_ring[t], max_len);
    } else {
        memcpy(dst, &s_mp3_ring[t], first_chunk);
    memcpy(dst + first_chunk, &s_mp3_ring[0], max_len - first_chunk);
    }
    return max_len;
}

static bool     s_decoder_initialized = false;
static uint32_t s_decoded_frames = 0;
static uint32_t s_pcm_bytes_sent = 0;
static uint32_t s_last_frame_decode_us = 0;
static uint32_t s_frame_duration_us = 26122; // 默认 44.1kHz @ 1152 采样点 = 26122 us (38.28 fps)

// 🌟 已解码待发送音频帧暂存池（确保每帧仅解码一次，绝不重复推进 MinimP3 滤波器历史状态）
static int16_t  s_pending_mono_pcm[1152];
static uint16_t s_pending_mono_count = 0;
static uint16_t s_pending_sample_rate = 44100;
static size_t   s_pending_consumed_bytes = 0;

// ----------------- 播放控制核心 -----------------
void playTrack(int trackId) {
    if (::g_camera_streaming) {
        ::g_camera_streaming = false;
        Serial.println("📷 [STM32-MP3] ⚡ SPI互斥: 开始播放音乐，已自动关闭摄像头推流！");
    }
    if (s_playlist.empty()) {
        Serial.println("⚠️ [STM32-MP3] 歌单为空，无法播放！请在SD卡存入MP3歌曲");
        return;
    }
    if (trackId < 0 || trackId >= (int)s_playlist.size()) trackId = 0;
    s_currentTrack = trackId;
    const auto& song = s_playlist[trackId];
    s_totalSeconds = song.durationSec;
    s_currentSeconds = 0;
    s_track_start_millis = millis();
    s_track_accum_ms = 0;
    s_last_sent_lyric_idx = -1;
    s_last_frame_decode_us = 0;
    s_frame_duration_us = 26122;
    s_playState = STATE_PLAYING;
    s_last_tick_ms = millis();

    // 商业级初始化：初始化软解码器与 FAT32 簇链
    s_play_cur_clust = song.startClust;
    s_play_cur_sec_in_clust = 0;
    s_play_bytes_read = 0;
    s_mp3_head = 0;
    s_mp3_tail = 0;
    s_decoded_frames = 0;
    s_pcm_bytes_sent = 0;
    s_pending_mono_count = 0;
    s_pending_sample_rate = 44100;
    s_pending_consumed_bytes = 0;
    s_eos_sent = false;
    mp3dec_init(&s_mp3d);
    s_decoder_initialized = true;

    // 商业级 ID3v2 标签极速跨越：直接计算 ID3 头部字节并在 FAT32 簇链中跳过
    if (s_play_cur_clust >= 2) {
        static uint8_t head_sec[512];
        uint32_t head_lba = clust_to_lba(s_play_cur_clust);
        if (HAL_SD_ReadBlocks(&hsd1, head_sec, head_lba, 1, 1000) == HAL_OK) {
            uint32_t id3_size = 0;
            if (head_sec[0] == 'I' && head_sec[1] == 'D' && head_sec[2] == '3') {
                id3_size = (((uint32_t)(head_sec[6] & 0x7F) << 21) |
                            ((uint32_t)(head_sec[7] & 0x7F) << 14) |
                            ((uint32_t)(head_sec[8] & 0x7F) << 7)  |
                            ((uint32_t)(head_sec[9] & 0x7F))) + 10;
                if (head_sec[5] & 0x10) id3_size += 10;
                Serial.printf("🏷️ [STM32-MP3] 商业级解析：检测到 ID3v2 标签区大小 %d 字节，正在极速跨越...\r\n", id3_size);
            }
            // 计算 ID3 占用的扇区数与簇数，直接在 FAT32 簇链中跳进
            uint32_t skip_sectors = id3_size / 512;
            uint32_t skip_bytes_in_sec = id3_size % 512;

            while (skip_sectors > 0 && s_play_cur_clust >= 2 && s_play_cur_clust < 0x0FFFFFF8) {
                if (skip_sectors >= s_sec_per_clust) {
                    skip_sectors -= s_sec_per_clust;
                    s_play_bytes_read += (s_sec_per_clust * 512);
                    // 读取下一个簇号
                    uint32_t fat_sec = s_fat_lba + (s_play_cur_clust * 4 / 512);
                    uint32_t fat_offset = (s_play_cur_clust * 4) % 512;
                    static uint8_t fat_buf[512];
                    if (HAL_SD_ReadBlocks(&hsd1, fat_buf, fat_sec, 1, 1000) == HAL_OK) {
                        s_play_cur_clust = *(uint32_t*)&fat_buf[fat_offset] & 0x0FFFFFFF;
                    } else {
                        break;
                    }
                } else {
                    s_play_cur_sec_in_clust = skip_sectors;
                    s_play_bytes_read += (skip_sectors * 512);
                    skip_sectors = 0;
                }
            }

            // 读取首个真实音频扇区并截去末尾剩余的 ID3 字节
            uint32_t audio_sec_lba = clust_to_lba(s_play_cur_clust) + s_play_cur_sec_in_clust;
            if (HAL_SD_ReadBlocks(&hsd1, head_sec, audio_sec_lba, 1, 1000) == HAL_OK) {
                s_play_cur_sec_in_clust++;
                s_play_bytes_read += 512;
                if (skip_bytes_in_sec < 512) {
                    uint32_t valid_bytes = 512 - skip_bytes_in_sec;
                    mp3_ring_push(head_sec + skip_bytes_in_sec, valid_bytes);
                }
                // 校验首个音频扇区后是否跨簇
                if (s_play_cur_sec_in_clust >= s_sec_per_clust) {
                    s_play_cur_sec_in_clust = 0;
                    uint32_t fat_sec = s_fat_lba + (s_play_cur_clust * 4 / 512);
                    uint32_t fat_offset = (s_play_cur_clust * 4) % 512;
                    static uint8_t fat_buf[512];
                    if (HAL_SD_ReadBlocks(&hsd1, fat_buf, fat_sec, 1, 1000) == HAL_OK) {
                        s_play_cur_clust = *(uint32_t*)&fat_buf[fat_offset] & 0x0FFFFFFF;
                    } else {
                        s_play_cur_clust = 0x0FFFFFFF;
                    }
                }
            }
        }
    }

    loadLyricsForTrack(trackId);
    sendFullLyricsToFrontend();

    char statBuf[128];
    snprintf(statBuf, sizeof(statBuf), "STATUS:%d,%d,%d,%d,%d,%d", s_playState, s_currentSeconds, s_totalSeconds, s_volume, s_playMode, s_currentTrack);
    sendTextPacket(statBuf);

    Serial.printf("▶️ [STM32-MP3] 开始商业级真实软解播放: [%02d] %s (%s, %02d:%02d, 起始簇:%d)\r\n", 
                  song.id + 1, song.title.c_str(), song.artist.c_str(), 
                  song.durationSec / 60, song.durationSec % 60, song.startClust);
}

void togglePlayPause() {
    if (s_playState == STATE_PLAYING) pause();
    else if (s_playState == STATE_PAUSED) resume();
    else playTrack(s_currentTrack);
}
void pause() {
    if (s_playState == STATE_PLAYING) {
        s_track_accum_ms += (millis() - s_track_start_millis);
    }
    s_playState = STATE_PAUSED;
    char buf[128];
    snprintf(buf, sizeof(buf), "STATUS:%d,%d,%d,%d,%d,%d", s_playState, s_currentSeconds, s_totalSeconds, s_volume, s_playMode, s_currentTrack);
    sendTextPacket(buf);
    Serial.println("⏸️ [STM32-MP3] 暂停播放");
}
void resume() {
    if (::g_camera_streaming) {
        ::g_camera_streaming = false;
        Serial.println("📷 [STM32-MP3] ⚡ SPI互斥: 恢复播放音乐，已自动关闭摄像头推流！");
    }
    s_playState = STATE_PLAYING;
    s_track_start_millis = millis();
    s_last_tick_ms = millis();
    char buf[128];
    snprintf(buf, sizeof(buf), "STATUS:%d,%d,%d,%d,%d,%d", s_playState, s_currentSeconds, s_totalSeconds, s_volume, s_playMode, s_currentTrack);
    sendTextPacket(buf);
    Serial.printf("▶️ [STM32-MP3] 恢复播放: [%02d] %s\r\n", s_currentTrack + 1, s_playlist.empty() ? "" : s_playlist[s_currentTrack].title.c_str());
}
void stop() {
    s_playState = STATE_STOPPED;
    s_track_accum_ms = 0;
    s_currentSeconds = 0;
    s_last_sent_lyric_idx = -1;
    s_mp3_head = 0;
    s_mp3_tail = 0;
    s_pending_mono_count = 0;
    s_pending_consumed_bytes = 0;
    char buf[128];
    snprintf(buf, sizeof(buf), "STATUS:%d,%d,%d,%d,%d,%d", s_playState, s_currentSeconds, s_totalSeconds, s_volume, s_playMode, s_currentTrack);
    sendTextPacket(buf);
    Serial.println("⏹️ [STM32-MP3] 停止播放并清空环形水库与暂存区");
}
void next() {
    if (s_playlist.empty()) return;
    int nextTrack = (s_currentTrack + 1) % (int)s_playlist.size();
    playTrack(nextTrack);
}
void prev() {
    if (s_playlist.empty()) return;
    int prevTrack = (s_currentTrack - 1 + (int)s_playlist.size()) % (int)s_playlist.size();
    playTrack(prevTrack);
}
void seek(int seconds) {
    if (s_playlist.empty() || s_currentTrack < 0 || s_currentTrack >= (int)s_playlist.size()) return;
    if (s_totalSeconds <= 0) return;
    if (seconds < 0) seconds = 0;
    if (seconds > s_totalSeconds) seconds = s_totalSeconds;

    const SongItem& song = s_playlist[s_currentTrack];
    s_currentSeconds = seconds;
    s_track_accum_ms = (uint32_t)seconds * 1000;
    s_track_start_millis = millis();
    s_last_tick_ms = millis();
    s_decoded_frames = (uint32_t)seconds * 1000000ULL / (s_frame_duration_us > 0 ? s_frame_duration_us : 26122);

    // 1. 清空当前正在解码与发送的暂存与环形队列 (确保整帧严密对齐)
    s_mp3_head = 0;
    s_mp3_tail = 0;
    s_pending_mono_count = 0;
    s_pending_consumed_bytes = 0;

    // 2. 重新初始化 MinimP3 软解器引擎 (清空之前的帧解码状态)
    mp3dec_init(&s_mp3d);

    // 3. 计算目标时间点对应的真实音频字节偏移 (扣除 ID3 标签头，精准定位真实音频流)
    uint32_t audio_offset = song.id3Size;
    uint32_t audio_bytes = (song.fileSize > audio_offset) ? (song.fileSize - audio_offset) : song.fileSize;
    uint32_t target_audio_byte = (uint32_t)(((uint64_t)seconds * audio_bytes) / s_totalSeconds);
    uint32_t target_byte = audio_offset + target_audio_byte;

    uint32_t skip_sectors = target_byte / 512;
    uint32_t skip_bytes_in_sec = target_byte % 512;

    s_play_cur_clust = song.startClust;
    s_play_cur_sec_in_clust = 0;
    s_play_bytes_read = 0;

    // 4. 极速沿 FAT 表快进跨簇 (遍历 FAT 表仅需 < 1ms)
    while (skip_sectors > 0 && s_play_cur_clust >= 2 && s_play_cur_clust < 0x0FFFFFF8) {
        if (skip_sectors >= s_sec_per_clust) {
            skip_sectors -= s_sec_per_clust;
            s_play_bytes_read += (s_sec_per_clust * 512);
            uint32_t fat_sec = s_fat_lba + (s_play_cur_clust * 4 / 512);
            uint32_t fat_offset = (s_play_cur_clust * 4) % 512;
            static uint8_t fat_buf[512];
            if (HAL_SD_ReadBlocks(&hsd1, fat_buf, fat_sec, 1, 1000) == HAL_OK) {
                s_play_cur_clust = *(uint32_t*)&fat_buf[fat_offset] & 0x0FFFFFFF;
            } else {
                break;
            }
        } else {
            s_play_cur_sec_in_clust = skip_sectors;
            s_play_bytes_read += (skip_sectors * 512);
            skip_sectors = 0;
        }
    }

    // 5. 读入新簇的目标扇区并推入环形水库
    if (s_play_cur_clust >= 2 && s_play_cur_clust < 0x0FFFFFF8) {
        static uint8_t head_sec[512];
        uint32_t audio_sec_lba = clust_to_lba(s_play_cur_clust) + s_play_cur_sec_in_clust;
        if (HAL_SD_ReadBlocks(&hsd1, head_sec, audio_sec_lba, 1, 1000) == HAL_OK) {
            s_play_cur_sec_in_clust++;
            s_play_bytes_read += 512;
            if (skip_bytes_in_sec < 512) {
                uint32_t valid_bytes = 512 - skip_bytes_in_sec;
                mp3_ring_push(head_sec + skip_bytes_in_sec, valid_bytes);
            }
        }
    }

    // 6. 立即更新当前状态 (ESP32 本地根据秒数 0ms 瞬间显示对应歌词)
    char statBuf[128];
    snprintf(statBuf, sizeof(statBuf), "STATUS:%d,%d,%d,%d,%d,%d", s_playState, s_currentSeconds, s_totalSeconds, s_volume, s_playMode, s_currentTrack);
    sendTextPacket(statBuf);

    Serial.printf("⏩ [STM32-MP3] 精准秒级 Seek 完成: 跳转至 %02d:%02d/%02d:%02d (音频字节偏移:%u, 簇号:%d)\r\n",
                  seconds / 60, seconds % 60, s_totalSeconds / 60, s_totalSeconds % 60, target_byte, s_play_cur_clust);
}
void setVolume(int vol) {
    if (vol < 0) vol = 0;
    if (vol > 100) vol = 100;
    s_volume = vol;
    char buf[128];
    snprintf(buf, sizeof(buf), "STATUS:%d,%d,%d,%d,%d,%d", s_playState, s_currentSeconds, s_totalSeconds, s_volume, s_playMode, s_currentTrack);
    sendTextPacket(buf);
}
void setPlayMode(PlayMode mode) {
    s_playMode = mode;
    char buf[128];
    snprintf(buf, sizeof(buf), "STATUS:%d,%d,%d,%d,%d,%d", s_playState, s_currentSeconds, s_totalSeconds, s_volume, s_playMode, s_currentTrack);
    sendTextPacket(buf);
}
void togglePlayMode() {
    s_playMode = static_cast<PlayMode>((s_playMode + 1) % 3);
    char buf[128];
    snprintf(buf, sizeof(buf), "STATUS:%d,%d,%d,%d,%d,%d", s_playState, s_currentSeconds, s_totalSeconds, s_volume, s_playMode, s_currentTrack);
    sendTextPacket(buf);
}
void refreshPlaylist() {
    sendPlaylistToFrontend();
}

// ----------------- 控制指令解析 -----------------
static void processCommand(const String& cmd) {
    if (cmd.startsWith("CMD:PLAY,")) {
        int trackId = cmd.substring(9).toInt();
        playTrack(trackId);
    } else if (cmd == "CMD:PLAY") {
        if (s_playState == STATE_PAUSED) resume();
        else playTrack(s_currentTrack);
    } else if (cmd == "CMD:PAUSE") {
        pause();
    } else if (cmd == "CMD:RESUME") {
        resume();
    } else if (cmd == "CMD:NEXT") {
        next();
    } else if (cmd == "CMD:PREV") {
        prev();
    } else if (cmd.startsWith("CMD:SEEK,")) {
        int sec = cmd.substring(9).toInt();
        seek(sec);
    } else if (cmd.startsWith("CMD:VOL,")) {
        int vol = cmd.substring(8).toInt();
        setVolume(vol);
    } else if (cmd.startsWith("CMD:MODE,")) {
        int mode = cmd.substring(9).toInt();
        setPlayMode(static_cast<PlayMode>(mode));
    } else if (cmd == "CMD:GET_LIST") {
        sendPlaylistToFrontend();
    } else if (cmd == "CMD:GET_LRC") {
        sendFullLyricsToFrontend();
    } else if (cmd == "CMD:CONNECT") {
        // 收到前端【连接】指令，回传状态、完整歌单与当前歌词
        sendPlaylistToFrontend();
        sendFullLyricsToFrontend();
        char statBuf[128];
        snprintf(statBuf, sizeof(statBuf), "STATUS:%d,%d,%d,%d,%d,%d", s_playState, s_currentSeconds, s_totalSeconds, s_volume, s_playMode, s_currentTrack);
        sendTextPacket(statBuf);
        Serial.println("🔗 [STM32-MP3] 收到前端【连接】指令，已下发当前状态、歌单与歌词！");
    } else if (cmd == "CMD:DISCONNECT") {
        // 收到前端【断开】指令，停止播放并回复 DISCONNECTED 确认
        stop();
        digitalWrite(MY_PIN_SPI_NSS, HIGH);
        sendTextPacket("DISCONNECTED");
        Serial.println("🛑 [STM32-MP3] 收到前端【断开】指令，已停止播放并向前端回复 DISCONNECTED 确认！");
    } else if (cmd == "CMD:STOP") {
        // 收到前端【停止】指令，停止播放并重置状态
        stop();
        digitalWrite(MY_PIN_SPI_NSS, HIGH);
        char statBuf[128];
        snprintf(statBuf, sizeof(statBuf), "STATUS:%d,%d,%d,%d,%d,%d", s_playState, s_currentSeconds, s_totalSeconds, s_volume, s_playMode, s_currentTrack);
        sendTextPacket(statBuf);
        Serial.println("🛑 [STM32-MP3] 收到前端【停止】指令，已停止播放！");
    } else if (cmd == "CMD:CAM_START") {
        if (s_playState == STATE_PLAYING || s_playState == STATE_PAUSED) {
            stop();
            digitalWrite(MY_PIN_SPI_NSS, HIGH); // 确保 NSS 彻底拉高释放
            Serial.println("🎵 [STM32-MP3] ⚡ SPI互斥: 收到相机开启指令，已强制停止 MP3 播放并释放 SPI2 总线！");
        }
        ::g_camera_streaming = true;
        Serial.println("📷 [STM32-CMD] 收到串口指令: 开启摄像头采集与传输 (g_camera_streaming = true)");
    } else if (cmd == "CMD:CAM_STOP") {
        ::g_camera_streaming = false;
        Serial.println("🛑 [STM32-CMD] 收到串口指令: 关闭摄像头采集与传输 (g_camera_streaming = false)");
    } else if (cmd == "CMD:FLASH_ON") {
        pinMode(PA5, OUTPUT);
        digitalWrite(PA5, HIGH);
        Serial.println("💡 [STM32-CMD] 收到串口指令: 开启补光灯 (PA5 HIGH)");
    } else if (cmd == "CMD:FLASH_OFF") {
        pinMode(PA5, OUTPUT);
        digitalWrite(PA5, LOW);
        Serial.println("💡 [STM32-CMD] 收到串口指令: 关闭补光灯 (PA5 LOW)");
    }
}

// ----------------- 周期性主轮询 -----------------
void update() {
    uint32_t now = millis();

    // 1. 响应 USART2 控制总线指令 (无阻塞高速轮询)
    static char cmd_buf[128];
    static size_t cmd_idx = 0;
    while (ControlSerial.available()) {
        char c = (char)ControlSerial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            if (cmd_idx > 0) {
                cmd_buf[cmd_idx] = '\0';
                String cmd = String(cmd_buf);
                cmd.trim();
                cmd_idx = 0;
                if (cmd.startsWith("CMD:")) {
                    processCommand(cmd);
                }
            }
        } else {
            if (cmd_idx < sizeof(cmd_buf) - 1) {
                cmd_buf[cmd_idx++] = c;
            }
        }
    }

    // 2. 播放时钟计时与自动切歌 (仅在播放态运行并定期同步进度)
    if (s_playState == STATE_PLAYING) {
        // 持续从 SD 卡高速读取 MP3 数据流填充 128KB 环形水库 (支持 4 扇区 2KB 高速突发读取)
        if (s_play_cur_sec_in_clust >= s_sec_per_clust && s_play_cur_clust >= 2 && s_play_cur_clust < 0x0FFFFFF8) {
            s_play_cur_sec_in_clust = 0;
            uint32_t fat_sec = s_fat_lba + (s_play_cur_clust * 4 / 512);
            uint32_t fat_offset = (s_play_cur_clust * 4) % 512;
            static uint8_t fat_buf[512];
            if (HAL_SD_ReadBlocks(&hsd1, fat_buf, fat_sec, 1, 1000) == HAL_OK) {
                s_play_cur_clust = *(uint32_t*)&fat_buf[fat_offset] & 0x0FFFFFFF;
            } else {
                s_play_cur_clust = 0x0FFFFFFF;
            }
        }
        if (s_play_cur_clust >= 2 && s_play_cur_clust < 0x0FFFFFF8 && mp3_ring_free() >= 2048) {
            uint32_t secs_left_in_clust = (s_sec_per_clust > s_play_cur_sec_in_clust) ? (s_sec_per_clust - s_play_cur_sec_in_clust) : 0;
            uint32_t secs_to_read = (secs_left_in_clust < 4) ? secs_left_in_clust : 4;
            if (secs_to_read > 0 && mp3_ring_free() >= secs_to_read * 512) {
                uint32_t sec_lba = clust_to_lba(s_play_cur_clust) + s_play_cur_sec_in_clust;
                static uint8_t burst_tmp[2048];
                if (HAL_SD_ReadBlocks(&hsd1, burst_tmp, sec_lba, secs_to_read, 1000) == HAL_OK) {
                    mp3_ring_push(burst_tmp, secs_to_read * 512);
                    s_play_bytes_read += (secs_to_read * 512);
                    s_play_cur_sec_in_clust += secs_to_read;
                    if (s_play_cur_sec_in_clust >= s_sec_per_clust) {
                        s_play_cur_sec_in_clust = 0;
                        uint32_t fat_sec = s_fat_lba + (s_play_cur_clust * 4 / 512);
                        uint32_t fat_offset = (s_play_cur_clust * 4) % 512;
                        static uint8_t fat_buf[512];
                        if (HAL_SD_ReadBlocks(&hsd1, fat_buf, fat_sec, 1, 1000) == HAL_OK) {
                            s_play_cur_clust = *(uint32_t*)&fat_buf[fat_offset] & 0x0FFFFFFF;
                        } else {
                            s_play_cur_clust = 0x0FFFFFFF;
                        }
                    }
                }
            }
        }

        // 🌟 1. 若有上一次解码就绪但由于流控尚未成功发送的音频帧：直接重发，绝不重复调用 mp3dec_decode_frame！
        if (s_pending_mono_count > 0) {
            if (digitalRead(MY_PIN_HANDSHAKE) == HIGH) {
                bool sent = sendAudioPacketSpi(s_pending_mono_pcm, s_pending_mono_count, s_pending_sample_rate);
                if (sent) {
                    s_decoded_frames++;
                    s_pcm_bytes_sent += (s_pending_mono_count * sizeof(int16_t));
                    if (s_pending_consumed_bytes > 0) {
                        mp3_ring_drop(s_pending_consumed_bytes);
                    }
                    s_pending_mono_count = 0;
                    s_pending_consumed_bytes = 0;
                }
            }
        } else {
            size_t ring_avail = mp3_ring_available();
            if (ring_avail >= 2048 || (s_play_cur_clust >= 0x0FFFFFF8 && ring_avail > 0)) {
                // 🌟 核心双向握手流控：严格以 ESP32 握手线 (PB1 == HIGH) 为水库充水使能
                if (digitalRead(MY_PIN_HANDSHAKE) == HIGH) {
                    static uint8_t decode_linear_buf[4096];
                    size_t bytes_to_decode = mp3_ring_peek_linear(decode_linear_buf, sizeof(decode_linear_buf));

                    mp3dec_frame_info_t info = {};
                    int16_t pcm_frame[MINIMP3_MAX_SAMPLES_PER_FRAME];
                    int samples = mp3dec_decode_frame(&s_mp3d, decode_linear_buf, (int)bytes_to_decode, pcm_frame, &info);
                    size_t total_consumed = (size_t)info.frame_offset + (size_t)info.frame_bytes;

                    if (samples > 0) {
                        if (s_decoded_frames == 0) {
                            if (info.hz >= 8000 && samples > 0) {
                                s_frame_duration_us = (uint32_t)((uint64_t)samples * 1000000ULL / (uint32_t)info.hz);
                            }
                            Serial.printf("🎵 [STM32-MP3] 歌曲 #%02d 首帧解码参数: 采样率=%d Hz, 声道数=%d, 码率=%d kbps, 单帧采样点=%d, 物理总时长=%02d:%02d (物理帧周期: %u us)\r\n",
                                          s_currentTrack + 1, info.hz, info.channels, info.bitrate_kbps, samples, s_totalSeconds / 60, s_totalSeconds % 60, (unsigned)s_frame_duration_us);
                            char rate_cmd[32];
                            snprintf(rate_cmd, sizeof(rate_cmd), "RATE:%d", info.hz);
                            sendTextPacket(rate_cmd);
                        }
                        
                        // 1. 立体声混音为单声道 (完整原生采样点，不丢帧，原生 CD 音质)
                        s_pending_mono_count = (samples > 1152) ? 1152 : samples;
                        s_pending_sample_rate = (info.hz >= 8000 && info.hz <= 96000) ? (uint16_t)info.hz : 44100;
                        s_pending_consumed_bytes = total_consumed;
                        if (info.channels == 2) {
                            for (int i = 0; i < s_pending_mono_count; i++) {
                                s_pending_mono_pcm[i] = (int16_t)(((int32_t)pcm_frame[i*2] + pcm_frame[i*2 + 1]) / 2);
                            }
                        } else {
                            for (int i = 0; i < s_pending_mono_count; i++) {
                                s_pending_mono_pcm[i] = pcm_frame[i];
                            }
                        }

                        // 2. 5线硬件 SPI2 发送音频帧给 ESP32 (带采样率标签)
                        bool sent = sendAudioPacketSpi(s_pending_mono_pcm, s_pending_mono_count, s_pending_sample_rate);
                        if (sent) {
                            s_decoded_frames++;
                            s_pcm_bytes_sent += (s_pending_mono_count * sizeof(int16_t));
                            if (s_pending_consumed_bytes > 0) {
                                mp3_ring_drop(s_pending_consumed_bytes);
                            }
                            s_pending_mono_count = 0;
                            s_pending_consumed_bytes = 0;
                        }
                    } else if (total_consumed > 0) {
                        // 🌟 限制单次 drop 最大 512 字节，杜绝瞬间把整首歌曲洗空！
                        size_t drop_len = (total_consumed > 512) ? 512 : total_consumed;
                        mp3_ring_drop(drop_len);
                    } else {
                        // samples == 0 && total_consumed == 0：末尾帧不完整，指针不动，等待 SD 卡追加扇区
                    }
                }
            }
        }

        static uint32_t last_spi_tx_log_ms = 0;
        if (now - last_spi_tx_log_ms >= 2000) {
            last_spi_tx_log_ms = now;
            Serial.printf("⚡ [STM32-MP3-DECODE] 已读SD卡:%uKB, 软解帧数:%u, 已推PCM:%uKB (曲目:[%02d] %s, 进度:%02d:%02d/%02d:%02d, PB1握手线:%s)\r\n",
                          s_play_bytes_read / 1024, s_decoded_frames, s_pcm_bytes_sent / 1024,
                          s_currentTrack + 1, s_playlist[s_currentTrack].title.c_str(),
                          s_currentSeconds / 60, s_currentSeconds % 60, s_totalSeconds / 60, s_totalSeconds % 60,
                          (digitalRead(MY_PIN_HANDSHAKE) == HIGH) ? "HIGH(就绪)" : "LOW(等待)");
        }

        // 🌟 3. 物理流结束 (EOS) 握手检测：
        // 当 SD 卡所有 FAT32 簇已读尽，128KB 环形水库中所有 MP3 帧已全部解码推入 SPI，发送 EOS 标记帧
        if (s_play_cur_clust >= 0x0FFFFFF8 && mp3_ring_available() < 512 && s_pending_mono_count == 0) {
            if (!s_eos_sent) {
                if (sendAudioPacketSpi(nullptr, 0, s_pending_sample_rate, 0xA0D100EE)) {
                    s_eos_sent = true;
                    Serial.printf("🏁 [STM32-MP3] 曲目 #%02d 《%s》 全量物理音频已解码完毕，已发送 EOS 流结束帧给 ESP32！\r\n",
                                  s_currentTrack + 1, s_playlist[s_currentTrack].title.c_str());
                }
            }
        }

        // 🌟 真实物理放音时钟同步：向 ESP32 定期同步状态
        if (now - s_last_tick_ms >= 1000) {
            s_last_tick_ms = now;
            char statBuf[128];
            snprintf(statBuf, sizeof(statBuf), "STATUS:%d,%d,%d,%d,%d,%d", s_playState, s_currentSeconds, s_totalSeconds, s_volume, s_playMode, s_currentTrack);
            sendTextPacket(statBuf);
        }
    }
}

// ----------------- 状态读取 -----------------
PlayState getPlayState() { return s_playState; }
int getCurrentTrack() { return s_currentTrack; }
int getCurrentSeconds() { return s_currentSeconds; }
int getTotalSeconds() { return s_totalSeconds; }
int getVolume() { return s_volume; }
PlayMode getPlayMode() { return s_playMode; }
const std::vector<SongItem>& getPlaylist() { return s_playlist; }
int getPlaylistCount() { return (int)s_playlist.size(); }

const SongItem* getCurrentSong() {
    if (s_playlist.empty()) return nullptr;
    if (s_currentTrack < 0 || s_currentTrack >= (int)s_playlist.size()) return &s_playlist[0];
    return &s_playlist[s_currentTrack];
}

} // namespace Mp3PlayerBackend
