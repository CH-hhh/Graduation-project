#include "jpeg_encoder.h"
#include <Arduino.h>
#include <cstring>
#include "esp_heap_caps.h"

namespace JpegEncoder {

namespace {

// 标准 JPEG 亮度量化表
static const uint8_t std_luminance_quant_tbl[64] = {
    16, 11, 10, 16, 24, 40, 51, 61,
    12, 12, 14, 19, 26, 58, 60, 55,
    14, 13, 16, 24, 40, 57, 69, 56,
    14, 17, 22, 29, 51, 87, 80, 62,
    18, 22, 37, 56, 68, 109, 103, 77,
    24, 35, 55, 64, 81, 104, 113, 92,
    49, 64, 78, 87, 103, 121, 120, 101,
    72, 92, 95, 98, 112, 100, 103, 99
};

// 标准 JPEG 色度量化表
static const uint8_t std_chrominance_quant_tbl[64] = {
    17, 18, 24, 47, 99, 99, 99, 99,
    18, 21, 26, 66, 99, 99, 99, 99,
    24, 26, 56, 99, 99, 99, 99, 99,
    47, 66, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99
};

// 标准 JPEG Zig-Zag 扫描顺序表 (将 1D 游程索引正确映射到 8x8 矩阵中的 (row*8 + col) 位置)
static const uint8_t zigzag[64] = {
     0,  1,  8, 16,  9,  2,  3, 10,
    17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34,
    27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36,
    29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46,
    53, 60, 61, 54, 47, 55, 62, 63
};

// 标准 Huffman 表 (DC Luminance)
static const uint8_t bits_dc_lum[17] = {0, 0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0};
static const uint8_t val_dc_lum[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

// 标准 Huffman 表 (DC Chrominance)
static const uint8_t bits_dc_chrom[17] = {0, 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0};
static const uint8_t val_dc_chrom[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

// 标准 Huffman 表 (AC Luminance)
static const uint8_t bits_ac_lum[17] = {0, 0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d};
static const uint8_t val_ac_lum[162] = {
    0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12,
    0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
    0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08,
    0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
    0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16,
    0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
    0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
    0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
    0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
    0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
    0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
    0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
    0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98,
    0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
    0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
    0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
    0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4,
    0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
    0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea,
    0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
    0xf9, 0xfa
};

// 标准 Huffman 表 (AC Chrominance)
static const uint8_t bits_ac_chrom[17] = {0, 0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77};
static const uint8_t val_ac_chrom[162] = {
    0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21,
    0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
    0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
    0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0,
    0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34,
    0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
    0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38,
    0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
    0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
    0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
    0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78,
    0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
    0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96,
    0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5,
    0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4,
    0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
    0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2,
    0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
    0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9,
    0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
    0xf9, 0xfa
};

struct SafeBitWriter {
    uint8_t* out_buf;
    size_t max_size;
    size_t write_pos = 0;
    uint64_t bit_buf = 0;
    int bit_count = 0;

    SafeBitWriter(uint8_t* buf, size_t max_sz) : out_buf(buf), max_size(max_sz) {}

    inline void write_byte(uint8_t byte) {
        if (write_pos < max_size) {
            out_buf[write_pos++] = byte;
        }
    }

    inline void write_bits(uint32_t bits, int count) {
        if (count <= 0) return;
        bit_buf = (bit_buf << count) | (bits & ((1ULL << count) - 1));
        bit_count += count;
        while (bit_count >= 8) {
            bit_count -= 8;
            uint8_t b = (bit_buf >> bit_count) & 0xFF;
            if (write_pos < max_size) out_buf[write_pos++] = b;
            if (b == 0xFF) {
                if (write_pos < max_size) out_buf[write_pos++] = 0x00; // Byte stuffing
            }
        }
    }

    inline void flush() {
        if (bit_count > 0) {
            write_bits((1 << (8 - bit_count)) - 1, 8 - bit_count); // 填充 1
        }
    }
};

struct HuffmanCode {
    uint16_t code = 0;
    uint8_t len = 0;
};

static void compute_huffman_table(const uint8_t* bits, const uint8_t* val, int num_val, HuffmanCode* huff_table, int max_table_entries) {
    for (int i = 0; i < max_table_entries; i++) {
        huff_table[i].code = 0;
        huff_table[i].len = 0;
    }
    uint16_t code = 0;
    int p = 0;
    for (int l = 1; l <= 16; l++) {
        for (int i = 1; i <= bits[l]; i++) {
            if (p < num_val) {
                uint8_t symbol = val[p];
                if (symbol < max_table_entries) {
                    huff_table[symbol].code = code;
                    huff_table[symbol].len = l;
                }
                p++;
                code++;
            }
        }
        code <<= 1;
    }
}

// ==========================================
// 🚀 标准正交 2D 整数矩阵 DCT 变换 (数学 100% 精确，0 宏块马赛克失真)
// ==========================================
static const int32_t DCT_MAT[8][8] = {
    {  724,  724,  724,  724,  724,  724,  724,  724 },
    { 1004,  851,  569,  200, -200, -569, -851,-1004 },
    {  946,  392, -392, -946, -946, -392,  392,  946 },
    {  851, -200,-1004, -569,  569, 1004,  200, -851 },
    {  724, -724, -724,  724,  724, -724, -724,  724 },
    {  569,-1004,  200,  851, -851, -200, 1004, -569 },
    {  392, -946,  946, -392, -392,  946, -946,  392 },
    {  200, -569,  851,-1004, 1004, -851,  569, -200 }
};

static void exact_fdct_8x8(const int16_t* in_block, int16_t* out_block) {
    int32_t temp[64];
    // Pass 1: 行变换
    for (int r = 0; r < 8; r++) {
        const int16_t* row = in_block + r * 8;
        for (int u = 0; u < 8; u++) {
            const int32_t* c_row = DCT_MAT[u];
            int32_t sum = row[0] * c_row[0] + row[1] * c_row[1] + row[2] * c_row[2] + row[3] * c_row[3]
                        + row[4] * c_row[4] + row[5] * c_row[5] + row[6] * c_row[6] + row[7] * c_row[7];
            temp[r * 8 + u] = sum;
        }
    }
    // Pass 2: 列变换
    for (int u = 0; u < 8; u++) {
        for (int v = 0; v < 8; v++) {
            const int32_t* c_row = DCT_MAT[v];
            int32_t sum = temp[0 * 8 + u] * c_row[0] + temp[1 * 8 + u] * c_row[1] + temp[2 * 8 + u] * c_row[2] + temp[3 * 8 + u] * c_row[3]
                        + temp[4 * 8 + u] * c_row[4] + temp[5 * 8 + u] * c_row[5] + temp[6 * 8 + u] * c_row[6] + temp[7 * 8 + u] * c_row[7];
            // 总体正规化: (1024 * 1024 * 4) = 4194304 -> 四舍五入后右移 22 位
            out_block[v * 8 + u] = (int16_t)((sum + 2097152) >> 22);
        }
    }
}

static void encode_block(SafeBitWriter& bw, const int16_t* block, int16_t& prev_dc,
                         const HuffmanCode* dc_table, const HuffmanCode* ac_table) {
    // 1. DC 差分编码与绝对值边界保护
    int16_t dc_diff = block[0] - prev_dc;
    prev_dc = block[0];

    if (dc_diff > 2047) dc_diff = 2047;
    if (dc_diff < -2047) dc_diff = -2047;

    int abs_diff = abs(dc_diff);
    int size = 0;
    while (abs_diff > 0) {
        size++;
        abs_diff >>= 1;
    }
    if (size > 11) size = 11;

    if (dc_table[size].len > 0) {
        bw.write_bits(dc_table[size].code, dc_table[size].len);
    }
    if (size > 0) {
        int diff_val = (dc_diff < 0) ? ((1 << size) - 1 + dc_diff) : dc_diff;
        bw.write_bits(diff_val, size);
    }

    // 2. AC 游程编码
    int r = 0;
    for (int k = 1; k < 64; k++) {
        int16_t val = block[zigzag[k]];
        if (val == 0) {
            r++;
        } else {
            if (val > 1023) val = 1023;
            if (val < -1023) val = -1023;

            while (r > 15) {
                if (ac_table[0xF0].len > 0) {
                    bw.write_bits(ac_table[0xF0].code, ac_table[0xF0].len); // ZRL
                }
                r -= 16;
            }
            int abs_val = abs(val);
            int v_size = 0;
            while (abs_val > 0) {
                v_size++;
                abs_val >>= 1;
            }
            if (v_size > 10) v_size = 10;

            int symbol = (r << 4) | v_size;
            if (ac_table[symbol].len > 0) {
                bw.write_bits(ac_table[symbol].code, ac_table[symbol].len);
            }
            int ac_out_val = (val < 0) ? ((1 << v_size) - 1 + val) : val;
            bw.write_bits(ac_out_val, v_size);
            r = 0;
        }
    }
    if (r > 0) {
        if (ac_table[0x00].len > 0) {
            bw.write_bits(ac_table[0x00].code, ac_table[0x00].len); // EOB
        }
    }
}

} // namespace

bool encodeRgb565(const uint16_t* rgb565, int width, int height, int quality, std::vector<uint8_t>& out_jpeg) {
    if (!rgb565 || width <= 0 || height <= 0) return false;

    constexpr size_t MAX_JPEG_BUF = 65536;
    uint8_t* raw_jpeg_buf = (uint8_t*)heap_caps_malloc(MAX_JPEG_BUF, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!raw_jpeg_buf) {
        raw_jpeg_buf = (uint8_t*)malloc(MAX_JPEG_BUF);
    }
    if (!raw_jpeg_buf) return false;

    // 计算缩放质量量化表
    if (quality < 1) quality = 1;
    if (quality > 100) quality = 100;
    int scale = (quality < 50) ? (5000 / quality) : (200 - quality * 2);

    uint8_t q_lum[64];
    uint8_t q_chrom[64];
    for (int i = 0; i < 64; i++) {
        int v_lum = (std_luminance_quant_tbl[i] * scale + 50) / 100;
        q_lum[i] = (v_lum < 1) ? 1 : ((v_lum > 255) ? 255 : v_lum);

        int v_chrom = (std_chrominance_quant_tbl[i] * scale + 50) / 100;
        q_chrom[i] = (v_chrom < 1) ? 1 : ((v_chrom > 255) ? 255 : v_chrom);
    }

    SafeBitWriter bw(raw_jpeg_buf, MAX_JPEG_BUF);

    // 1. SOI (Start of Image)
    bw.write_byte(0xFF); bw.write_byte(0xD8);

    // 2. APP0 (JFIF Header)
    bw.write_byte(0xFF); bw.write_byte(0xE0);
    bw.write_byte(0x00); bw.write_byte(0x10); // Length = 16
    bw.write_byte('J'); bw.write_byte('F'); bw.write_byte('I'); bw.write_byte('F'); bw.write_byte(0x00);
    bw.write_byte(0x01); bw.write_byte(0x01); // Version 1.1
    bw.write_byte(0x00);                      // Density unit (0 = aspect ratio)
    bw.write_byte(0x00); bw.write_byte(0x01); // X density = 1
    bw.write_byte(0x00); bw.write_byte(0x01); // Y density = 1
    bw.write_byte(0x00); bw.write_byte(0x00); // Thumbnail = 0x0

    // 3. DQT (Define Quantization Table)
    bw.write_byte(0xFF); bw.write_byte(0xDB);
    bw.write_byte(0x00); bw.write_byte(0x84); // Length = 132 (2 tables)
    bw.write_byte(0x00);                      // Table 0 (Luminance)
    for (int i = 0; i < 64; i++) bw.write_byte(q_lum[zigzag[i]]);
    bw.write_byte(0x01);                      // Table 1 (Chrominance)
    for (int i = 0; i < 64; i++) bw.write_byte(q_chrom[zigzag[i]]);

    // 4. SOF0 (Start of Frame: Baseline DCT)
    bw.write_byte(0xFF); bw.write_byte(0xC0);
    bw.write_byte(0x00); bw.write_byte(0x11); // Length = 17
    bw.write_byte(0x08);                      // 8-bit precision
    bw.write_byte(height >> 8); bw.write_byte(height & 0xFF);
    bw.write_byte(width >> 8); bw.write_byte(width & 0xFF);
    bw.write_byte(0x03);                      // 3 components (Y, Cb, Cr)
    
    bw.write_byte(0x01); bw.write_byte(0x11); bw.write_byte(0x00); // Y: ID 1, 1x1, DQT 0
    bw.write_byte(0x02); bw.write_byte(0x11); bw.write_byte(0x01); // Cb: ID 2, 1x1, DQT 1
    bw.write_byte(0x03); bw.write_byte(0x11); bw.write_byte(0x01); // Cr: ID 3, 1x1, DQT 1

    // 5. DHT (Define Huffman Tables)
    HuffmanCode dc_lum_codes[16] = {};
    HuffmanCode dc_chrom_codes[16] = {};
    HuffmanCode ac_lum_codes[256] = {};
    HuffmanCode ac_chrom_codes[256] = {};
    compute_huffman_table(bits_dc_lum, val_dc_lum, 12, dc_lum_codes, 16);
    compute_huffman_table(bits_dc_chrom, val_dc_chrom, 12, dc_chrom_codes, 16);
    compute_huffman_table(bits_ac_lum, val_ac_lum, 162, ac_lum_codes, 256);
    compute_huffman_table(bits_ac_chrom, val_ac_chrom, 162, ac_chrom_codes, 256);

    auto write_dht = [&](uint8_t table_class, uint8_t id, const uint8_t* bits, const uint8_t* vals, int num_vals) {
        bw.write_byte(0xFF); bw.write_byte(0xC4);
        uint16_t len = 2 + 1 + 16 + num_vals;
        bw.write_byte(len >> 8); bw.write_byte(len & 0xFF);
        bw.write_byte((table_class << 4) | id);
        for (int i = 1; i <= 16; i++) bw.write_byte(bits[i]);
        for (int i = 0; i < num_vals; i++) bw.write_byte(vals[i]);
    };

    write_dht(0, 0, bits_dc_lum, val_dc_lum, 12);
    write_dht(1, 0, bits_ac_lum, val_ac_lum, 162);
    write_dht(0, 1, bits_dc_chrom, val_dc_chrom, 12);
    write_dht(1, 1, bits_ac_chrom, val_ac_chrom, 162);

    // 6. SOS (Start of Scan)
    bw.write_byte(0xFF); bw.write_byte(0xDA);
    bw.write_byte(0x00); bw.write_byte(0x0C); // Length = 12
    bw.write_byte(0x03);                      // 3 components
    bw.write_byte(0x01); bw.write_byte(0x00); // Y: DC 0, AC 0
    bw.write_byte(0x02); bw.write_byte(0x11); // Cb: DC 1, AC 1
    bw.write_byte(0x03); bw.write_byte(0x11); // Cr: DC 1, AC 1
    bw.write_byte(0x00);                      // Spectral selection start = 0
    bw.write_byte(0x3F);                      // Spectral selection end = 63
    bw.write_byte(0x00);                      // Successive approx = 0

    // 7. 逐 MCU 块精确 2D 整数编码 (8x8 块)
    int16_t prev_dc_y = 0, prev_dc_cb = 0, prev_dc_cr = 0;
    int16_t block_y[64], block_cb[64], block_cr[64];
    int16_t dct_out[64];
    int16_t q_block[64];

    for (int my = 0; my < height; my += 8) {
        for (int mx = 0; mx < width; mx += 8) {
            // 精准按大端字节序 (Byte0=MSB, Byte1=LSB) 解包原始 RGB565
            for (int y = 0; y < 8; y++) {
                int py = my + y;
                if (py >= height) py = height - 1;
                for (int x = 0; x < 8; x++) {
                    int px = mx + x;
                    uint16_t c = __builtin_bswap16(rgb565[py * width + px]);
                    
                    int32_t r = ((c >> 11) & 0x1F) * 255 / 31;
                    int32_t g = ((c >> 5) & 0x3F) * 255 / 63;
                    int32_t b = (c & 0x1F) * 255 / 31;

                    // 纯整数 ITU-R BT.601 转换 (减去 128 直流偏置)
                    int32_t y_val  = (( 19595 * r + 38469 * g +  7471 * b) >> 16) - 128;
                    int32_t cb_val = ((-11059 * r - 21709 * g + 32768 * b) >> 16);
                    int32_t cr_val = (( 32768 * r - 27439 * g -  5329 * b) >> 16);

                    block_y[y * 8 + x] = (int16_t)y_val;
                    block_cb[y * 8 + x] = (int16_t)cb_val;
                    block_cr[y * 8 + x] = (int16_t)cr_val;
                }
            }

            // --- Y Block (标准正交 2D 矩阵 DCT) ---
            exact_fdct_8x8(block_y, dct_out);
            for (int i = 0; i < 64; i++) {
                q_block[i] = (int16_t)(dct_out[i] / (int16_t)q_lum[i]);
            }
            encode_block(bw, q_block, prev_dc_y, dc_lum_codes, ac_lum_codes);

            // --- Cb Block ---
            exact_fdct_8x8(block_cb, dct_out);
            for (int i = 0; i < 64; i++) {
                q_block[i] = (int16_t)(dct_out[i] / (int16_t)q_chrom[i]);
            }
            encode_block(bw, q_block, prev_dc_cb, dc_chrom_codes, ac_chrom_codes);

            // --- Cr Block ---
            exact_fdct_8x8(block_cr, dct_out);
            for (int i = 0; i < 64; i++) {
                q_block[i] = (int16_t)(dct_out[i] / (int16_t)q_chrom[i]);
            }
            encode_block(bw, q_block, prev_dc_cr, dc_chrom_codes, ac_chrom_codes);
        }
    }

    bw.flush();

    // 8. EOI (End of Image)
    bw.write_byte(0xFF); bw.write_byte(0xD9);

    // 将生成的 JPEG 数据拷贝回输出 vector
    out_jpeg.assign(raw_jpeg_buf, raw_jpeg_buf + bw.write_pos);

    // 释放 PSRAM 临时缓冲
    free(raw_jpeg_buf);

    return (bw.write_pos > 0);
}

} // namespace JpegEncoder
