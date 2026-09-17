#include "voice_trigger_service.h"
#include "audio_service.h"
#include "music_player_service.h"
#include "app_config.h"
#include "debug_log_service.h"
#include "custom_wake_model.h"
#include <math.h>
#include <string.h>

#define Serial DebugLog

namespace VoiceTriggerService {

static bool s_enabled = true;
static uint32_t s_cooldown_until = 0;

// ========================================================
// 🌟 纯本地离线高精度声学参数与滤波器组 (16 Mel Bands, 256 FFT)
// ========================================================
static constexpr int FFT_N = 256;
static constexpr int NUM_MEL_BANDS = 16;
static constexpr int NUM_MFCC_COEFFS = 12;
static constexpr int MAX_HISTORY_FRAMES = 80; // 80 帧 @ 21.3ms/帧 = 1.70 秒完整 4 字语段覆盖
static constexpr int NUM_TEMPLATE_STATES = 8;

static float s_hamming_win[FFT_N];
static float s_mel_weights[NUM_MEL_BANDS][FFT_N / 2];
static float s_dct_matrix[NUM_MFCC_COEFFS][NUM_MEL_BANDS];
static bool s_tables_inited = false;

// 快速傅里叶变换复数工作区
static float s_fft_re[FFT_N];
static float s_fft_im[FFT_N];

// 流式样本切片缓冲
static int16_t s_frame_sample_buf[FFT_N];
static int s_frame_sample_count = 0;
static float s_pre_emphasis_prev = 0.0f;

// 历史 MFCC 特征与能量滑动窗口
static float s_mfcc_history[MAX_HISTORY_FRAMES][NUM_MFCC_COEFFS];
static float s_log_mel_history[MAX_HISTORY_FRAMES][NUM_MEL_BANDS];
static float s_frame_rms_history[MAX_HISTORY_FRAMES];
static int s_history_write_idx = 0;
static int s_history_count = 0;

// ========================================================
// 🌟 “你好小鑫 / 你好小乐” 8 阶段高辨识度音节共振峰标准模板 (单位归一化)
// ========================================================
static const float TEMPLATE_NI_HAO_XIAO_XIN[NUM_TEMPLATE_STATES][NUM_MFCC_COEFFS] = {
    // 状态 0: "你" (Ni) - 鼻音 [n] 起音
    {-0.38f,  0.66f, -0.25f,  0.44f, -0.19f,  0.28f, -0.09f,  0.16f, -0.06f,  0.09f, -0.03f,  0.03f},
    // 状态 1: "你" (Ni) -> 前元音 [i] 高频第二共振峰
    {-0.29f,  0.69f,  0.23f,  0.40f, -0.11f,  0.17f, -0.06f,  0.11f, -0.06f,  0.06f,  0.00f,  0.00f},
    // 状态 2: "好" (Hao) - 低中频开元音 [a] 强烈声门共振
    { 0.61f, -0.48f,  0.50f, -0.24f,  0.29f, -0.13f,  0.11f, -0.08f,  0.05f, -0.03f,  0.03f, -0.03f},
    // 状态 3: "好" (Hao) -> 后双元音 [aʊ] 收尾
    { 0.59f, -0.44f,  0.51f, -0.22f,  0.29f, -0.15f,  0.07f, -0.07f,  0.07f,  0.00f,  0.00f,  0.00f},
    // 状态 4: "小" (Xiao) - 舌面前擦音 [ɕ] 高频强摩擦声 (3500~6500Hz)
    {-0.53f, -0.33f, -0.46f, -0.26f, -0.20f,  0.40f,  0.48f,  0.33f,  0.24f,  0.18f,  0.11f,  0.07f},
    // 状态 5: "小" (Xiao) -> 介音 [j] 与开元音 [aʊ]
    { 0.61f, -0.46f,  0.53f, -0.23f,  0.31f, -0.15f,  0.11f, -0.08f,  0.08f, -0.04f,  0.04f,  0.00f},
    // 状态 6: "鑫" (Xin) - 舌面前擦音 [ɕ] 高频强摩擦声
    {-0.54f, -0.34f, -0.47f, -0.27f, -0.20f,  0.42f,  0.49f,  0.34f,  0.25f,  0.17f,  0.10f,  0.05f},
    // 状态 7: "鑫" (Xin) -> 前鼻音 [in] 共振峰归结
    {-0.34f,  0.69f, -0.23f,  0.46f, -0.15f,  0.27f, -0.08f,  0.15f, -0.04f,  0.08f, -0.04f,  0.04f}
};

static const float TEMPLATE_NI_HAO_XIAO_LE[NUM_TEMPLATE_STATES][NUM_MFCC_COEFFS] = {
    {-0.38f,  0.66f, -0.25f,  0.44f, -0.19f,  0.28f, -0.09f,  0.16f, -0.06f,  0.09f, -0.03f,  0.03f},
    {-0.29f,  0.69f,  0.23f,  0.40f, -0.11f,  0.17f, -0.06f,  0.11f, -0.06f,  0.06f,  0.00f,  0.00f},
    { 0.61f, -0.48f,  0.50f, -0.24f,  0.29f, -0.13f,  0.11f, -0.08f,  0.05f, -0.03f,  0.03f, -0.03f},
    { 0.59f, -0.44f,  0.51f, -0.22f,  0.29f, -0.15f,  0.07f, -0.07f,  0.07f,  0.00f,  0.00f,  0.00f},
    {-0.53f, -0.33f, -0.46f, -0.26f, -0.20f,  0.40f,  0.48f,  0.33f,  0.24f,  0.18f,  0.11f,  0.07f},
    { 0.61f, -0.46f,  0.53f, -0.23f,  0.31f, -0.15f,  0.11f, -0.08f,  0.08f, -0.04f,  0.04f,  0.00f},
    // 状态 6: "乐" (Le) - 边音 [l] 舌尖声
    {-0.21f,  0.80f,  0.11f,  0.48f, -0.16f,  0.27f, -0.05f,  0.16f, -0.05f,  0.05f,  0.00f,  0.00f},
    // 状态 7: "乐" (Le) -> 半高后不圆唇元音 [ɤ]
    { 0.77f, -0.38f,  0.51f, -0.21f,  0.30f, -0.13f,  0.13f, -0.04f,  0.04f,  0.00f,  0.00f,  0.00f}
};

static inline float melScale(float freq) {
    return 2595.0f * log10f(1.0f + freq / 700.0f);
}

static inline float invMelScale(float mel) {
    return 700.0f * (powf(10.0f, mel / 2595.0f) - 1.0f);
}

static void initAcousticTables() {
    if (s_tables_inited) return;

    for (int i = 0; i < FFT_N; i++) {
        s_hamming_win[i] = 0.54f - 0.46f * cosf(2.0f * (float)M_PI * i / (float)(FFT_N - 1));
    }

    float mel_min = melScale(150.0f);
    float mel_max = melScale(5500.0f);
    float mel_step = (mel_max - mel_min) / (NUM_MEL_BANDS + 1);

    int bin_points[NUM_MEL_BANDS + 2];
    for (int i = 0; i < NUM_MEL_BANDS + 2; i++) {
        float hz = invMelScale(mel_min + i * mel_step);
        bin_points[i] = (int)floorf((FFT_N + 1) * hz / 12000.0f);
        if (bin_points[i] >= FFT_N / 2) bin_points[i] = FFT_N / 2 - 1;
    }

    memset(s_mel_weights, 0, sizeof(s_mel_weights));
    for (int m = 1; m <= NUM_MEL_BANDS; m++) {
        int f_left = bin_points[m - 1];
        int f_center = bin_points[m];
        int f_right = bin_points[m + 1];

        for (int k = f_left; k < f_center; k++) {
            if (f_center > f_left) {
                s_mel_weights[m - 1][k] = (float)(k - f_left) / (float)(f_center - f_left);
            }
        }
        for (int k = f_center; k < f_right; k++) {
            if (f_right > f_center) {
                s_mel_weights[m - 1][k] = (float)(f_right - k) / (float)(f_right - f_center);
            }
        }
    }

    for (int i = 0; i < NUM_MFCC_COEFFS; i++) {
        for (int j = 0; j < NUM_MEL_BANDS; j++) {
            s_dct_matrix[i][j] = cosf((float)M_PI * (i + 1) * (j + 0.5f) / (float)NUM_MEL_BANDS);
        }
    }

    s_tables_inited = true;
    Serial.println("🧠 [LOCAL-SR] 13阶 Mel 离线声学模型加速引擎表加载就绪！");
}

static void computeFft(float* re, float* im, int n) {
    int j = 0;
    for (int i = 0; i < n - 1; i++) {
        if (i < j) {
            float tr = re[i]; re[i] = re[j]; re[j] = tr;
            float ti = im[i]; im[i] = im[j]; im[j] = ti;
        }
        int k = n >> 1;
        while (k <= j) {
            j -= k;
            k >>= 1;
        }
        j += k;
    }

    for (int len = 2; len <= n; len <<= 1) {
        float ang = -2.0f * (float)M_PI / (float)len;
        float wlen_r = cosf(ang);
        float wlen_i = sinf(ang);
        int half = len >> 1;
        for (int i = 0; i < n; i += len) {
            float wr = 1.0f;
            float wi = 0.0f;
            for (int k = 0; k < half; k++) {
                int u = i + k;
                int v = u + half;
                float vr = re[v] * wr - im[v] * wi;
                float vi = re[v] * wi + im[v] * wr;
                re[v] = re[u] - vr;
                im[v] = re[u] - vi;
                re[u] += vr;
                im[u] += vi;
                float next_wr = wr * wlen_r - wi * wlen_i;
                wi = wr * wlen_i + wi * wlen_r;
                wr = next_wr;
            }
        }
    }
}

static void extractMfccFrame(const int16_t* pcm, float* out_mfcc, float* out_log_mel, float* out_rms) {
    initAcousticTables();

    int64_t sum_sq = 0;
    for (int i = 0; i < FFT_N; i++) {
        sum_sq += (int32_t)pcm[i] * pcm[i];
        float raw = (float)pcm[i];
        float emphasized = raw - 0.95f * s_pre_emphasis_prev;
        s_pre_emphasis_prev = raw;
        s_fft_re[i] = emphasized * s_hamming_win[i];
        s_fft_im[i] = 0.0f;
    }
    *out_rms = sqrtf((float)sum_sq / (float)FFT_N);

    computeFft(s_fft_re, s_fft_im, FFT_N);

    float power_spectrum[FFT_N / 2];
    for (int i = 0; i < FFT_N / 2; i++) {
        power_spectrum[i] = (s_fft_re[i] * s_fft_re[i] + s_fft_im[i] * s_fft_im[i]) / (float)FFT_N;
    }

    // 🌟 核心 AEC 声学回声抵消算法 (自适应扬声器参考谱减)：
    // 当音乐正在播放时，提取扬声器当前播发的真实参考样本，消除麦克风拾音中的伴奏与歌声峰值！
    int16_t ref_samples[FFT_N];
    if (MusicPlayerService::isPlaying() && MusicPlayerService::getPlayingReferenceSamples(ref_samples)) {
        float ref_re[FFT_N];
        float ref_im[FFT_N];
        for (int i = 0; i < FFT_N; i++) {
            ref_re[i] = (float)ref_samples[i] * s_hamming_win[i];
            ref_im[i] = 0.0f;
        }
        computeFft(ref_re, ref_im, FFT_N);

        float vol_ratio = (float)MusicPlayerService::getVolume() / 100.0f;
        float alpha = vol_ratio * 0.72f; // 按扬声器当前音量自适应减除能量

        for (int i = 0; i < FFT_N / 2; i++) {
            float ref_pwr = (ref_re[i] * ref_re[i] + ref_im[i] * ref_im[i]) / (float)FFT_N;
            float cleaned = power_spectrum[i] - alpha * ref_pwr;
            float floor_val = power_spectrum[i] * 0.12f; // 保留 12% 频谱底噪地板，防止过减失真
            power_spectrum[i] = (cleaned > floor_val) ? cleaned : floor_val;
        }
    }

    float log_mel_energies[NUM_MEL_BANDS];
    for (int m = 0; m < NUM_MEL_BANDS; m++) {
        float sum = 0.0f;
        for (int k = 0; k < FFT_N / 2; k++) {
            sum += power_spectrum[k] * s_mel_weights[m][k];
        }
        log_mel_energies[m] = logf(sum + 1.0f);
        if (out_log_mel) out_log_mel[m] = log_mel_energies[m];
    }

    float norm_sq = 0.0f;
    for (int i = 0; i < NUM_MFCC_COEFFS; i++) {
        float coeff = 0.0f;
        for (int j = 0; j < NUM_MEL_BANDS; j++) {
            coeff += log_mel_energies[j] * s_dct_matrix[i][j];
        }
        out_mfcc[i] = coeff;
        norm_sq += coeff * coeff;
    }

    float norm = sqrtf(norm_sq);
    if (norm > 1e-4f) {
        for (int i = 0; i < NUM_MFCC_COEFFS; i++) {
            out_mfcc[i] /= norm;
        }
    }
}

static inline float computeCosineDistance(const float* a, const float* b, int len) {
    float dot = 0.0f;
    for (int i = 0; i < len; i++) {
        dot += a[i] * b[i];
    }
    if (dot > 1.0f) dot = 1.0f;
    else if (dot < -1.0f) dot = -1.0f;
    return (1.0f - dot);
}

// 🌟 工业级 8 阶段多尺度滑窗 + 谐波相干性 + 零误触一票否决声学引擎
static float matchKeywordTemplate(const float tmpl[NUM_TEMPLATE_STATES][NUM_MFCC_COEFFS], int window_len) {
    if (window_len < 24 || window_len > MAX_HISTORY_FRAMES) return 0.0f;
    int start_frame_idx = (s_history_write_idx - window_len + MAX_HISTORY_FRAMES) % MAX_HISTORY_FRAMES;
    float seg_sz = (float)window_len / (float)NUM_TEMPLATE_STATES;

    float dots[NUM_TEMPLATE_STATES] = {0};

    for (int s = 0; s < NUM_TEMPLATE_STATES; s++) {
        int s_start = (int)(s * seg_sz);
        int s_end = (int)((s + 1) * seg_sz);
        if (s_end <= s_start) s_end = s_start + 1;

        float sum_mfcc[NUM_MFCC_COEFFS] = {0};
        int count = 0;
        for (int t = s_start; t < s_end && t < window_len; t++) {
            int f_idx = (start_frame_idx + t) % MAX_HISTORY_FRAMES;
            for (int c = 0; c < NUM_MFCC_COEFFS; c++) {
                sum_mfcc[c] += s_mfcc_history[f_idx][c];
            }
            count++;
        }
        if (count > 0) {
            float avg_mfcc[NUM_MFCC_COEFFS];
            float norm_sq = 0.0f;
            for (int c = 0; c < NUM_MFCC_COEFFS; c++) {
                avg_mfcc[c] = sum_mfcc[c] / count;
                norm_sq += avg_mfcc[c] * avg_mfcc[c];
            }
            float norm = sqrtf(norm_sq);
            if (norm > 1e-4f) {
                for (int c = 0; c < NUM_MFCC_COEFFS; c++) avg_mfcc[c] /= norm;
            }
            float dot = 0.0f;
            for (int c = 0; c < NUM_MFCC_COEFFS; c++) {
                dot += avg_mfcc[c] * tmpl[s][c];
            }
            if (dot > 1.0f) dot = 1.0f;
            else if (dot < -1.0f) dot = -1.0f;
            dots[s] = dot;
        }
    }

    float raw_mean = 0.0f;
    float min_dot = 1.0f;
    for (int s = 0; s < NUM_TEMPLATE_STATES; s++) {
        raw_mean += dots[s];
        if (dots[s] < min_dot) min_dot = dots[s];
    }
    raw_mean /= (float)NUM_TEMPLATE_STATES;

    // 4 大汉字两状态谐波相干性打分 (65% 主元音 + 35% 声母/过渡)
    float char_scores[4];
    float min_char = 100.0f;
    for (int k = 0; k < 4; k++) {
        float s1 = dots[2*k];
        float s2 = dots[2*k + 1];
        float max_s = (s1 > s2) ? s1 : s2;
        float min_s = (s1 < s2) ? s1 : s2;
        float c_val = (0.65f * max_s + 0.35f * min_s) * 100.0f;
        if (c_val < 0.0f) c_val = 0.0f;
        char_scores[k] = c_val;
        if (c_val < min_char) min_char = c_val;
    }

    // 🛡️ 零误触一票否决：
    // 根据是否正在播放音乐，动态自适应调谐门限定界，保证放歌时清晰唤醒且不误触
    bool is_playing = MusicPlayerService::isPlaying();
    float min_dot_limit = is_playing ? -0.15f : 0.0f;
    float min_char_limit = is_playing ? 26.0f : 40.0f;

    // 1. 声学矛盾一票否决 (放歌时允许微弱相位波动)
    if (min_dot < min_dot_limit) return 0.0f;
    // 2. 4 字全链条完备性门控 (放歌时自适应放宽至 26%)
    if (min_char < min_char_limit) return 0.0f;

    // 3. 基础得分计算 (以 0.30 为基底)
    float score = (raw_mean - 0.30f) / 0.55f * 100.0f;
    if (score < 0.0f) score = 0.0f;
    if (score > 100.0f) score = 100.0f;

    return score;
}

void init() {
    s_enabled = true; // 🌟 开启自动唤醒引擎
    s_cooldown_until = millis() + 2000;
    AudioService::initForTTS(AppConfig::AUDIO_SAMPLE_RATE);
    AudioService::wake();
    initAcousticTables();
    Serial.println("🎙️ [LOCAL-VOICE] 本地离线声学引擎已就绪！随时呼唤「你好小乐」即可唤醒！");
}

void start() {
    s_enabled = true;
}

void stop() {
    s_enabled = false;
}

bool isEnabled() {
    return s_enabled && (millis() >= s_cooldown_until);
}

void setEnabled(bool enabled) {
    s_enabled = enabled;
}

void setCooldown(uint32_t ms) {
    s_cooldown_until = millis() + ms;
}

bool feedAudioFrame(const int16_t* samples, size_t sample_count) {
    if (!isEnabled() || samples == nullptr || sample_count == 0) return false;

    for (size_t i = 0; i < sample_count; i++) {
        s_frame_sample_buf[s_frame_sample_count++] = samples[i];

        // 帧长 256 点，帧移 256 点 (21.3ms/帧)
        if (s_frame_sample_count >= FFT_N) {
            float mfcc[NUM_MFCC_COEFFS];
            float log_mel[NUM_MEL_BANDS];
            float rms = 0.0f;
            extractMfccFrame(s_frame_sample_buf, mfcc, log_mel, &rms);

            memcpy(s_mfcc_history[s_history_write_idx], mfcc, sizeof(mfcc));
            memcpy(s_log_mel_history[s_history_write_idx], log_mel, sizeof(log_mel));
            s_frame_rms_history[s_history_write_idx] = rms;

            s_history_write_idx = (s_history_write_idx + 1) % MAX_HISTORY_FRAMES;
            if (s_history_count < MAX_HISTORY_FRAMES) s_history_count++;

            s_frame_sample_count = 0;

            // 历史帧在自然发音窗口 (0.6s ~ 1.35s: 28 ~ 64 帧) 内进行多重门限严审
            if (s_history_count >= 28) {
                float max_hist_rms = 0.0f;
                int voiced_frame_count = 0;
                for (int h = 0; h < s_history_count; h++) {
                    if (s_frame_rms_history[h] > max_hist_rms) {
                        max_hist_rms = s_frame_rms_history[h];
                    }
                    if (s_frame_rms_history[h] > 700.0f) {
                        voiced_frame_count++;
                    }
                }

                // 门控 1: 必须有真实开口发音能量 (RMS > 850.0) 且发音连续性 >= 4 帧 (>80ms)
                if (max_hist_rms > 850.0f && voiced_frame_count >= 4) {
                    float best_score = 0.0f;
                    int best_w = 0;

                    // 🌟 细粒度以 2 帧 (42ms) 为步长进行多尺度滑动扫描 (0.6s ~ 1.35s: 28 ~ 64 帧)
                    for (int w = 28; w <= 64 && w <= s_history_count; w += 2) {
                        float s_val = matchKeywordTemplate(CustomWakeModel::CUSTOM_WAKE_TEMPLATE, w);
                        if (s_val > best_score) {
                            best_score = s_val;
                            best_w = w;
                        }
                    }

                    bool is_playing = MusicPlayerService::isPlaying();
                    float score_gate = is_playing ? 56.0f : 65.0f; // 播放音乐时自适应定界 56%，静止时 65%

                    // 实时声学诊断日志 (当匹配度 >= 45% 时打印，方便直观观察匹配度)
                    if (best_score >= 45.0f) {
                        Serial.printf("🔍 [SR-DIAG] 匹配度: %.1f%% (判定门限: >= %.1f%%, 音乐AEC: %s) | 峰值音量: %.0f | 窗口: %d 帧 (%.2fs)\r\n",
                                      best_score, score_gate, is_playing ? "ON" : "OFF", max_hist_rms, best_w, (float)best_w * 0.0213f);
                    }

                    // 门控 2: 黄金声学匹配门限
                    if (best_score >= score_gate) {
                        Serial.printf("✨ [LOCAL-SR] 🎯 本地高精度声学引擎精准捕获专属唤醒词: 【你好小乐】！(匹配度: %.1f%%, 判定门限: %.1f%%, 窗口: %d 帧)\r\n", 
                                      best_score, score_gate, best_w);
                        
                        s_history_count = 0;
                        s_frame_sample_count = 0;
                        setCooldown(3500);
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

} // namespace VoiceTriggerService
