import glob
import wave
import numpy as np
import os
from sklearn.linear_model import LogisticRegression, SGDClassifier
from sklearn.svm import LinearSVC
from sklearn.model_selection import StratifiedKFold
from sklearn.metrics import classification_report, roc_auc_score

# 1. Feature Extractor (Exact match with ESP32-S3 C++ engine)
def extract_mfcc_frames(pcm, sr=12000):
    n_frames = len(pcm) // 256
    if n_frames < 8:
        return None, None
        
    mel_min = 1127.0 * np.log(1.0 + 100.0 / 700.0)
    mel_max = 1127.0 * np.log(1.0 + 5500.0 / 700.0)
    mel_points = np.linspace(mel_min, mel_max, 18)
    hz_points = 700.0 * (np.exp(mel_points / 1127.0) - 1.0)
    bin_points = np.floor((256 + 1) * hz_points / sr).astype(int)
    
    mel_fb = np.zeros((16, 128))
    for m in range(1, 17):
        f_m_minus = bin_points[m-1]
        f_m = bin_points[m]
        f_m_plus = bin_points[m+1]
        for k in range(f_m_minus, f_m):
            if k < 128 and f_m > f_m_minus:
                mel_fb[m-1, k] = (k - f_m_minus) / (f_m - f_m_minus)
        for k in range(f_m, f_m_plus):
            if k < 128 and f_m_plus > f_m:
                mel_fb[m-1, k] = (f_m_plus - k) / (f_m_plus - f_m)
                
    dct_mat = np.zeros((12, 16))
    for i in range(12):
        for j in range(16):
            dct_mat[i, j] = np.cos(np.pi * (i + 1) * (j + 0.5) / 16.0)
            
    hamm = 0.54 - 0.46 * np.cos(2.0 * np.pi * np.arange(256) / 255.0)
    prev = 0.0
    
    mfccs, rms_list = [], []
    for f in range(n_frames):
        chunk = pcm[f*256 : (f+1)*256].astype(float)
        rms = np.sqrt(np.mean(chunk**2))
        rms_list.append(rms)
        
        emphasized = np.zeros(256)
        for i in range(256):
            emphasized[i] = chunk[i] - 0.95 * prev
            prev = chunk[i]
        win = emphasized * hamm
        fft_res = np.fft.rfft(win, 256)[:128]
        pow_spec = (np.abs(fft_res)**2) / 256.0
        log_mel = np.log(np.dot(mel_fb, pow_spec) + 1.0)
        mfcc = np.dot(dct_mat, log_mel)
        norm = np.linalg.norm(mfcc)
        if norm > 1e-4:
            mfcc = mfcc / norm
        mfccs.append(mfcc)
        
    return np.array(mfccs), np.array(rms_list)

def extract_8state_vector(mfccs_sub):
    seg_sz = len(mfccs_sub) / 8.0
    states = []
    for s in range(8):
        s_start = int(s * seg_sz)
        s_end = int((s + 1) * seg_sz)
        if s_end <= s_start: s_end = s_start + 1
        avg_m = np.mean(mfccs_sub[s_start:s_end], axis=0)
        norm = np.linalg.norm(avg_m)
        if norm > 1e-4: avg_m /= norm
        states.append(avg_m)
    return np.array(states).flatten() # 8 x 12 = 96 dimensional feature vector

print("[INFO] Loading dataset...")

# 2. Extract Positive Samples (100 wake words)
wake_files = glob.glob("e:/学习/毕业论文/代码/gemini/backup_20260812_2337/tools/voice_dataset/wake_word/*.wav")
X_pos = []

for f in wake_files:
    pcm = np.frombuffer(wave.open(f).readframes(wave.open(f).getnframes()), dtype=np.int16)
    if wave.open(f).getframerate() == 24000:
        pcm = pcm[::2]
    pcm = np.clip(pcm.astype(float) * 5.0, -32700, 32700).astype(np.int16)
    mfccs, rms_list = extract_mfcc_frames(pcm)
    if mfccs is None or len(mfccs) < 16:
        continue
    # Active range based on energy
    active = [i for i, r in enumerate(rms_list) if r > 800.0]
    if len(active) >= 12:
        # Standard positive slice
        vec = extract_8state_vector(mfccs[active[0] : active[-1] + 1])
        X_pos.append(vec)
        # Data augmentation with small boundary shifts
        if active[0] > 0:
            X_pos.append(extract_8state_vector(mfccs[active[0]-1 : active[-1]]))
        if active[-1] + 1 < len(mfccs):
            X_pos.append(extract_8state_vector(mfccs[active[0] : active[-1]+1]))

X_pos = np.array(X_pos)
print(f"[DATA] Positive wake samples: {X_pos.shape[0]} (96-dim)")

# 3. Extract Negative Samples (Noise + Non-wake speech slices + Synthesized conversational audio)
noise_files = glob.glob("e:/学习/毕业论文/代码/gemini/backup_20260812_2337/tools/voice_dataset/noise/*.wav")
X_neg = []

for f in noise_files:
    pcm = np.frombuffer(wave.open(f).readframes(wave.open(f).getnframes()), dtype=np.int16)
    if wave.open(f).getframerate() == 24000:
        pcm = pcm[::2]
    pcm = np.clip(pcm.astype(float) * 5.0, -32700, 32700).astype(np.int16)
    mfccs, rms_list = extract_mfcc_frames(pcm)
    if mfccs is None or len(mfccs) < 24:
        continue
    # Sliding window extraction of negative slices
    for w in [24, 32, 40, 48]:
        for st in range(0, len(mfccs) - w, 4):
            vec = extract_8state_vector(mfccs[st : st + w])
            X_neg.append(vec)

# Also extract non-target sub-windows from positive files (prefix and suffix silence / partial syllables)
for f in wake_files[:30]:
    pcm = np.frombuffer(wave.open(f).readframes(wave.open(f).getnframes()), dtype=np.int16)
    if wave.open(f).getframerate() == 24000:
        pcm = pcm[::2]
    pcm = np.clip(pcm.astype(float) * 5.0, -32700, 32700).astype(np.int16)
    mfccs, rms_list = extract_mfcc_frames(pcm)
    if mfccs is not None and len(mfccs) > 30:
        # First half slice (only "Ni-Hao", missing "Xiao-Le") -> Negative
        X_neg.append(extract_8state_vector(mfccs[: len(mfccs)//2]))
        # Second half slice (only "Xiao-Le", missing "Ni-Hao") -> Negative
        X_neg.append(extract_8state_vector(mfccs[len(mfccs)//2 :]))

X_neg = np.array(X_neg)
print(f"[DATA] Negative noise/speech samples: {X_neg.shape[0]} (96-dim)")

# 4. Train Discriminant Linear SVM Classifier
X = np.vstack([X_pos, X_neg])
y = np.hstack([np.ones(len(X_pos)), np.zeros(len(X_neg))])

print("\n[TRAIN] Running 5-Fold Stratified Cross Validation...")
skf = StratifiedKFold(n_splits=5, shuffle=True, random_state=42)
scores = []
for fold, (train_idx, val_idx) in enumerate(skf.split(X, y)):
    clf = LinearSVC(C=1.0, class_weight='balanced', random_state=42, max_iter=20000)
    clf.fit(X[train_idx], y[train_idx])
    preds = clf.predict(X[val_idx])
    acc = np.mean(preds == y[val_idx])
    scores.append(acc)
    print(f"  [Fold {fold+1}] Validation Accuracy: {acc*100:.2f}%")

print(f"[RESULT] 5-Fold Mean Accuracy: {np.mean(scores)*100:.2f}%\n")

# 5. Fit Full Model
final_clf = LinearSVC(C=1.5, class_weight='balanced', random_state=42, max_iter=30000)
final_clf.fit(X, y)

weights = final_clf.coef_[0].reshape(8, 12) # (8, 12)
bias = float(final_clf.intercept_[0])

print("=== Training Completed ===")
print(f"Bias: {bias:.4f}")
print(f"Weights L2 Norm: {np.linalg.norm(weights):.4f}")

# Calculate in-sample score distributions
pos_scores = [np.dot(v, final_clf.coef_[0]) + bias for v in X_pos]
neg_scores = [np.dot(v, final_clf.coef_[0]) + bias for v in X_neg]

print(f"Positive SVM score: min={min(pos_scores):.2f}, max={max(pos_scores):.2f}, mean={np.mean(pos_scores):.2f}")
print(f"Negative SVM score: min={min(neg_scores):.2f}, max={max(neg_scores):.2f}, mean={np.mean(neg_scores):.2f}")

# Generate C++ Header file
cpp_header = f"""// ===================================================================
// 🧠 ESP32-S3 专属离线唤醒词高精度判别式机器学习模型 (Linear SVM KWS)
// 训练样本: {len(X_pos)} 组正样本, {len(X_neg)} 组负样本 (96 维声学空间超平面)
// 5折交叉验证准确率: {np.mean(scores)*100:.2f}%
// ===================================================================
#pragma once
#include <stdint.h>

namespace CustomWakeModel {{

static constexpr int NUM_STATES = 8;
static constexpr int NUM_COEFFS = 12;

// 🌟 96 维线性判别超平面权重矩阵 (W[8][12])
static const float SVM_WEIGHT_MATRIX[NUM_STATES][NUM_COEFFS] = {{
"""

for s in range(8):
    row_str = ", ".join([f"{weights[s, c]:.4f}f" for c in range(12)])
    cpp_header += f"    {{{row_str}}}, // 状态 {s}\n"

cpp_header += f"""}};

// 🌟 最优决策分类偏置与判定阈值
static constexpr float SVM_BIAS = {bias:.4f}f;
static constexpr float SVM_DECISION_THRESHOLD = 0.0f; // Score > 0.0 即判定为「你好小乐」

}} // namespace CustomWakeModel
"""

header_path = "e:/学习/毕业论文/代码/gemini/backup_20260812_2337/Master_Gateway_S3/include/custom_wake_model.h"
with open(header_path, "w", encoding="utf-8") as f:
    f.write(cpp_header)

print(f"\n[DONE] Model successfully exported to: {header_path}")
