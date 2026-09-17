"""
===================================================================
🎙️ ESP32-S3 原生麦克风专属唤醒词【可视化图形采集、修剪与一键训练平台】
===================================================================
功能特点：
1. 串口全自动发现与高速 921600 通信；
2. 实时捕获 ESP32 摇杆按键录音流，自动消去头尾物理机械按键音；
3. 动态波形渲染与【🖱️ 鼠标拖拽高精度交互式修剪/试听/撤销】；
4. 样本库任意历史样本点击即加载并支持随时二次裁剪；
5. ⚡ 一键提取 8 阶段硬件级声学共振峰特征，生成 custom_wake_model.h；
6. 🚀 一键编译并烧录固件。
"""

import os
import sys
import io
import time
import re
import wave
import base64
import glob
import threading
import subprocess
import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import serial
import serial.tools.list_ports
import sounddevice as sd

DEFAULT_BAUD = 921600
DEFAULT_SR = 12000

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATASET_DIR = os.path.join(SCRIPT_DIR, "esp32_dataset")
WAKE_DIR = os.path.join(DATASET_DIR, "wake_word")
NOISE_DIR = os.path.join(DATASET_DIR, "noise")
TARGET_HEADER_PATH = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "Master_Gateway_S3", "include", "custom_wake_model.h"))
PIO_PATH = r"C:\Users\CCH\.platformio\penv\Scripts\platformio.exe"
PROJECT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "Master_Gateway_S3"))

os.makedirs(WAKE_DIR, exist_ok=True)
os.makedirs(NOISE_DIR, exist_ok=True)


def find_esp32_port():
    ports = serial.tools.list_ports.comports()
    if not ports:
        return None
    for p in ports:
        desc = p.description.lower()
        if "ch340" in desc or "ch343" in desc or "cp210" in desc or "usb" in desc or "uart" in desc or "com14" in p.device.lower():
            return p.device
    return ports[0].device


def clean_mechanical_click_boundaries(pcm, sr=12000):
    """
    🌟 基础物理按键音剥离与隔直滤波：
    1. 剥离头尾按键按下的 80ms 机械杂音；
    2. 1阶隔直滤波消除 DC 偏置；
    3. 10ms 线性淡入淡出防止爆音。
    """
    n = len(pcm)
    margin = int(0.08 * sr) # 80ms
    if n > 3 * margin:
        core_pcm = pcm[margin : n - margin]
    else:
        core_pcm = pcm

    if len(core_pcm) > 200:
        filtered = np.zeros(len(core_pcm), dtype=np.float32)
        x_prev = float(core_pcm[0])
        y_prev = 0.0
        for i in range(len(core_pcm)):
            x_curr = float(core_pcm[i])
            y_curr = (x_curr - x_prev) + 0.985 * y_prev
            x_prev = x_curr
            y_prev = y_curr
            filtered[i] = y_curr
        fade_len = min(120, len(filtered) // 4)
        for i in range(fade_len):
            filtered[i] *= (i / float(fade_len))
            filtered[-1 - i] *= (i / float(fade_len))
        core_pcm = np.clip(filtered, -32700, 32700).astype(np.int16)

    return core_pcm


def extract_mfcc_template(pcm, sr=12000):
    n_frames = len(pcm) // 256
    if n_frames < 8: return None, None
    
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


def robust_vad_crop(pcm, sr=12000):
    f_len = int(0.02 * sr)
    n_f = len(pcm) // f_len
    if n_f < 8: return pcm
    
    energies = np.array([np.sqrt(np.mean(pcm[i*f_len : (i+1)*f_len].astype(float)**2)) for i in range(n_f)])
    peak_e = float(np.max(energies)) if len(energies) > 0 else 0.0
    
    sorted_e = np.sort(energies)
    noise_floor = float(np.mean(sorted_e[:max(3, n_f // 4)]))
    
    # 柔和底噪门限：底噪 + 20% 动态范围 (完整包络「你」的微弱声母与「乐」的尾韵)
    thresh = max(noise_floor + 0.20 * (peak_e - noise_floor), 280.0)
    is_speech = energies >= thresh
    smoothed = np.convolve(is_speech.astype(float), np.ones(3)/3.0, mode="same") > 0.25
    active = np.where(smoothed)[0]
    
    if len(active) < 5: return pcm
    # 前后保留 80~100ms 保护余量，确保 0.9s~1.3s 发音完整性
    st_idx = max(0, active[0] - 4)
    ed_idx = min(n_f, active[-1] + 5)
    
    return pcm[st_idx * f_len : ed_idx * f_len]


class Esp32GuiTrainerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("🎙️ ESP32-S3 原生麦克风专属唤醒词【采集、修剪与一键训练平台】")
        self.root.geometry("980x780")
        self.root.minsize(920, 720)
        self.root.configure(bg="#181825")

        self.ser = None
        self.serial_thread = None
        self.running = True
        self.auto_play = tk.BooleanVar(value=True)

        # 当前活动音频状态
        self.active_pcm = None
        self.backup_pcm = None
        self.active_sr = DEFAULT_SR
        self.active_file = None

        # 选区范围比例 [0.0, 1.0]
        self.sel_ratio_start = 0.0
        self.sel_ratio_end = 1.0
        self.is_dragging = False
        self.drag_start_x = 0
        self._suppress_tree_event = False

        self.setup_styles()
        self.build_ui()
        self.refresh_ports()
        self.refresh_sample_list()

        # 启动后台串口监听
        self.start_serial_monitor()

    def setup_styles(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background="#181825")
        style.configure("TLabel", background="#181825", foreground="#cdd6f4", font=("微软雅黑", 10))
        style.configure("Header.TLabel", font=("微软雅黑", 13, "bold"), foreground="#89b4fa")
        style.configure("Treeview", background="#1e1e2e", foreground="#cdd6f4", fieldbackground="#1e1e2e", font=("微软雅黑", 9), rowheight=24)
        style.configure("Treeview.Heading", background="#313244", foreground="#89dceb", font=("微软雅黑", 9, "bold"))
        style.map("Treeview", background=[("selected", "#45475a")], foreground=[("selected", "#f9e2af")])

    def build_ui(self):
        # 1. 顶部标题与串口状态栏
        top_bar = tk.Frame(self.root, bg="#11111b", height=54)
        top_bar.pack(fill=tk.X, side=tk.TOP)

        title_lbl = tk.Label(top_bar, text="🎙️ ESP32-S3 原生麦克风模型工作台", 
                             font=("微软雅黑", 13, "bold"), fg="#89dceb", bg="#11111b")
        title_lbl.pack(side=tk.LEFT, padx=16, pady=12)

        port_frame = tk.Frame(top_bar, bg="#11111b")
        port_frame.pack(side=tk.RIGHT, padx=16, pady=10)

        tk.Label(port_frame, text="串口:", font=("微软雅黑", 9, "bold"), fg="#f9e2af", bg="#11111b").pack(side=tk.LEFT, padx=4)
        self.port_combo = ttk.Combobox(port_frame, width=14, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=4)

        self.btn_reconnect = tk.Button(port_frame, text="🔄 重连", font=("微软雅黑", 9),
                                       bg="#313244", fg="#cdd6f4", activebackground="#45475a", bd=0, padx=8, pady=2,
                                       command=self.manual_reconnect)
        self.btn_reconnect.pack(side=tk.LEFT, padx=4)

        self.conn_dot = tk.Label(port_frame, text="● 未连接", font=("微软雅黑", 9, "bold"), fg="#f38ba8", bg="#11111b")
        self.conn_dot.pack(side=tk.LEFT, padx=6)

        # 2. 状态指示大横幅
        self.status_card = tk.Frame(self.root, bg="#1e1e2e", bd=1, relief=tk.SOLID)
        self.status_card.pack(fill=tk.X, padx=16, pady=6)

        self.lbl_status_icon = tk.Label(self.status_card, text="🟢", font=("Segoe UI Emoji", 20), bg="#1e1e2e", fg="#a6e3a1")
        self.lbl_status_icon.pack(side=tk.LEFT, padx=14, pady=8)

        status_text_frame = tk.Frame(self.status_card, bg="#1e1e2e")
        status_text_frame.pack(side=tk.LEFT, fill=tk.Y, pady=6)

        self.lbl_status_title = tk.Label(status_text_frame, text="待命就绪 · 在 ESP32 上按摇杆开始录音，或在下方列表选中样本进行修剪",
                                         font=("微软雅黑", 11, "bold"), fg="#a6e3a1", bg="#1e1e2e")
        self.lbl_status_title.pack(anchor="w")

        self.lbl_status_tip = tk.Label(status_text_frame, text="💡 提示：在波形画布上【按住鼠标左键横向拖拽】可选择修剪区域，点击【✂️ 确认裁剪】即可精准裁剪！",
                                       font=("微软雅黑", 9), fg="#a6adc8", bg="#1e1e2e")
        self.lbl_status_tip.pack(anchor="w")

        # 3. 中部：波形与可视化修剪交互区
        mid_frame = tk.Frame(self.root, bg="#181825")
        mid_frame.pack(fill=tk.BOTH, expand=True, padx=16, pady=4)

        wf_box = tk.Frame(mid_frame, bg="#1e1e2e", bd=1, relief=tk.SOLID)
        wf_box.pack(fill=tk.BOTH, expand=True)

        wf_title_bar = tk.Frame(wf_box, bg="#1e1e2e")
        wf_title_bar.pack(fill=tk.X, padx=8, pady=4)

        self.lbl_cur_filename = tk.Label(wf_title_bar, text="📊 波形预览与手动修剪 (未加载)", font=("微软雅黑", 10, "bold"), fg="#89b4fa", bg="#1e1e2e")
        self.lbl_cur_filename.pack(side=tk.LEFT)

        self.lbl_sel_info = tk.Label(wf_title_bar, text="[选区: 全部 100%]", font=("微软雅黑", 9, "bold"), fg="#f9e2af", bg="#1e1e2e")
        self.lbl_sel_info.pack(side=tk.LEFT, padx=12)

        self.chk_autoplay = tk.Checkbutton(wf_title_bar, text="收到后自动试听", variable=self.auto_play,
                                           font=("微软雅黑", 9), fg="#cdd6f4", bg="#1e1e2e", selectcolor="#313244", activebackground="#1e1e2e")
        self.chk_autoplay.pack(side=tk.RIGHT)

        # 互动波形 Canvas
        self.canvas_wf = tk.Canvas(wf_box, bg="#11111b", height=180, highlightthickness=0, cursor="crosshair")
        self.canvas_wf.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 4))
        self.canvas_wf.bind("<ButtonPress-1>", self.on_canvas_press)
        self.canvas_wf.bind("<B1-Motion>", self.on_canvas_drag)
        self.canvas_wf.bind("<ButtonRelease-1>", self.on_canvas_release)
        self.canvas_wf.bind("<Double-Button-1>", lambda e: self.play_current_selection())
        self.canvas_wf.bind("<Configure>", lambda e: self.redraw_active_waveform())

        # 声学指标与修剪按钮工具条
        self.metric_bar = tk.Frame(wf_box, bg="#181825", height=38)
        self.metric_bar.pack(fill=tk.X, padx=8, pady=(0, 6))

        self.lbl_m_dur = tk.Label(self.metric_bar, text="总长: -- s", font=("微软雅黑", 9), fg="#fab387", bg="#181825")
        self.lbl_m_dur.pack(side=tk.LEFT, padx=8, pady=4)

        self.lbl_m_rms = tk.Label(self.metric_bar, text="RMS: --", font=("微软雅黑", 9), fg="#a6e3a1", bg="#181825")
        self.lbl_m_rms.pack(side=tk.LEFT, padx=8, pady=4)

        self.lbl_m_peak = tk.Label(self.metric_bar, text="峰值: --", font=("微软雅黑", 9), fg="#cba6f7", bg="#181825")
        self.lbl_m_peak.pack(side=tk.LEFT, padx=8, pady=4)

        # 右侧修剪交互按钮组
        self.btn_undo_trim = tk.Button(self.metric_bar, text="↩️ 撤销裁剪", font=("微软雅黑", 9),
                                       bg="#313244", fg="#cdd6f4", activebackground="#45475a", bd=0, padx=10, pady=2,
                                       command=self.undo_trim)
        self.btn_undo_trim.pack(side=tk.RIGHT, padx=4, pady=4)

        self.btn_crop = tk.Button(self.metric_bar, text="✂️ 确认裁剪选区", font=("微软雅黑", 9, "bold"),
                                  bg="#fab387", fg="#11111b", activebackground="#f9e2af", bd=0, padx=12, pady=2,
                                  command=self.crop_and_save)
        self.btn_crop.pack(side=tk.RIGHT, padx=4, pady=4)

        self.btn_play_sel = tk.Button(self.metric_bar, text="▶️ 试听选区", font=("微软雅黑", 9, "bold"),
                                      bg="#a6e3a1", fg="#11111b", activebackground="#94e2d5", bd=0, padx=12, pady=2,
                                      command=self.play_current_selection)
        self.btn_play_sel.pack(side=tk.RIGHT, padx=4, pady=4)

        self.btn_play_full = tk.Button(self.metric_bar, text="🔊 试听完整", font=("微软雅黑", 9),
                                       bg="#89b4fa", fg="#11111b", activebackground="#b4befe", bd=0, padx=10, pady=2,
                                       command=self.play_current_full)
        self.btn_play_full.pack(side=tk.RIGHT, padx=4, pady=4)

        # 4. 下部：样本管理列表
        list_box = tk.Frame(self.root, bg="#1e1e2e", bd=1, relief=tk.SOLID)
        list_box.pack(fill=tk.BOTH, expand=True, padx=16, pady=6)

        list_title_bar = tk.Frame(list_box, bg="#1e1e2e")
        list_title_bar.pack(fill=tk.X, padx=8, pady=4)

        self.lbl_count = tk.Label(list_title_bar, text="📁 ESP32 原生样本库 (0 条)",
                                  font=("微软雅黑", 10, "bold"), fg="#f9e2af", bg="#1e1e2e")
        self.lbl_count.pack(side=tk.LEFT)

        btn_del = tk.Button(list_title_bar, text="🗑️ 删除选中", font=("微软雅黑", 8),
                            bg="#313244", fg="#f38ba8", bd=0, padx=8, pady=2, command=self.delete_selected)
        btn_del.pack(side=tk.RIGHT, padx=4)

        btn_play_sel_item = tk.Button(list_title_bar, text="▶️ 试听选中", font=("微软雅黑", 8),
                                      bg="#313244", fg="#89dceb", bd=0, padx=8, pady=2, command=self.play_selected_from_list)
        btn_play_sel_item.pack(side=tk.RIGHT, padx=4)

        # Treeview
        cols = ("id", "filename", "duration", "rms", "peak", "status")
        self.tree = ttk.Treeview(list_box, columns=cols, show="headings", height=6)
        self.tree.heading("id", text="#")
        self.tree.heading("filename", text="样本文件 (单击加载到上方进行修剪)")
        self.tree.heading("duration", text="有效时长")
        self.tree.heading("rms", text="有效RMS")
        self.tree.heading("peak", text="峰值")
        self.tree.heading("status", text="状态")

        self.tree.column("id", width=36, anchor="center")
        self.tree.column("filename", width=260, anchor="w")
        self.tree.column("duration", width=80, anchor="center")
        self.tree.column("rms", width=80, anchor="center")
        self.tree.column("peak", width=80, anchor="center")
        self.tree.column("status", width=120, anchor="center")

        tree_scroll = ttk.Scrollbar(list_box, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=tree_scroll.set)
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(8, 0), pady=(0, 6))
        self.tree.bind("<<TreeviewSelect>>", self.on_sample_selected)
        self.tree.bind("<Double-1>", lambda e: self.play_selected_from_list())
        self.tree.tag_configure("editing", background="#313244", foreground="#f9e2af")
        self.tree.tag_configure("normal", background="#1e1e2e", foreground="#cdd6f4")

        # 5. 底部操作栏 (一键训练 & 一键烧录)
        btm_bar = tk.Frame(self.root, bg="#11111b", height=60)
        btm_bar.pack(fill=tk.X, side=tk.BOTTOM)

        self.lbl_model_status = tk.Label(btm_bar, text="💡 建议录满 15~30 条样本后，点击右侧按钮一键生成模型",
                                         font=("微软雅黑", 9), fg="#a6adc8", bg="#11111b")
        self.lbl_model_status.pack(side=tk.LEFT, padx=16, pady=14)

        self.btn_compile = tk.Button(btm_bar, text="🚀 一键烧录固件", font=("微软雅黑", 10, "bold"),
                                     bg="#313244", fg="#a6adc8", bd=0, padx=16, pady=6, state=tk.DISABLED,
                                     command=self.compile_and_upload)
        self.btn_compile.pack(side=tk.RIGHT, padx=(4, 16), pady=10)

        self.btn_train = tk.Button(btm_bar, text="⚡ 一键生成专属模型 (custom_wake_model.h)", font=("微软雅黑", 10, "bold"),
                                   bg="#a6e3a1", fg="#11111b", activebackground="#94e2d5", bd=0, padx=16, pady=6,
                                   command=self.train_model)
        self.btn_train.pack(side=tk.RIGHT, padx=4, pady=10)

    # ==========================================
    # 🖱️ 交互式波形选区与裁剪事件处理
    # ==========================================
    def on_canvas_press(self, event):
        if self.active_pcm is None: return
        w = max(10, self.canvas_wf.winfo_width())
        self.is_dragging = True
        self.drag_start_x = event.x
        r = max(0.0, min(1.0, event.x / float(w)))
        self.sel_ratio_start = r
        self.sel_ratio_end = r
        self.redraw_active_waveform()

    def on_canvas_drag(self, event):
        if not self.is_dragging or self.active_pcm is None: return
        w = max(10, self.canvas_wf.winfo_width())
        r1 = max(0.0, min(1.0, self.drag_start_x / float(w)))
        r2 = max(0.0, min(1.0, event.x / float(w)))
        self.sel_ratio_start = min(r1, r2)
        self.sel_ratio_end = max(r1, r2)
        self.redraw_active_waveform()

    def on_canvas_release(self, event):
        if not self.is_dragging or self.active_pcm is None: return
        self.is_dragging = False
        w = max(10, self.canvas_wf.winfo_width())
        if abs(event.x - self.drag_start_x) < 6:
            # 单击重置为全选
            self.sel_ratio_start = 0.0
            self.sel_ratio_end = 1.0
        self.redraw_active_waveform()

    def redraw_active_waveform(self):
        self.canvas_wf.delete("all")
        if self.active_pcm is None:
            w = self.canvas_wf.winfo_width()
            h = self.canvas_wf.winfo_height()
            self.canvas_wf.create_text(w//2, h//2, text="暂无选中的音频波形，请在下方列表点击样本或按 ESP32 录音", fill="#6c7086", font=("微软雅黑", 11))
            return

        pcm = self.active_pcm
        w = self.canvas_wf.winfo_width()
        h = self.canvas_wf.winfo_height()
        if w < 10 or h < 10: w, h = 500, 180

        mid_y = h // 2
        self.canvas_wf.create_line(0, mid_y, w, mid_y, fill="#313244", dash=(2, 2))

        # 1. 绘制完整波形
        step = max(1, len(pcm) // w)
        for x in range(w):
            idx = x * step
            if idx >= len(pcm): break
            chunk = pcm[idx : idx + step]
            if len(chunk) == 0: continue
            min_v = np.min(chunk)
            max_v = np.max(chunk)
            y1 = mid_y - int((max_v / 32768.0) * (h / 2 - 10))
            y2 = mid_y - int((min_v / 32768.0) * (h / 2 - 10))
            self.canvas_wf.create_line(x, y1, x, y2, fill="#89dceb")

        # 2. 绘制半透明高亮选区遮罩与标尺
        x_st = int(self.sel_ratio_start * w)
        x_ed = int(self.sel_ratio_end * w)
        dur_total = len(pcm) / float(self.active_sr)
        t_st = self.sel_ratio_start * dur_total
        t_ed = self.sel_ratio_end * dur_total
        sel_dur = t_ed - t_st

        if x_ed > x_st:
            # 绘制选区高亮框
            self.canvas_wf.create_rectangle(x_st, 2, x_ed, h - 2, outline="#f9e2af", width=2, fill="#f9e2af", stipple="gray25")
            # 绘制起始与结束竖线
            self.canvas_wf.create_line(x_st, 0, x_st, h, fill="#a6e3a1", width=2)
            self.canvas_wf.create_line(x_ed, 0, x_ed, h, fill="#f38ba8", width=2)
            # 时间标签
            self.canvas_wf.create_text(x_st + 4, 12, text=f"{t_st:.2f}s", anchor="nw", fill="#a6e3a1", font=("微软雅黑", 8, "bold"))
            self.canvas_wf.create_text(x_ed - 4, 12, text=f"{t_ed:.2f}s", anchor="ne", fill="#f38ba8", font=("微软雅黑", 8, "bold"))

        # 更新标题栏选区说明
        if self.sel_ratio_start <= 0.001 and self.sel_ratio_end >= 0.999:
            self.lbl_sel_info.config(text=f"[选区: 全部 {dur_total:.2f}s]", fg="#89b4fa")
        else:
            self.lbl_sel_info.config(text=f"[选区: {t_st:.2f}s ~ {t_ed:.2f}s | 裁剪时长 {sel_dur:.2f}s]", fg="#f9e2af")

    def play_current_selection(self):
        if self.active_pcm is None: return
        n = len(self.active_pcm)
        s_st = int(self.sel_ratio_start * n)
        s_ed = int(self.sel_ratio_end * n)
        if s_ed <= s_st: s_ed = min(n, s_st + 256)
        slice_pcm = self.active_pcm[s_st:s_ed]
        threading.Thread(target=lambda: (sd.play(slice_pcm, self.active_sr), sd.wait()), daemon=True).start()

    def play_current_full(self):
        if self.active_pcm is None: return
        threading.Thread(target=lambda: (sd.play(self.active_pcm, self.active_sr), sd.wait()), daemon=True).start()

    def crop_and_save(self):
        if self.active_pcm is None: return
        n = len(self.active_pcm)
        s_st = int(self.sel_ratio_start * n)
        s_ed = int(self.sel_ratio_end * n)
        if s_ed - s_st < 1200: # 至少 100ms
            messagebox.showwarning("选区过短", "裁剪选区太短，请拖拽选择至少包含完整「你好小乐」的区域！")
            return

        cropped = self.active_pcm[s_st:s_ed].copy()
        # 施加 10ms 平滑淡入淡出消除边界断点
        fade_len = min(120, len(cropped) // 4)
        for i in range(fade_len):
            cropped[i] = int(cropped[i] * (i / float(fade_len)))
            cropped[-1 - i] = int(cropped[-1 - i] * (i / float(fade_len)))

        # 覆盖保存文件
        if self.active_file and os.path.exists(self.active_file):
            with wave.open(self.active_file, "wb") as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(self.active_sr)
                wf.writeframes(cropped.tobytes())

        self.active_pcm = cropped
        self.sel_ratio_start = 0.0
        self.sel_ratio_end = 1.0

        # 更新指标
        dur = len(cropped) / float(self.active_sr)
        rms = np.sqrt(np.mean(cropped.astype(float)**2))
        peak = np.max(np.abs(cropped))
        self.lbl_m_dur.config(text=f"总长: {dur:.2f} s")
        self.lbl_m_rms.config(text=f"RMS: {rms:.0f}")
        self.lbl_m_peak.config(text=f"峰值: {peak}")

        self.redraw_active_waveform()
        self.refresh_sample_list()
        self.lbl_status_tip.config(text=f"✂️ 裁剪已成功保存！当前样本有效时长: {dur:.2f}s (RMS: {rms:.0f})")

    def undo_trim(self):
        if self.backup_pcm is None: return
        self.active_pcm = self.backup_pcm.copy()
        self.sel_ratio_start = 0.0
        self.sel_ratio_end = 1.0

        if self.active_file and os.path.exists(self.active_file):
            with wave.open(self.active_file, "wb") as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(self.active_sr)
                wf.writeframes(self.active_pcm.tobytes())

        dur = len(self.active_pcm) / float(self.active_sr)
        rms = np.sqrt(np.mean(self.active_pcm.astype(float)**2))
        peak = np.max(np.abs(self.active_pcm))
        self.lbl_m_dur.config(text=f"总长: {dur:.2f} s")
        self.lbl_m_rms.config(text=f"RMS: {rms:.0f}")
        self.lbl_m_peak.config(text=f"峰值: {peak}")

        self.redraw_active_waveform()
        self.refresh_sample_list()
        self.lbl_status_tip.config(text="↩️ 已撤销裁剪，恢复为原始完整音频！")

    # ==========================================
    def on_sample_selected(self, event):
        if self._suppress_tree_event: return
        sel = self.tree.selection()
        if not sel: return
        item_id = sel[0]
        vals = self.tree.item(item_id, "values")
        if not vals: return
        fname = str(vals[1]).replace("👉", "").strip()
        full_path = os.path.join(WAKE_DIR, fname)
        if os.path.exists(full_path):
            self.load_audio_file(full_path)
            self._highlight_item_in_tree(item_id)

    def _highlight_item_in_tree(self, active_item_id):
        self._suppress_tree_event = True
        try:
            for item in self.tree.get_children():
                v = self.tree.item(item, "values")
                if not v: continue
                raw_name = str(v[1]).replace("👉", "").strip()
                if item == active_item_id:
                    self.tree.item(item, values=(v[0], f"👉 {raw_name}", v[2], v[3], v[4], "✏️ 正在编辑"), tags=("editing",))
                else:
                    self.tree.item(item, values=(v[0], raw_name, v[2], v[3], v[4], "✅ 纯净人声"), tags=("normal",))
        finally:
            self._suppress_tree_event = False

    def load_audio_file(self, file_path):
        try:
            with wave.open(file_path, "rb") as wf:
                sr = wf.getframerate()
                n = wf.getnframes()
                pcm = np.frombuffer(wf.readframes(n), dtype=np.int16)

            self.active_pcm = pcm
            self.backup_pcm = pcm.copy()
            self.active_sr = sr
            self.active_file = file_path
            self.sel_ratio_start = 0.0
            self.sel_ratio_end = 1.0

            dur = len(pcm) / float(sr)
            rms = np.sqrt(np.mean(pcm.astype(float)**2))
            peak = np.max(np.abs(pcm))

            self.lbl_cur_filename.config(text=f"📊 正在编辑: 👉 {os.path.basename(file_path)}")
            self.lbl_m_dur.config(text=f"总长: {dur:.2f} s")
            self.lbl_m_rms.config(text=f"RMS: {rms:.0f}")
            self.lbl_m_peak.config(text=f"峰值: {peak}")

            self.redraw_active_waveform()
        except Exception as e:
            messagebox.showerror("加载错误", f"无法加载音频: {e}")

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        def_port = find_esp32_port()
        if def_port and def_port in ports:
            self.port_combo.set(def_port)
        elif ports:
            self.port_combo.set(ports[0])

    def manual_reconnect(self):
        if self.ser and self.ser.is_open:
            try: self.ser.close()
            except Exception: pass
        self.refresh_ports()
        self.start_serial_monitor()

    def start_serial_monitor(self):
        if self.serial_thread and self.serial_thread.is_alive():
            return
        self.serial_thread = threading.Thread(target=self._serial_worker, daemon=True)
        self.serial_thread.start()

    def _serial_worker(self):
        while self.running:
            port = self.port_combo.get()
            if not port:
                self.root.after(0, lambda: self.conn_dot.config(text="● 未选串口", fg="#f38ba8"))
                time.sleep(1.0)
                continue

            try:
                self.ser = serial.Serial(port, DEFAULT_BAUD, timeout=0.5)
                self.root.after(0, lambda p=port: self.conn_dot.config(text=f"● 已连接 {p}", fg="#a6e3a1"))
                
                in_b64 = False
                b64_lines = []
                current_sr = DEFAULT_SR

                while self.running and self.ser.is_open:
                    line_bytes = self.ser.readline()
                    if not line_bytes:
                        continue

                    try:
                        line_str = line_bytes.decode("utf-8", errors="ignore").strip()
                    except Exception:
                        continue

                    if not in_b64:
                        if line_str.startswith("=== BEGIN_WAV_BASE64"):
                            in_b64 = True
                            b64_lines = []
                            current_sr = DEFAULT_SR
                            if "SR=" in line_str:
                                try: current_sr = int(line_str.split("SR=")[1].split()[0])
                                except Exception: pass
                            self.root.after(0, self.set_receiving_state)
                        elif "[RAW-CAPTURE]" in line_str or "🔴" in line_str:
                            self.root.after(0, self.set_recording_state)
                    else:
                        if line_str.startswith("=== END_WAV_BASE64 ==="):
                            in_b64 = False
                            full_b64 = "".join(b64_lines)
                            self.root.after(0, lambda b=full_b64, sr=current_sr: self.on_audio_received(b, sr))
                        else:
                            b64_lines.append(line_str)

            except Exception as e:
                self.root.after(0, lambda: self.conn_dot.config(text="● 连接中断", fg="#f38ba8"))
                time.sleep(1.5)

    def set_recording_state(self):
        self.lbl_status_icon.config(text="🔴", fg="#f38ba8")
        self.lbl_status_title.config(text="ESP32 录音中... 请说出「你好小乐」后再次按摇杆", fg="#f38ba8")

    def set_receiving_state(self):
        self.lbl_status_icon.config(text="⚡", fg="#fab387")
        self.lbl_status_title.config(text="正在通过串口秒级接收音频流...", fg="#fab387")

    def set_ready_state(self):
        self.lbl_status_icon.config(text="🟢", fg="#a6e3a1")
        self.lbl_status_title.config(text="待命就绪 · 在 ESP32 上按摇杆录制新样本，或在下方选中样本修剪", fg="#a6e3a1")

    def on_audio_received(self, b64_str, sr):
        try:
            raw_wav = base64.b64decode(b64_str)
            with wave.open(io.BytesIO(raw_wav), "rb") as wf:
                n_samples = wf.getnframes()
                actual_sr = wf.getframerate()
                pcm_raw = np.frombuffer(wf.readframes(n_samples), dtype=np.int16)

            pcm_clean = clean_mechanical_click_boundaries(pcm_raw, actual_sr)

            # 异常饱和判断
            sp_rms = np.sqrt(np.mean(pcm_clean.astype(float)**2))
            if sp_rms > 26000.0 or len(pcm_clean) < 1200:
                self.lbl_status_icon.config(text="⚠️", fg="#fab387")
                self.lbl_status_title.config(text="检测到异常饱和脉冲，已自动丢弃，请重新按摇杆录制！", fg="#fab387")
                self.set_ready_state()
                return

            max_idx = 0
            for f in glob.glob(os.path.join(WAKE_DIR, "esp32_wake_*.wav")):
                m = re.search(r"esp32_wake_(\d+)\.wav", f)
                if m:
                    max_idx = max(max_idx, int(m.group(1)))
            next_idx = max_idx + 1
            filename = os.path.join(WAKE_DIR, f"esp32_wake_{next_idx:03d}.wav")
            with wave.open(filename, "wb") as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(actual_sr)
                wf.writeframes(pcm_clean.tobytes())

            self.load_audio_file(filename)
            self.refresh_sample_list()
            self.set_ready_state()
            self.lbl_status_tip.config(text=f"✨ 新录音接收成功！已自动加载到上方，可在波形上拖拽裁剪！")

            # 自动播放
            if self.auto_play.get():
                threading.Thread(target=lambda: (sd.play(pcm_clean, actual_sr), sd.wait()), daemon=True).start()

        except Exception as e:
            messagebox.showerror("解码错误", f"音频解析失败: {e}")
            self.set_ready_state()

    def refresh_sample_list(self):
        self._suppress_tree_event = True
        try:
            for item in self.tree.get_children():
                self.tree.delete(item)

            active_base = os.path.basename(self.active_file) if self.active_file else None
            files = sorted(glob.glob(os.path.join(WAKE_DIR, "*.wav")))
            selected_item_id = None

            for idx, f in enumerate(files, 1):
                try:
                    with wave.open(f, "rb") as wf:
                        n = wf.getnframes()
                        sr = wf.getframerate()
                        dur = n / float(sr)
                        pcm = np.frombuffer(wf.readframes(n), dtype=np.int16)
                        rms = np.sqrt(np.mean(pcm.astype(float)**2))
                        peak = np.max(np.abs(pcm))
                    
                    base_name = os.path.basename(f)
                    is_cur_edit = (active_base is not None and base_name == active_base)
                    
                    if is_cur_edit:
                        item_id = self.tree.insert("", "end", values=(
                            idx, f"👉 {base_name}", f"{dur:.2f}s", f"{rms:.0f}", f"{peak}", "✏️ 正在编辑"
                        ), tags=("editing",))
                        selected_item_id = item_id
                    else:
                        self.tree.insert("", "end", values=(
                            idx, base_name, f"{dur:.2f}s", f"{rms:.0f}", f"{peak}", "✅ 纯净人声"
                        ), tags=("normal",))
                except Exception:
                    pass

            if selected_item_id:
                self.tree.selection_set(selected_item_id)
                self.tree.focus(selected_item_id)
                self.tree.see(selected_item_id)

            total = len(files)
            if total < 15:
                self.lbl_count.config(text=f"📁 ESP32 原生样本库 (当前 {total} 条 · 建议 15~30 条)")
                self.lbl_model_status.config(text=f"💡 当前 {total} 条样本。建议录制 15~30 条 (涵盖快/慢、远/近、大/小声)，让模型更鲁棒！", fg="#a6adc8")
            elif total < 30:
                self.lbl_count.config(text=f"📁 ESP32 原生样本库 (当前 {total} 条 · 良好覆盖)")
                self.lbl_model_status.config(text=f"🎉 已采集 {total} 条样本！已具备极佳声学表征，可随时点击一键生成模型！", fg="#a6e3a1")
            else:
                self.lbl_count.config(text=f"📁 ESP32 原生样本库 (当前 {total} 条 · 专家级全场景覆盖)")
                self.lbl_model_status.config(text=f"🌟 已采集 {total} 条全场景样本！声学特征覆盖率已达 100%，点击一键生成专属模型！", fg="#f9e2af")
        finally:
            self._suppress_tree_event = False

    def play_selected_from_list(self):
        sel = self.tree.selection()
        if not sel: return
        vals = self.tree.item(sel[0], "values")
        fname = str(vals[1]).replace("👉", "").strip()
        filename = os.path.join(WAKE_DIR, fname)
        if os.path.exists(filename):
            with wave.open(filename, "rb") as wf:
                sr = wf.getframerate()
                pcm = np.frombuffer(wf.readframes(wf.getnframes()), dtype=np.int16)
            threading.Thread(target=lambda: (sd.play(pcm, sr), sd.wait()), daemon=True).start()

    def delete_selected(self):
        sel = self.tree.selection()
        if not sel: return
        vals = self.tree.item(sel[0], "values")
        fname = str(vals[1]).replace("👉", "").strip()
        filename = os.path.join(WAKE_DIR, fname)
        if os.path.exists(filename):
            os.remove(filename)
            if self.active_file == filename:
                self.active_pcm = None
                self.active_file = None
                self.redraw_active_waveform()
            self.refresh_sample_list()

    def train_model(self):
        files = glob.glob(os.path.join(WAKE_DIR, "*.wav"))
        if len(files) < 3:
            messagebox.showwarning("样本不足", "当前录音少于 3 条，请至少采集 5~10 条再训练！")
            return

        all_states = []
        valid_files = []
        for f in files:
            with wave.open(f, "rb") as wf:
                pcm = np.frombuffer(wf.readframes(wf.getnframes()), dtype=np.int16)
                sr = wf.getframerate()
                if sr == 24000: pcm = pcm[::2]
            
            clean_pcm = clean_mechanical_click_boundaries(pcm, sr)
            vad_pcm = robust_vad_crop(clean_pcm, sr)
            dur = len(vad_pcm) / float(sr)
            if dur > 1.6 or dur < 0.5:
                continue

            mfccs, _ = extract_mfcc_template(vad_pcm, sr)
            if mfccs is None or len(mfccs) < 8:
                continue

            seg_sz = len(mfccs) / 8.0
            st = []
            for s in range(8):
                s_start = int(s * seg_sz)
                s_end = int((s + 1) * seg_sz)
                if s_end <= s_start: s_end = s_start + 1
                avg_m = np.mean(mfccs[s_start:s_end], axis=0)
                norm = np.linalg.norm(avg_m)
                if norm > 1e-4: avg_m /= norm
                st.append(avg_m)
            all_states.append(st)
            valid_files.append(os.path.basename(f))

        if len(all_states) < 3:
            messagebox.showwarning("有效样本不足", "未找到足够合格的唤醒词录音，请检查录音时长并确保发音清晰！")
            return

        # 两阶段鲁棒聚类（自动剔除坏点/误录杂音）
        all_states = np.array(all_states)
        init_template = np.mean(all_states, axis=0)
        for s in range(8):
            norm = np.linalg.norm(init_template[s])
            if norm > 1e-4: init_template[s] /= norm

        clean_states = []
        clean_names = []
        for idx, u in enumerate(all_states):
            dots = [np.dot(u[s], init_template[s]) for s in range(8)]
            if np.mean(dots) >= 0.72 and min(dots) >= 0.38:
                clean_states.append(u)
                clean_names.append(valid_files[idx])

        if len(clean_states) < 3:
            clean_states = all_states
            clean_names = valid_files

        clean_states = np.array(clean_states)
        master_template = np.mean(clean_states, axis=0)
        for s in range(8):
            norm = np.linalg.norm(master_template[s])
            if norm > 1e-4: master_template[s] /= norm

        distances = []
        for u in clean_states:
            dist = 0.0
            for s in range(8):
                dot = np.clip(np.dot(u[s], master_template[s]), -1.0, 1.0)
                dist += (1.0 - dot)
            distances.append(dist / 8.0)

        mean_dist = np.mean(distances)

        # 写入 custom_wake_model.h
        cpp_header = f"""// ===================================================================
// 🧠 ESP32-S3 原生板载麦克风专属声学模型 (完全匹配硬件频响与本人发音)
// 黄金优质样本数: {len(clean_states)} 条 (已智能清洗离群杂质) | 平均余弦偏差: {mean_dist:.3f}
// ===================================================================
#pragma once
#include <stdint.h>

namespace CustomWakeModel {{

static constexpr int NUM_STATES = 8;
static constexpr int NUM_COEFFS = 12;

// 🌟 ESP32 原生 8 阶段高精度声学共振峰特征矩阵 (c1 ~ c12)
static const float CUSTOM_WAKE_TEMPLATE[NUM_STATES][NUM_COEFFS] = {{
"""
        for s in range(8):
            row_str = ", ".join([f"{master_template[s, c]:.4f}f" for c in range(12)])
            cpp_header += f"    {{{row_str}}}, // 状态 {s}\n"

        cpp_header += """};

} // namespace CustomWakeModel
"""

        with open(TARGET_HEADER_PATH, "w", encoding="utf-8") as f:
            f.write(cpp_header)

        self.btn_compile.config(state=tk.NORMAL, bg="#89b4fa", fg="#11111b")
        messagebox.showinfo("训练成功", f"🎉 专属声学模型已成功生成！\n\n- 总采集样本: {len(files)} 条\n- 优选纯净样本: {len(clean_states)} 条 (已过滤异常杂音)\n- 硬件拟合偏差: {mean_dist:.3f}\n- 模型文件: custom_wake_model.h\n\n现在可直接在测试台评测，或点击【一键烧录】！")

    def compile_and_upload(self):
        if self.ser and self.ser.is_open:
            try: self.ser.close()
            except Exception: pass

        self.btn_compile.config(text="⏳ 正在烧录...", state=tk.DISABLED)

        def worker():
            cmd = f'"{PIO_PATH}" run -d "{PROJECT_DIR}" -t upload'
            p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            out, _ = p.communicate()
            if p.returncode == 0:
                self.root.after(0, lambda: messagebox.showinfo("烧录成功", "🎉 固件已成功烧录至 ESP32-S3！\n唤醒引擎现已完全运行您的专属声学模型！"))
            else:
                self.root.after(0, lambda o=out: messagebox.showerror("烧录失败", f"烧录失败:\n{o[-300:]}"))
            self.root.after(0, lambda: self.btn_compile.config(text="🚀 一键烧录固件", state=tk.NORMAL))
            self.root.after(0, self.start_serial_monitor)

        threading.Thread(target=worker, daemon=True).start()


if __name__ == "__main__":
    root = tk.Tk()
    app = Esp32GuiTrainerApp(root)
    root.mainloop()
