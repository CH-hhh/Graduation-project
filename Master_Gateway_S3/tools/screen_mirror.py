#!/usr/bin/env python3
"""
ESP32-S3 屏幕实时投屏工具 (USB 串口) - 颜色与性能修复版
===================================================
将 ESP32-S3 的 240×320 LCD 屏幕内容通过 USB 串口实时投射到 PC 窗口。

依赖安装: pip install pyserial pygame numpy
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("错误: 请先安装 pyserial: pip install pyserial")
    sys.exit(1)

try:
    import pygame
except ImportError:
    print("错误: 请先安装 pygame: pip install pygame")
    sys.exit(1)

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False
    print("提示: 未检测到 numpy，将使用纯 Python 解码 (推荐安装 numpy: pip install numpy)")


# 屏幕参数
SCREEN_W = 240
SCREEN_H = 320
FRAME_SIZE = SCREEN_W * SCREEN_H * 2  # RGB565, 153600 bytes
HEADER_SIZE = 8


def rgb565_to_surface_numpy(pixel_data: bytes, width: int, height: int, swap_rb: bool = False):
    """使用 NumPy 大端 (>u2) 极速解析 RGB565"""
    raw16 = np.frombuffer(pixel_data, dtype='>u2')
    
    r = ((raw16 >> 11) & 0x1F) << 3
    g = ((raw16 >> 5) & 0x3F) << 2
    b = (raw16 & 0x1F) << 3
    
    if swap_rb:
        rgb888 = np.stack([b, g, r], axis=-1).astype(np.uint8).reshape((height, width, 3))
    else:
        rgb888 = np.stack([r, g, b], axis=-1).astype(np.uint8).reshape((height, width, 3))
        
    return pygame.surfarray.make_surface(np.transpose(rgb888, (1, 0, 2)))


def rgb565_to_surface_fallback(pixel_data: bytes, width: int, height: int, swap_rb: bool = False):
    """纯 Python 大端字节序解析回退方案"""
    surface = pygame.Surface((width, height))
    pixels = pygame.PixelArray(surface)
    idx = 0
    for y in range(height):
        for x in range(width):
            if idx + 1 < len(pixel_data):
                high = pixel_data[idx]
                low = pixel_data[idx + 1]
                rgb565 = (high << 8) | low
                
                r = ((rgb565 >> 11) & 0x1F) << 3
                g = ((rgb565 >> 5) & 0x3F) << 2
                b = (rgb565 & 0x1F) << 3
                
                if swap_rb:
                    pixels[x, y] = (b, g, r)
                else:
                    pixels[x, y] = (r, g, b)
            idx += 2
    del pixels
    return surface

import struct

def rle_decode_to_surface_numpy(rle_data: bytes, width: int, height: int, swap_rb: bool = False):
    """使用 NumPy 极速 RLE 解码 (0.2ms)"""
    arr = np.frombuffer(rle_data, dtype=np.uint8)
    if len(arr) % 3 != 0:
        arr = arr[:(len(arr) // 3) * 3]
    if len(arr) == 0:
        return pygame.Surface((width, height))
        
    arr = arr.reshape(-1, 3)
    counts = arr[:, 0]
    highs = arr[:, 1].astype(np.uint16)
    lows = arr[:, 2].astype(np.uint16)
    raw16 = (lows << 8) | highs  # 字节序回转: TFT_eSPI大端存储 → ESP32小端读取 → 需回转

    pixels16 = np.repeat(raw16, counts)

    target_len = width * height
    if len(pixels16) < target_len:
        pixels16 = np.pad(pixels16, (0, target_len - len(pixels16)))
    elif len(pixels16) > target_len:
        pixels16 = pixels16[:target_len]

    r = ((pixels16 >> 11) & 0x1F) << 3
    g = ((pixels16 >> 5) & 0x3F) << 2
    b = (pixels16 & 0x1F) << 3

    if swap_rb:
        rgb888 = np.stack([b, g, r], axis=-1).astype(np.uint8).reshape((height, width, 3))
    else:
        rgb888 = np.stack([r, g, b], axis=-1).astype(np.uint8).reshape((height, width, 3))

    return pygame.surfarray.make_surface(np.transpose(rgb888, (1, 0, 2)))


def rle_decode_fallback(rle_data: bytes, width: int, height: int, swap_rb: bool = False):
    """纯 Python RLE 解码回退方案"""
    surface = pygame.Surface((width, height))
    pixels = pygame.PixelArray(surface)
    
    x, y = 0, 0
    idx = 0
    total_bytes = len(rle_data)
    
    while idx + 2 < total_bytes and y < height:
        count = rle_data[idx]
        high = rle_data[idx + 1]
        low = rle_data[idx + 2]
        idx += 3
        
        rgb565 = (low << 8) | high  # 字节序回转: TFT_eSPI大端存储 → ESP32小端读取 → 需回转
        r = ((rgb565 >> 11) & 0x1F) << 3
        g = ((rgb565 >> 5) & 0x3F) << 2
        b = (rgb565 & 0x1F) << 3
        color = (b, g, r) if swap_rb else (r, g, b)
        
        for _ in range(count):
            if x < width and y < height:
                pixels[x, y] = color
            x += 1
            if x >= width:
                x = 0
                y += 1
                if y >= height:
                    break
    del pixels
    return surface


def find_sync(ser: serial.Serial):
    """在串口数据流中极速寻找帧同步头 0xAA 0x55 0xFB 0x00 0x00 0xF0 0x01 0x40"""
    ser.timeout = 0.1
    buf = bytearray()
    start_time = time.time()
    HEADER = b'\xAA\x55\xFB\x00\x00\xF0\x01\x40'

    while time.time() - start_time < 3.0:  # ⬆ 增加到 3 秒，给 ESP32 足够时间启动帧流
        # 大块读取数据
        n = ser.in_waiting
        chunk = ser.read(max(n, 4096) if n else 4096)
        if chunk:
            buf.extend(chunk)

            while True:
                idx = buf.find(b'\xAA\x55\xFB')
                if idx == -1:
                    # 没找到包头，保留最后 3 个字节防跨包断裂
                    if len(buf) > 3:
                        del buf[:-3]
                    break

                # 检查是否有完整的 8 字节 header
                header_end = idx + 8
                if len(buf) < header_end:
                    # 长度不够，等待下一次循环补充读取
                    break

                # 校验完整 8 字节包头
                if buf[idx:header_end] == HEADER:
                    # 校验成功！返回同步头之后的所有残留数据
                    return buf[header_end:]
                else:
                    # 误匹配（可能是 RGB565 像素恰好包含 AA 55 FB），跳过这 1 个字节继续向后寻找
                    buf = buf[idx+1:]

    return None


def list_serial_ports():
    """列出所有可用串口"""
    import serial.tools.list_ports
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("未检测到任何串口设备")
        return []
    
    print("\n可用串口设备:")
    print("-" * 60)
    for i, port in enumerate(ports):
        print(f"  [{i+1}] {port.device} - {port.description}")
    print("-" * 60)
    return ports


def send_mirror_on(ser: serial.Serial):
    """向 ESP32 发送投屏开启命令（只发一次，避免重复触发）"""
    ser.write(b"MIRROR_ON\r\n")
    ser.flush()


def main():
    parser = argparse.ArgumentParser(description="ESP32-S3 屏幕实时投屏工具")
    parser.add_argument("--port", type=str, default=None,
                        help="串口端口号 (如 COM7)")
    parser.add_argument("--scale", type=int, default=2, choices=[1, 2, 3],
                        help="窗口放大倍数 (默认 2x)")
    parser.add_argument("--baud", type=int, default=2000000,
                        help="波特率 (默认 2000000)")
    parser.add_argument("--swap-rb", action="store_true",
                        help="红蓝通道反转 (若颜色依旧偏色可加此选项)")
    args = parser.parse_args()
    
    port = args.port
    if not port:
        ports = list_serial_ports()
        if not ports:
            sys.exit(1)
        if len(ports) == 1:
            port = ports[0].device
            print(f"\n自动选择: {port}")
        else:
            try:
                choice = int(input("\n请输入串口编号: ")) - 1
                port = ports[choice].device
            except (ValueError, IndexError):
                print("无效选择")
                sys.exit(1)
    
    print(f"\n正在连接 {port}...")
    try:
        ser = serial.Serial(port, args.baud, timeout=0.1)
        ser.dtr = True
        ser.rts = True
        ser.reset_input_buffer()
    except serial.SerialException as e:
        print(f"串口连接失败: {e}")
        sys.exit(1)
    
    print("串口连接成功!")
    send_mirror_on(ser)

    pygame.init()
    win_w = SCREEN_W * args.scale
    win_h = SCREEN_H * args.scale
    screen = pygame.display.set_mode((win_w, win_h))
    pygame.display.set_caption(f"ESP32-S3 Screen Mirror ({SCREEN_W}x{SCREEN_H} @{args.scale}x)")
    
    font = pygame.font.SysFont("Microsoft YaHei", 18)
    screen.fill((20, 20, 30))
    text = font.render("等待 ESP32 帧数据...", True, (100, 200, 255))
    screen.blit(text, (win_w // 2 - text.get_width() // 2, win_h // 2 - 10))
    pygame.display.flip()
    
    HEADER_RLE = b'\xAA\x55\xFC'
    HEADER_RAW = b'\xAA\x55\xFB'

    def parse_dims(buf, idx):
        """从 8 字节包头解析宽/高 (大端), 校验屏幕尺寸合法性"""
        if len(buf) < idx + 8:
            return None
        w = (buf[idx + 4] << 8) | buf[idx + 5]
        h = (buf[idx + 6] << 8) | buf[idx + 7]
        if w in (240, 320) and h in (240, 320):
            return w, h
        return None

    stream_buffer = bytearray()
    frame_count = 0
    fps_timer = time.time()
    last_retry_time = time.time()
    last_frame_time = time.time()
    frame_w, frame_h = SCREEN_W, SCREEN_H
    frame_size = frame_w * frame_h * 2
    running = True
    
    print("🖥️  投屏程序已启动! 按 ESC 退出。\n")
    
    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
            
            if not running:
                break
            
            # 大块读取数据追加至 stream_buffer
            try:
                n = ser.in_waiting
                if n > 0:
                    chunk = ser.read(n)
                    if chunk:
                        stream_buffer.extend(chunk)
            except Exception as e:
                print(f"\n串口读取异常: {e}")
                break

            idx_rle = stream_buffer.find(HEADER_RLE)
            idx_raw = stream_buffer.find(HEADER_RAW)

            # 1. 优先解析 RLE 压缩包 (0xFC)
            if idx_rle != -1 and (idx_raw == -1 or idx_rle < idx_raw):
                if idx_rle > 0:
                    del stream_buffer[:idx_rle]

                dims = parse_dims(stream_buffer, 0)
                if dims is None:
                    # 包头不完整或误匹配, 跳过 1 字节继续找
                    if len(stream_buffer) >= 8:
                        del stream_buffer[0]
                    continue
                fw, fh = dims

                # 检查是否有 12 字节（8 字节包头 + 4 字节 Payload 长度）
                if len(stream_buffer) >= 12:
                    payload_len = struct.unpack('>I', stream_buffer[8:12])[0]
                    if len(stream_buffer) >= 12 + payload_len:
                        rle_data = bytes(stream_buffer[12 : 12 + payload_len])
                        del stream_buffer[: 12 + payload_len]
                        last_frame_time = time.time()

                        if (fw, fh) != (frame_w, frame_h):
                            frame_w, frame_h = fw, fh
                            frame_size = frame_w * frame_h * 2
                            win_w = frame_w * args.scale
                            win_h = frame_h * args.scale
                            screen = pygame.display.set_mode((win_w, win_h))
                            pygame.display.set_caption(
                                f"ESP32-S3 Screen Mirror ({frame_w}x{frame_h} @{args.scale}x)"
                            )

                        if HAS_NUMPY:
                            surface = rle_decode_to_surface_numpy(rle_data, frame_w, frame_h, swap_rb=args.swap_rb)
                        else:
                            surface = rle_decode_fallback(rle_data, frame_w, frame_h, swap_rb=args.swap_rb)

                        if args.scale != 1:
                            surface = pygame.transform.scale(
                                surface, (frame_w * args.scale, frame_h * args.scale)
                            )
                        screen.blit(surface, (0, 0))

                        frame_count += 1
                        elapsed = time.time() - fps_timer
                        if elapsed >= 1.0:
                            fps = frame_count / elapsed
                            pygame.display.set_caption(
                                f"ESP32-S3 Screen Mirror ({frame_w}x{frame_h} @{args.scale}x) - {fps:.1f} FPS"
                            )
                            frame_count = 0
                            fps_timer = time.time()

                        pygame.display.flip()
                        continue

            # 2. 兼容原始 Uncompressed 帧包 (0xFB)
            elif idx_raw != -1:
                if idx_raw > 0:
                    del stream_buffer[:idx_raw]

                dims = parse_dims(stream_buffer, 0)
                if dims is None:
                    if len(stream_buffer) >= 8:
                        del stream_buffer[0]
                    continue
                fw, fh = dims
                raw_frame_size = fw * fh * 2

                if len(stream_buffer) >= 8 + raw_frame_size:
                    frame_data = bytes(stream_buffer[8 : 8 + raw_frame_size])
                    del stream_buffer[: 8 + raw_frame_size]
                    last_frame_time = time.time()

                    if (fw, fh) != (frame_w, frame_h):
                        frame_w, frame_h = fw, fh
                        frame_size = frame_w * frame_h * 2
                        win_w = frame_w * args.scale
                        win_h = frame_h * args.scale
                        screen = pygame.display.set_mode((win_w, win_h))
                        pygame.display.set_caption(
                            f"ESP32-S3 Screen Mirror ({frame_w}x{frame_h} @{args.scale}x)"
                        )

                    if HAS_NUMPY:
                        surface = rgb565_to_surface_numpy(frame_data, frame_w, frame_h, swap_rb=args.swap_rb)
                    else:
                        surface = rgb565_to_surface_fallback(frame_data, frame_w, frame_h, swap_rb=args.swap_rb)

                    if args.scale != 1:
                        surface = pygame.transform.scale(
                            surface, (frame_w * args.scale, frame_h * args.scale)
                        )
                    screen.blit(surface, (0, 0))

                    frame_count += 1
                    elapsed = time.time() - fps_timer
                    if elapsed >= 1.0:
                        fps = frame_count / elapsed
                        pygame.display.set_caption(
                            f"ESP32-S3 Screen Mirror ({frame_w}x{frame_h} @{args.scale}x) - {fps:.1f} FPS"
                        )
                        frame_count = 0
                        fps_timer = time.time()

                    pygame.display.flip()
                    continue

            # 限制 stream_buffer 大小，防止缓冲区无限积压
            if len(stream_buffer) > (frame_size * 2):
                stream_buffer.clear()

            # 若超时未收到新帧，重发 MIRROR_ON 开启指令
            now = time.time()
            if now - last_frame_time > 3.0 and now - last_retry_time > 3.0:
                stream_buffer.clear()
                ser.reset_input_buffer()
                send_mirror_on(ser)
                last_retry_time = now

            time.sleep(0.002)
    
    except KeyboardInterrupt:
        print("\n收到退出指令...")
    
    finally:
        try:
            ser.write(b"MIRROR_OFF\r\n")
            ser.flush()
            time.sleep(0.05)
            ser.close()
            print("已停止 ESP32 投屏输出。")
        except Exception:
            pass
        
        pygame.quit()
        print("程序已成功退出。")


if __name__ == "__main__":
    main()
