#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""连上 rtt2tcp 的 TCP 口，抓一段 JustFloat 流并解码验证。"""
import socket, struct, sys, time
from collections import Counter

HOST, PORT = "127.0.0.1", 5005
SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
TAIL = b"\x00\x00\x80\x7f"

s = socket.create_connection((HOST, PORT), timeout=5)
s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
print(f"已连接 {HOST}:{PORT}，抓 {SECONDS} 秒 ...")

buf = bytearray()
t0 = time.time()
while time.time() - t0 < SECONDS:
    try:
        chunk = s.recv(65536)
    except socket.timeout:
        continue
    if not chunk:
        break
    buf += chunk
s.close()

elapsed = time.time() - t0
print(f"收到 {len(buf)} 字节 / {elapsed:.2f}s = {len(buf)/elapsed/1024:.1f} KB/s")

# ---- 按帧尾切分 ----
frames, ok, bad = [], 0, 0
pos = 0
while True:
    i = buf.find(TAIL, pos)
    if i < 0:
        break
    payload = bytes(buf[pos:i])
    n = len(payload)
    if n == 4:
        frames.append(struct.unpack("<f", payload)[0]); ok += 1
    else:
        bad += 1
        if bad <= 5:
            print(f"  [异常帧] 载荷 {n} 字节: {payload[:24].hex(' ')}")
    pos = i + 4

tail_rest = len(buf) - pos
print(f"\n帧数: 合法 {ok} / 异常 {bad} / 尾部残留 {tail_rest} 字节")
if not frames:
    sys.exit("没有解出任何帧！")

print(f"点数: {len(frames)}  应等于 每帧点数 × 帧数")
print(f"电压: min={min(frames):.4f} V  max={max(frames):.4f} V  "
      f"pp={max(frames)-min(frames):.4f} V  mean={sum(frames)/len(frames):.4f} V")

# ---- 帧长分布：确认"一个帧只含 1 个 float"（那就是 1 通道）----
print(f"帧率: {ok/elapsed:.1f} 帧/s   数据率: {len(buf)/elapsed/1024:.1f} KB/s")
print(f"每帧字节: {len(buf)/max(ok,1):.1f}（应为 8.0 = 4 字节 float + 4 字节帧尾）")

# ---- ASCII 迷你示波器：把最后一个窗口画出来 ----
N = 512
win = frames[-N:] if len(frames) >= N else frames
lo, hi = min(win), max(win)
H, W = 12, 100
print(f"\n最后一窗波形 (min={lo:.4f} max={hi:.4f} V):")
if hi - lo < 1e-6:
    print("  ⚠️ 完全平直 —— PB15 上就是直流，没接 PA4")
else:
    grid = [[" "] * W for _ in range(H)]
    for x in range(W):
        v = win[int(x * len(win) / W)]
        y = H - 1 - int((v - lo) / (hi - lo) * (H - 1))
        grid[y][x] = "*"
    for row in grid:
        print("  |" + "".join(row) + "|")
    # 数一下周期，反推频率
    mid = (lo + hi) / 2
    cross = sum(1 for k in range(1, len(win)) if win[k-1] < mid <= win[k])
    print(f"  上升沿穿越中值次数 = {cross}")
