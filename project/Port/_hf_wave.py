#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把每个频点的一个突发画成 ASCII 波形 —— 判断"幅度掉了"还是"波形被削成阶梯"。"""
import socket, struct, sys, time
from collections import OrderedDict

HOST, PORT = "127.0.0.1", 5005
SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 16.0
TAIL = b"\x00\x00\x80\x7f"
POINTS = 512
W = 110
H = 11

s = socket.create_connection((HOST, PORT), timeout=5)
s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
s.settimeout(0.3)
buf = bytearray(); t0 = time.time()
while time.time() - t0 < SECONDS:
    try: buf += s.recv(262144)
    except socket.timeout: pass
s.close()

pairs = []; pos = 0
while True:
    i = buf.find(TAIL, pos)
    if i < 0: break
    if i - pos == 8:
        pairs.append(struct.unpack("<ff", bytes(buf[pos:i])))
    pos = i + 4

# 每个频率取第一个完整突发
seen = OrderedDict()
for k in range(0, len(pairs) - POINTS + 1, POINTS):
    ch = pairs[k:k+POINTS]
    f = int(round(ch[0][1]))
    if abs(ch[-1][1] - ch[0][1]) > 0.5:   # 跨换档，丢
        continue
    if f not in seen:
        seen[f] = [p[0] for p in ch]

print(f"抓到 {len(seen)} 个频点的波形\n")
for f in sorted(seen):
    v = seen[f]
    lo, hi = min(v), max(v)
    print(f"─── {f} Hz   峰峰值 {hi-lo:.4f} V   (min {lo:.3f}  max {hi:.3f}) ───")
    if hi - lo < 1e-6:
        print("   （平直，没信号）\n")
        continue
    grid = [[" "] * W for _ in range(H)]
    for x in range(W):
        y = H - 1 - int((v[int(x * len(v) / W)] - lo) / (hi - lo) * (H - 1))
        grid[max(0, min(H - 1, y))][x] = "*"
    for row in grid:
        print("   |" + "".join(row) + "|")
    print()
