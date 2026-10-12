#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
高频扫频幅度分析器

固件的数据结构（看清楚再解析，这里我一开始就理解反了）：
    每个点 = [float 电压][float 频率][帧尾 00 00 80 7F] = 12 字节
所以 **一帧 = 2 个通道 × 1 个时间点**，512 帧组成一次突发。
不是"一帧装 512 个点"。

于是解析方式：按帧尾切成一个个 (电压, 频率) 对，
再把它们按 512 个一组分块（一块 = 一次突发，块内频率恒定），
块内算峰峰值，最后按频率分组取中位数。
"""
import socket, struct, sys, time
from collections import defaultdict
from statistics import median

HOST, PORT = "127.0.0.1", 5005
SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
TAIL = b"\x00\x00\x80\x7f"
POINTS = 512


def capture(sec):
    s = socket.create_connection((HOST, PORT), timeout=5)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    s.settimeout(0.3)
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < sec:
        try:
            c = s.recv(262144)
        except socket.timeout:
            continue
        if not c:
            break
        buf += c
    s.close()
    return bytes(buf), time.time() - t0


def main():
    print(f"连接 {HOST}:{PORT}，抓 {SECONDS:.0f} 秒（一轮扫频 15 秒）...")
    data, el = capture(SECONDS)
    print(f"收到 {len(data)} 字节 / {el:.1f}s = {len(data)/el/1024:.1f} KB/s\n")
    if not data:
        sys.exit("0 字节 —— 桥没在推数据？")

    # ---- 按帧尾切分，每帧 = [float 电压][float 频率] ----
    pairs = []                     # 按顺序的 (电压, 频率)
    bad = 0
    pos = 0
    while True:
        i = data.find(TAIL, pos)
        if i < 0:
            break
        payload = data[pos:i]
        if len(payload) == 8:
            v, f = struct.unpack("<ff", payload)
            pairs.append((v, f))
        else:
            bad += 1
        pos = i + 4

    print(f"解出 {len(pairs)} 个 (电压,频率) 对 / 异常段 {bad}")
    if len(pairs) < POINTS:
        sys.exit("数据太少！")

    # ---- 每 POINTS 个一组 = 一次突发；块内频率恒定，算峰峰值 ----
    groups = defaultdict(list)     # freq -> [每次突发的峰峰值]
    for k in range(0, len(pairs) - POINTS + 1, POINTS):
        chunk = pairs[k:k + POINTS]
        v = [p[0] for p in chunk]
        f = chunk[0][1]
        fw = sum(1 for p in chunk if abs(p[1] - f) > 0.5)
        if fw > POINTS // 10:
            continue               # 块跨越了换档边界，丢掉
        groups[int(round(f))].append(max(v) - min(v))

    if not groups:
        sys.exit("没形成任何频率分组！")

    # ---- 按频率出表 ----
    print(f"\n{'DAC频率':>10} {'帧数':>6} {'峰峰值(中位)':>13} {'最小':>9} {'最大':>9} "
          f"{'相对500Hz':>10}")
    print("-" * 66)
    base = None
    for freq in sorted(groups):
        pp = groups[freq]
        m = median(pp)
        if base is None:
            base = m
        rel = m / base if base > 1e-9 else 0.0
        print(f"{freq:>9}Hz {len(pp):>6} {m:>12.4f}V {min(pp):>8.4f}V {max(pp):>8.4f}V "
              f"{rel*100:>9.1f}%")
    print("-" * 66)

    freqs = sorted(groups)
    if len(freqs) >= 2:
        base = median(groups[freqs[0]])
        top = median(groups[freqs[-1]])
        print(f"\n{base*1000:.0f}->{freqs[-1]}Hz :  {base*1000:.0f}mV -> {top*1000:.0f}mV "
              f"（保留 {top/base*100:.0f}%）")
        if top / base > 0.9:
            print("结论：高频基本不掉 ✓")
        elif top / base > 0.5:
            print("结论：高频有明显衰减")
        else:
            print("结论：高频塌陷严重")


if __name__ == "__main__":
    main()
