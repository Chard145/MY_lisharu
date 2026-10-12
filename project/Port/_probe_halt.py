#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""诊断：MCU 是否被 halt 住了？RTT 到底有没有在产数据？"""
import argparse, importlib.util, sys, time

spec = importlib.util.spec_from_file_location("r", r"D:\skill\jlink-rtt-bridge\rtt2tcp.py")
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)

pylink, lib = m.import_pylink()
ns = argparse.Namespace(dll=None, device="STM32G431C8", speed=4000)
jl = m.open_jlink(ns, pylink, lib)
print("[probe] 已连接 S/N=%s" % jl.serial_number)

# ---- 1) 读 DHCSR：Debug Halting Control and Status Register ----
DHCSR = 0xE000EDF0
try:
    w = jl.memory_read32(DHCSR, 1)[0]
    c_debugen = (w >> 0) & 1
    c_halt    = (w >> 1) & 1
    s_halt    = (w >> 17) & 1
    s_sleep   = (w >> 18) & 1
    s_lockup  = (w >> 19) & 1
    print(f"[probe] DHCSR = 0x{w:08X}")
    print(f"        C_DEBUGEN={c_debugen}  C_HALT={c_halt}  S_HALT={s_halt}  "
          f"S_SLEEP={s_sleep}  S_LOCKUP={s_lockup}")
    print("        ==> " + ("★ MCU 处于 HALT 状态（CPU 停了，固件不跑）" if (c_halt or s_halt)
                           else "MCU 在运行"))
except Exception as e:
    print(f"[probe] 读 DHCSR 失败: {e}")

# ---- 2) 先在不做任何动作的情况下读 2 秒 RTT ----
def read_for(sec):
    tot = 0; t0 = time.time()
    while time.time() - t0 < sec:
        d = m.rtt_to_bytes(jl.rtt_read(0, 8192))
        tot += len(d)
        if not d: time.sleep(0.002)
    return tot

n1 = read_for(2)
print(f"[probe] 复位前 2 秒 RTT 读到 {n1} 字节  -> {'有数据' if n1 > 1000 else '★ 无数据'}")

# ---- 3) 让 CPU 跑起来（reset + run），再读 ----
if len(sys.argv) > 1 and sys.argv[1] == "--resume":
    print("[probe] 执行 restart()（复位并运行）...")
    try:
        jl.restart()
    except Exception as e:
        print(f"[probe] restart 失败: {e}")
    time.sleep(1.0)
    n2 = read_for(3)
    print(f"[probe] 复位后 3 秒 RTT 读到 {n2} 字节  -> {'✓ 恢复了！' if n2 > 1000 else '✗ 仍然无数据'}")
else:
    print("[probe] （加 --resume 参数可执行复位并运行）")

m.shutdown(jl)
