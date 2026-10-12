#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 rtt2tcp.py  ——  SEGGER RTT  ->  TCP 联通桥          (portable / v1.1)
================================================================================
 用途：让 VOFA+ / 任意 TCP 客户端接收走 SWD(J-Link RTT) 的数据。

 为什么需要它：
   RTT 不是串口。它是"目标机 RAM 里的环形缓冲 + J-Link 后台轮询"，
   上位机（VOFA+ 只有 串口/TCP/UDP/文件 四种接口）读不了，
   必须有人读出来再转发。本脚本就干这一件事。

 第一次用？先跑自检：
   python rtt2tcp.py --doctor

 常用：
   python rtt2tcp.py                          # 127.0.0.1:5005, SWD 4000kHz
   python rtt2tcp.py --device STM32F407VE     # 换芯片
   python rtt2tcp.py --channel 1              # 读 RTT 通道 1（J-Scope 数据常用 1）
   python rtt2tcp.py --list-channels          # 列出目标机所有 RTT 通道
   python rtt2tcp.py --speed 12000            # 提 SWD 时钟 → 更高吞吐

 依赖：
   pip install pylink-square
   + SEGGER J-Link Software and Documentation Pack（提供 JLink_x64.dll）
   + J-Link 硬件，目标固件里跑过 SEGGER_RTT_Init()

 VOFA+ 侧：数据接口=TCP Client, IP=127.0.0.1, 端口=5005,
           数据引擎按固件实际协议选（JustFloat / FireWater / rawdata）

 ============================== 三条铁律 ==============================
  1. 本脚本运行期间**独占 J-Link** —— 不能同时烧录、不能同时开 J-Scope。
     顺序永远是：先烧录 → 再跑桥。要烧录就先关掉桥。
  2. **没有 TCP 客户端连接时也要继续把 RTT 读空并丢弃**（本脚本就是这么做的）。
     否则目标机缓冲很快写满，NO_BLOCK_SKIP 模式下固件开始丢帧，
     等上位机连上时看到的是断断续续的波形 —— 极难排查。
  3. **不要在传数据时 halt CPU**（断点/单步/JLink.exe 的 `h`）。
     halt 期间 RTT 不流动，缓冲一满就丢数据。
================================================================================
"""

import argparse
import glob
import os
import platform
import socket
import sys
import threading
import time

VERSION = "1.1"

# ============================== 默认参数 ==============================
DEF_HOST = "127.0.0.1"
DEF_PORT = 5005
DEF_SPEED = 4000            # kHz SWD 时钟
DEF_DEVICE = "STM32G431C8"  # 只是默认值，别的板子用 --device 覆盖
DEF_CHANNEL = 0             # 默认读 RTT 通道 0（终端）
READ_CHUNK = 4096           # 单次向 RTT 要多少字节；太小会明显拖慢吞吐
STALL_WARN_SEC = 8.0        # 连续读不到字节多久就报"被挤掉"警告（见 drain_rtt）
MAX_CLIENTS = 8             # 最多允许多少个 TCP 客户端同时收（扇出）

IS64 = sys.maxsize > 2**32
WANT_DLL = "JLink_x64.dll" if IS64 else "JLinkARM.dll"

# 环境变量里可以指定 J-Link 安装目录（跨机器时最省事的一招）
ENV_KEYS = ("JLINK_INSTALL_DIR", "JLINK_PATH", "SEGGER_JLINK_PATH")


# ============================================================================
#                          J-Link DLL 自动发现
# ============================================================================
def _from_env():
    out = []
    for k in ENV_KEYS:
        v = os.environ.get(k)
        if not v:
            continue
        if os.path.isfile(v):
            out.append(v)
        elif os.path.isdir(v):
            out.append(os.path.join(v, WANT_DLL))
    return out


def _from_registry():
    """SEGGER 安装包会写注册表，这是最可靠的自动发现途径。"""
    dirs = []
    if platform.system() != "Windows":
        return dirs
    try:
        import winreg
    except ImportError:
        return dirs
    hives = [winreg.HKEY_LOCAL_MACHINE, winreg.HKEY_CURRENT_USER]
    subs = [r"SOFTWARE\SEGGER\J-Link",
            r"SOFTWARE\WOW6432Node\SEGGER\J-Link",
            r"SOFTWARE\SEGGER\JLink",
            r"SOFTWARE\WOW6432Node\SEGGER\JLink"]
    for hive in hives:
        for sub in subs:
            try:
                with winreg.OpenKey(hive, sub) as key:
                    try:
                        v, _ = winreg.QueryValueEx(key, None)   # 默认值
                        if isinstance(v, str) and os.path.isdir(v):
                            dirs.append(v)
                    except OSError:
                        pass
                    i = 0
                    while True:
                        try:
                            _, val, _ = winreg.EnumValue(key, i)
                        except OSError:
                            break
                        i += 1
                        if isinstance(val, str) and os.path.isdir(val):
                            dirs.append(val)
            except OSError:
                continue
    return dirs


def _from_common_paths():
    pats = [
        r"C:\Program Files\SEGGER\JLink*",
        r"C:\Program Files (x86)\SEGGER\JLink*",
        r"D:\Program Files\SEGGER\JLink*",
        r"D:\SEGGER\JLink*",
        os.path.expanduser(r"~\SEGGER\JLink*"),
        "/opt/SEGGER/JLink*",
        "/Applications/SEGGER/JLink*",
    ]
    dirs = []
    for p in pats:
        dirs.extend(sorted(glob.glob(p), reverse=True))   # 新版本目录排前面
    return dirs


def _scan_roots(max_depth=3):
    """最后手段：在盘符根部浅扫。慢，但能救回装在奇葩目录的情况。"""
    roots = []
    if platform.system() == "Windows":
        for drive in "CDEFGH":
            d = drive + ":\\"
            if os.path.isdir(d):
                roots.append(d)
    else:
        roots.append(os.path.expanduser("~"))
    hits = []
    for root in roots:
        for depth in range(1, max_depth + 1):
            pat = os.path.join(root, *(["*"] * depth), WANT_DLL)
            try:
                hits.extend(glob.glob(pat))
            except OSError:
                pass
        if hits:
            break
    return hits


def find_dll(explicit=None, verbose=False):
    """按「显式 → 环境变量 → 注册表 → 常见路径 → 浅扫」的顺序找 JLink DLL。

    ★★ 位数必须匹配 ★★
       64 位 Python 只能加载 JLink_x64.dll；加载 32 位的 JLinkARM.dll 会报
       "Expected to be given a valid DLL." —— 见到这个报错九成是位数或路径不对。
    """
    tried = []

    def _check(p):
        if not p:
            return None
        tried.append(p)
        if not os.path.isfile(p):
            return None
        # 位数自检：文件名里带 x64 的只能是 64 位 Python 用
        low = os.path.basename(p).lower()
        if IS64 and low == "jlinkarm.dll":
            return None
        if (not IS64) and low == "jlink_x64.dll":
            return None
        return p

    if explicit:
        r = _check(explicit)
        if r:
            return r
        sys.exit(f"指定的 DLL 不可用（不存在或位数不匹配）: {explicit}\n"
                 f"本机 Python 是 {'64' if IS64 else '32'} 位，需要 {WANT_DLL}")

    for cand in _from_env():
        r = _check(cand)
        if r:
            return r

    for d in _from_registry() + _from_common_paths():
        r = _check(os.path.join(d, WANT_DLL))
        if r:
            return r

    for p in _scan_roots():
        r = _check(p)
        if r:
            return r

    if verbose:
        print("[dll  ] 已尝试以下位置，均未找到:")
        for p in tried:
            print(f"          {p}  {'存在' if os.path.exists(p) else '不存在'}")
    return None


def dll_not_found_hint():
    return (
        f"找不到 {WANT_DLL}（本机 Python 是 {'64' if IS64 else '32'} 位）。\n"
        "  → 安装 SEGGER J-Link Software and Documentation Pack（免费）：\n"
        "      https://www.segger.com/downloads/jlink/\n"
        "  → 或用环境变量指路（推荐，跨机器最省事）:\n"
        "      set JLINK_INSTALL_DIR=C:\\Program Files\\SEGGER\\JLink\n"
        "  → 或运行时手动指：\n"
        "      python rtt2tcp.py --dll <JLink_x64.dll 的完整路径>"
    )


# ============================================================================
#                              pylink 封装
# ============================================================================
def import_pylink():
    try:
        import pylink
        from pylink import library as pylink_library
        return pylink, pylink_library
    except ImportError:
        sys.exit("缺少依赖，请先执行:  python -m pip install pylink-square")


def open_jlink(args, pylink, pylink_library):
    dll = find_dll(args.dll)
    if not dll:
        sys.exit(dll_not_found_hint())

    print(f"[jlink] DLL: {dll}")
    print(f"[jlink] Python {sys.version.split()[0]} ({'64' if IS64 else '32'} 位)")

    lib = pylink_library.Library(dllpath=dll)
    jlink = pylink.JLink(lib=lib)
    jlink.open()
    jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
    jlink.connect(args.device, speed=args.speed)

    # rtt_start 偶尔失败（目标刚复位、控制块还没初始化完），重试几次
    last_err = None
    for _ in range(10):
        try:
            jlink.rtt_start()
            if jlink.rtt_get_num_up_buffers() >= 1:
                return jlink
        except Exception as e:          # noqa: BLE001 - pylink 抛的异常类型不稳定
            last_err = e
        time.sleep(0.3)
    raise RuntimeError(
        "RTT 启动失败。检查：\n"
        "  (1) 固件里真的调了 SEGGER_RTT_Init()\n"
        "  (2) 控制块 _SEGGER_RTT 在 RAM 里且没被优化掉（用 nm 查）\n"
        "  (3) 目标芯片型号 --device 填对了\n"
        f"  原始错误: {last_err}")


def rtt_to_bytes(data):
    """pylink 各版本 rtt_read 返回类型不一（list[int] / bytes），统一成 bytes"""
    if data is None:
        return b""
    if isinstance(data, (bytes, bytearray)):
        return bytes(data)
    return bytes(bytearray(data))


def list_channels(jlink):
    """打印目标机所有 RTT 通道。J-Scope 认的是通道名，所以这个功能很有用。"""
    n_down = jlink.rtt_get_num_down_buffers()
    n_up = jlink.rtt_get_num_up_buffers()
    print(f"[rtt  ] 下行通道 {n_down} 个，上行通道 {n_up} 个")
    js_ch = None
    for up in (True, False):
        n = n_up if up else n_down
        for i in range(n):
            try:
                d = jlink.rtt_get_buf_descriptor(i, up)
                name = getattr(d, "acName", getattr(d, "sName", None))
                if isinstance(name, bytes):
                    name = name.decode("ascii", "replace")
                if name is None:
                    name = str(d)
                size = getattr(d, "SizeOfBuffer", "?")
                tag = ""
                if up and name.startswith("JScope_"):
                    tag = "   ← J-Scope 数据通道"
                    js_ch = i
                print(f"[rtt  ] {'UP  ' if up else 'DOWN'} #{i}: {name!r:24} {size:>6} B{tag}")
            except Exception as e:      # noqa: BLE001
                print(f"[rtt  ] {'UP  ' if up else 'DOWN'} #{i}: <读取失败 {e}>")
    if js_ch is not None:
        print(f"[rtt  ] J-Scope 会读通道 #{js_ch}；想转发给 VOFA+ 就加 --channel {js_ch}")
    return js_ch


# ============================================================================
#                                 自检
# ============================================================================
def doctor(args):
    """跨机器排错用：把这条链路上每一环都查一遍。"""
    print(f"=== J-Link RTT 桥 环境自检  (rtt2tcp.py v{VERSION}) ===")
    print(f"操作系统 : {platform.system()} {platform.release()}  ({platform.machine()})")
    print()

    ok = True

    # --- 1) Python ---
    print(f"[1/6] Python      {sys.version.split()[0]}  ({'64' if IS64 else '32'} 位)")
    if not IS64:
        print("      ⚠ 32 位 Python 只能用 32 位的 JLinkARM.dll，部分新版 J-Link 包已不提供")

    # --- 2) pylink ---
    pylink = pylink_library = None
    try:
        import pylink as _p
        from pylink import library as _l
        pylink, pylink_library = _p, _l
        ver = getattr(_p, "__version__", "?")
        print(f"[2/6] pylink-square {ver}   OK")
    except ImportError:
        ok = False
        print("[2/6] pylink-square  ✗ 未安装")
        print("      → python -m pip install pylink-square")

    # --- 3) DLL ---
    dll = find_dll(args.dll)
    if dll:
        print(f"[3/6] JLink DLL   OK  {dll}")
    else:
        ok = False
        print(f"[3/6] JLink DLL   ✗ 找不到 {WANT_DLL}")
        print("      " + dll_not_found_hint().replace("\n", "\n      "))

    if not ok:
        print("\n结论：先把上面 ✗ 的项解决，再跑 --doctor。")
        return 1

    # --- 4) 在线检查：默认不做！（血泪坑，见下）---
    #
    # ★★ 2026-10-11 实测踩到的坑 ★★
    #   J-Link **允许多个进程同时打开** —— 桥正在跑的时候，第二个进程照样能
    #   jlink.open() + connect() 成功，一点报错都没有。
    #   但第二个连接一旦走到 rtt_start()，正在运行的那条 RTT 会话就被重置：
    #   桥的 rtt_read() 从此**永远返回 0 字节**。
    #   表现极具迷惑性 —— 桥进程还活着、5005 还在监听、也不报错，就是一个字节都不来。
    #   （当时误判成"固件挂了"，排查了好几轮。）
    #
    #   所以：默认只做静态检查；在线检查必须显式 --online，
    #   而且做之前要确认 J-Link 没被桥 / J-Scope / 烧录器占着。
    if not args.online:
        print("[4/6] J-Link 在线检查  已跳过（默认不做）")
        print("      ⚠ 在线检查会 connect + rtt_start，**会打断正在运行的 RTT 会话**")
        print("        （桥会变成「端口还在监听、但收不到字节」）")
        print("      确需在线检查（此时 J-Link 必须空闲）:  python rtt2tcp.py --doctor --online")
        print()
        print("结论：静态检查全过，直接开跑即可：")
        print(f"      python rtt2tcp.py --device {args.device} --speed {args.speed} "
              f"--channel {args.channel} --port {args.port}")
        return 0

    print(f"[4/6] J-Link 在线检查  尝试 SWD @{args.speed}kHz, device={args.device} ...")
    jlink = None
    try:
        jlink = open_jlink(args, pylink, pylink_library)
        vt = ""
        try:
            vt = f"  VTref={jlink.hardware_status.voltage}"
        except Exception:               # noqa: BLE001
            pass
        print(f"      OK  S/N={jlink.serial_number}{vt}")
    except SystemExit:
        raise
    except Exception as e:              # noqa: BLE001
        msg = str(e)
        print(f"      ⚠ 连不上：{msg.splitlines()[0] if msg else e}")
        print("      若提示 already open —— 桥/烧录器/J-Scope 正占着 J-Link，")
        print("      这是**正常现象**，关掉占用方再试，或直接跳到下面的结论。")
        print("\n结论：静态检查全过，在线检查跳过。可以开始用桥了。")
        return 0

    # --- 5) RTT 通道 ---
    print("[5/6] RTT 通道")
    try:
        js = list_channels(jlink)
        if js is None:
            print("      ℹ 没有 JScope_ 开头的通道 —— J-Scope 模式还用不了，")
            print("        但转发给 VOFA+ 不受影响（用默认 --channel 0/1 即可）")
        # 建议命令
        n_up = jlink.rtt_get_num_up_buffers()
        usable = [i for i in range(n_up)]
        if js is not None:
            rec_ch = js
        elif 1 in usable:
            rec_ch = 1
        else:
            rec_ch = DEF_CHANNEL
        print(f"[6/6] 建议命令")
        print(f"      python rtt2tcp.py --device {args.device} --speed {args.speed} "
              f"--channel {rec_ch} --port {args.port}")
    except Exception as e:              # noqa: BLE001
        print(f"      读取通道失败: {e}")

    print("\n结论：链路每一环都通了，开跑吧。")
    shutdown(jlink)                     # 不返回；见 shutdown() 里的说明
    return 0


# ============================================================================
#                                  主流程
# ============================================================================
class Stats:
    def __init__(self):
        self.total = 0
        self.window = 0
        self.t0 = time.time()

    def tick(self, seconds=2.0):
        now = time.time()
        if now - self.t0 < seconds:
            return None
        rate = self.window / (now - self.t0)
        self.window = 0
        self.t0 = now
        return rate


def parse_args():
    p = argparse.ArgumentParser(
        description=f"SEGGER RTT -> TCP bridge (v{VERSION}) for VOFA+ etc.")
    p.add_argument("--host", default=DEF_HOST, help=f"监听地址 (默认 {DEF_HOST})")
    p.add_argument("--port", type=int, default=DEF_PORT, help=f"监听端口 (默认 {DEF_PORT})")
    p.add_argument("--speed", type=int, default=DEF_SPEED, help=f"SWD 速度 kHz (默认 {DEF_SPEED})")
    p.add_argument("--device", default=DEF_DEVICE, help=f"目标芯片 (默认 {DEF_DEVICE})")
    p.add_argument("--channel", type=int, default=DEF_CHANNEL,
                   help=f"RTT 上行通道号 (默认 {DEF_CHANNEL})")
    p.add_argument("--dll", default=None, help="JLink DLL 路径（默认自动搜索）")
    p.add_argument("--list-channels", action="store_true",
                   help="只列出目标机的 RTT 通道名然后退出")
    p.add_argument("--doctor", action="store_true",
                   help="环境自检：查 Python/pylink/DLL，打印建议命令（默认不连目标机）")
    p.add_argument("--online", action="store_true",
                   help="配合 --doctor：额外做在线检查（会 connect + rtt_start，"
                        "**会打断正在运行的 RTT 会话**，仅在 J-Link 空闲时用）")
    return p.parse_args()


def shutdown(jlink):
    """干净收尾。

    ★ 为什么最后要 os._exit(0) ★
      pylink 的 JLink.__del__ 会再调一次 _finalize() —— 但那时解释器已经在拆模块，
      JLinkARM DLL 可能已被卸载，于是在退出瞬间抛一句很难看的
      「OSError: exception: access violation writing 0x...」回溯。
      它发生在所有正常逻辑之后（exit code 仍是 0），纯粹是噪音，
      但会让人以为是崩了。所以显式关完连接后直接结束进程，跳过析构。
    """
    try:
        jlink.rtt_stop()
    except Exception:                   # noqa: BLE001
        pass
    try:
        jlink.close()
    except Exception:                   # noqa: BLE001
        pass
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


def main():
    args = parse_args()

    if args.doctor:
        sys.exit(doctor(args))

    pylink, pylink_library = import_pylink()

    print(f"[jlink] 打开 SWD @ {args.speed} kHz, device={args.device} ...")
    jlink = open_jlink(args, pylink, pylink_library)
    print(f"[jlink] 连接成功  S/N={jlink.serial_number}")
    try:
        print(f"[jlink] VTref = {jlink.hardware_status.voltage}")
    except Exception:                   # noqa: BLE001
        pass

    if args.list_channels:
        list_channels(jlink)
        shutdown(jlink)                 # 不返回，见 shutdown() 里的说明
        return

    try:
        desc = jlink.rtt_get_buf_descriptor(args.channel, True)
        print(f"[rtt  ] 上行缓冲 #{args.channel} 大小 {desc.SizeOfBuffer} 字节")
    except Exception as e:              # noqa: BLE001
        print(f"[rtt  ] 读缓冲描述失败（不影响转发）: {e}")

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(MAX_CLIENTS)             # ★ 支持多客户端并发，见下面 client 列表的说明
    srv.settimeout(0.2)
    print(f"[tcp  ] 监听 {args.host}:{args.port}，等上位机以 TCP Client 连入 "
          f"（最多 {MAX_CLIENTS} 个并发）...")

    stats = Stats()
    # ★ 为什么是「客户端列表」而不是单个 client ★
    #   原版只服务 1 个客户端（listen(1) + 单个变量）。后果很烦：
    #   VOFA+ 连着的时候，我的自检脚本 _rtt_verify.py 一连上就把它顶掉了；
    #   反过来也一样（脚本连上了却收不到数据，被误判成"桥挂了"）。
    #   实录：VOFA+ 已连接 → 我跑脚本 → 脚本报 0 字节 → 我误判成故障。
    #   改成"扇出"：谁来连都收同一份数据，互不影响。
    clients = []
    client_lock = threading.Lock()
    stop_flag = threading.Event()

    def drain_rtt():
        """后台线程：只做一件事——把 RTT 读空，绝不让目标机缓冲写满"""
        nonlocal clients
        zero_since = None               # 连续读不到字节的起点，用于卡死自诊断
        warned = False

        while not stop_flag.is_set():
            try:
                data = rtt_to_bytes(jlink.rtt_read(args.channel, READ_CHUNK))
            except Exception as e:      # noqa: BLE001
                print(f"\n[rtt  ] 读取异常: {e}")
                time.sleep(0.5)
                continue

            if not data:
                # ★ 卡死自诊断（2026-10-11 对照实验确认的坑）★
                #   J-Link 允许多进程同时打开，第二个进程的 rtt_start() 会把先来者的
                #   RTT 会话挤掉 —— 且**不报任何错**。表现就是本进程还活着、端口还在
                #   监听、但永远读不到字节。不提示的话根本猜不出原因。
                if zero_since is None:
                    zero_since = time.time()
                elif (not warned) and (time.time() - zero_since > STALL_WARN_SEC):
                    warned = True
                    print(f"\n[warn ] 已连续 {STALL_WARN_SEC:.0f} 秒读不到任何字节。")
                    print("[warn ] 最可能的原因：有另一个进程连过 J-Link（J-Scope、烧录器、")
                    print("[warn ] 或者又一次 --doctor/--list-channels），把本进程的 RTT 会话挤掉了。")
                    print("[warn ] J-Link 允许多进程同时打开且不报错，所以无法自动恢复。")
                    print("[warn ] 解法：关掉本窗口，**单独**重新启动桥（确保没别的程序占 J-Link）。")
                    sys.stdout.flush()
                time.sleep(0.0005)      # 没数据歇 0.5ms，别把 CPU 烧了
                continue

            zero_since = None
            warned = False

            stats.total += len(data)
            stats.window += len(data)

            with client_lock:
                targets = list(clients)
            if not targets:
                continue                # ★ 没客户端也要读，只是丢弃（铁律 2）

            dead = []
            for c in targets:
                try:
                    c.sendall(data)
                except OSError:
                    dead.append(c)
            if dead:
                with client_lock:
                    for c in dead:
                        try:
                            c.close()
                        except OSError:
                            pass
                        if c in clients:
                            clients.remove(c)
                print(f"\n[tcp  ] {len(dead)} 个客户端断开，剩 {len(clients)} 个")

    reader = threading.Thread(target=drain_rtt, daemon=True)
    reader.start()

    try:
        while True:
            try:
                conn, addr = srv.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                with client_lock:
                    if len(clients) >= MAX_CLIENTS:
                        conn.close()
                        print(f"\n[tcp  ] 拒绝 {addr[1]}：已达上限 {MAX_CLIENTS}")
                    else:
                        clients.append(conn)
                        print(f"\n[tcp  ] 客户端 #{len(clients)} 已连接 {addr[0]}:{addr[1]}")
            except socket.timeout:
                pass

            rate = stats.tick()
            if rate is not None:
                with client_lock:
                    ncli = len(clients)
                mark = f"{ncli} 个" if ncli else "未连接"
                print(f"\r[stat ] {rate/1024:8.1f} KB/s   累计 {stats.total/1024:9.1f} KB   "
                      f"客户端 {mark}   ", end="", flush=True)

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n[exit ] 退出中 ...")
    finally:
        stop_flag.set()
        reader.join(timeout=1.0)
        with client_lock:
            for c in clients:
                try:
                    c.close()
                except OSError:
                    pass
            clients.clear()
        srv.close()
        shutdown(jlink)


if __name__ == "__main__":
    main()
