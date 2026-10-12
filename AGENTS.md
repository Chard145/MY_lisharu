# MY_lisharu — STM32G431 可变频正弦波信号发生器

电赛信号类练手工程。DAC + TIM + DMA 输出连续可调频正弦波，用内部 OPAMP 跟随输出。

---

## 1. 仓库 / Git（踩过的坑，优先看）

- **仓库根是 `C:\Users\chard\Desktop\MY_lisharu`**，不是 `project\`。
  `project\` 只是仓库里的一个子目录。
  → **绝对不要在 `project\` 里再 `git init`**。曾经这么干过，结果提交进了嵌套仓库，
    在 GitHub 上完全看不到，排查了很久。
- remote：`https://github.com/Chard145/MY_lisharu.git`，分支 `main`。
- 凭据：Windows 凭据管理器里存着 `git:https://github.com`（用户 `Chard145`），
  没有装 credential manager，也不需要重新登录。

### 本机 git 网络的血泪坑

| 现象 | 原因 | 修法 |
|---|---|---|
| 所有 git 网络操作超时 | 全局配了 `http.proxy = http://127.0.0.1:57777`，但代理软件没开时端口是死的 | 不用代理时 `git config --global --unset http.proxy` 和 `--unset https.proxy` |
| `SSL certificate ... unable to get local issuer certificate (20)` | 全局 `http.sslbackend = openssl`，本机证书链验不过 | `git config --global http.sslbackend schannel`（用 Windows 证书库，实测直连 github.com:443 可通） |

排查顺序：先 `Test-NetConnection github.com -Port 443`，再 `git ls-remote origin`。
`ls-remote` **退出码 0 但零输出 = 仓库存在但完全是空的**（一次都没推成功）。

### 不要跟踪的东西

`build/`（一次编译 ~47MB 中间产物）、`.vscode/`、`compile_commands.json`、
以及根目录下 `project/build/Debug/` 里的 elf/hex/map —— 全部已在 `.gitignore` 里，
仓库排除后约 7.8MB / 158 个文件。

> **改中文源码文件时不要用 PowerShell 文本管道**
> （`(Get-Content -Raw) -replace ... | Set-Content -Encoding UTF8`）。
> 会把 UTF-8 中文按 GBK 误读，注释整片变成 `?`。要用 `edit` / `write` 工具。

---

## 2. 硬件信号链

```
TIM6 TRGO ──► DAC3 (DHR12R1, DMA1) ──► DAC3_OUT1
                                          │ 片上内部连线（不占引脚！）
                                          ▼
                                    OPAMP1 (电压跟随器)
                                          ▼
                                    PA2  (LQFP48 pin 10)  ← 示波器探头接这里
                                          │
                                          ▼
                                    PB15 (ADC2_IN15)  ← 回采，送 VOFA+
```

**关键约束（换芯片/改方案前必读）：**

- G4 系列里**只有 DAC3 / DAC4 能内部连到运放**，DAC1 / DAC2 不行。
- LQFP48 封装里 **DAC3 没有引出引脚** —— 所以走内部运放不是"图省事"，是唯一选择。
- **PA4 现在没有信号**（那是旧方案 DAC1 的输出脚）。
- `OPAMP1_BASE = 0x40010300`（APB2 域，**不是** 0x48000300）。
  G4 的外设基址和 F4 不一样，别照搬 F4 的地址。
- **G431 上不存在 `__HAL_RCC_OPAMP_CLK_ENABLE`**（CubeMX 里 `ClockEnableMode="none"`），
  G4 的运放没有时钟门控。别手写这个宏。
- `HAL_OPAMP_Init()` **只配置不启动**；必须**手动**再调一次 `HAL_OPAMP_Start(&hopamp1)`，
  而且它只在 `State == HAL_OPAMP_STATE_READY` 时才置 OPAMPEN。main.c 里跟在
  `Signal_Sweep_Init()` 后面。
- DAC 输出缓冲用 ST 官方配方：`DAC_OUTPUTBUFFER_DISABLE` + `CHIPCONNECT_INTERNAL`
  → `DAC_MCR` MODE1 = 0b011, MCR = 0x8003（HFSEL，>160MHz 用）。
  **不要自己写 `DAC3->MCR`**，试过 MODE1=0b001，内部通路直接断掉，只剩 98mV 串扰。

---

## 3. 失真的根因（本项目最重要的一条）

**症状**：低频正常；频率一高就"中频掉 1/2、1/4 正弦，高频掉电压直到趋近 1/2Vcc"。

**根因**：`sine_tab[]` 是 **1024 点整周期**表，但 DMA 只播**前 N 点** ——
实际播的是"一个周期的前 N/1024"，等效于**一段上升斜坡**，
幅度上限 = `sin(2π·N/1024)`。

实测吻合度：N=64→38%、N=32→20%、N=16→10%，和示波器读数完全对上。

**改法**（`Core/Src/signal.c`）：

- `sine_fill_n(n)`：按当前 N 重新生成 `sine_tab[i] = 2048 + 2047.5·sin(2π·i/N)`。
  → **N 不再需要整除 1024**，任意 N 都能出完整正弦。
- `signal_pick_n()`：从 `N = min(1024, 5MHz/f)` 往下扫，取频率误差 ≤ 0.05% 的 N。
- `SIGNAL_ADAPTIVE_N`：`0` = 参考工程固定 N=100；`1` = 原分频段表；
  **`2` = 推荐**。
- `SIGNAL_DAC_MAX_RATE = 5000000`（实测 DAC 最高更新率约 5MHz，
  50kHz × 100 点仍满幅；早先估的 2MHz 是错的）。

**实测结果**：500Hz ~ 50kHz 全程 **3.25Vpp 满幅**，频率误差 ≤ 0.05%
（固定 N=100 时误差 0.58%）。

**取舍三角**：N（平滑度） ↔ `ARR+1 = 170e6/(f·N)`（频率分辨率） ↔ `f·N`（DAC 更新率）。
想在高频多要几个点，就把 `SIGNAL_DAC_MAX_RATE` 提到 8000000 试试。

**选型注意**：`sine_tab[]` 是按 N 现填的，**换 N 必须重新填表**，
`signal.c` 里用 `s_npts` / `s_last_npts` 判重，改动时别把这个逻辑弄丢。

---

## 4. 时钟与关键参数

- STM32G431C8T6，LQFP48，**170MHz**（HSI 16MHz → PLLM=DIV4 → 4MHz → PLLN=85 → PLLP=DIV2），
  APB1 = APB2 = 170MHz。
- 扫频档位（`signal_slow.c`）：500 / 1000 / 2000 / 3000 / 5000 /
  8000 / 12000 / 20000 / 30000 / 50000 Hz，每档 1.5 秒，一轮 15 秒。
  起点定 500Hz 是因为 ADC 跑到 500kSPS 后一窗只有 2048 点 = 4.1ms，
  100Hz 的 10ms 周期装不下，峰峰值会被低估。
- ADC 采集：`ADC_VOFA_FS_HZ = 500000`，`ADC_VOFA_POINTS = 512`，
  采样时间 6.5 周期，TIM7 `ARR = 170000000/500000 - 1`。
- 资源占用：RAM 27240 B（83%），FLASH ~38488 B（29%）。

---

## 5. 构建 / 烧录 / 上位机

工具链（本机绝对路径）：

| 工具 | 路径 |
|---|---|
| CMake | `D:\AI_Models\CMark\bin\cmake.exe` |
| Ninja | `D:\AI_Models\ninja\ninja-win\ninja.exe` |
| arm-gcc | `D:\AI_Models\ARM_GNU_Toolchain\arm-gnu-toolchain-15.2.rel1-mingw-w64-x86_64-arm-none-eabi\bin\` |
| J-Link | `D:\MY_tool_at_ALL\J_Flash_APP\J-linkV9.64SEGGER\JLink_V964\` |

```powershell
cmake --build "C:\Users\chard\Desktop\MY_lisharu\project\build\Debug"
# objcopy 转 hex，然后
JLink.exe -CommanderScript flash.jlink
```

> **烧录前必须先关掉 RTT 桥** —— J-Link 独占。

### RTT → VOFA+ 数据桥

- 桥脚本：`project\Port\rtt2tcp.bat`（走 `Port\rtt2tcp.py`）。
  必须用 `Start-Process explorer.exe -ArgumentList "<bat>"` 启动，
  直接 `Start-Process <bat>` 的话窗口会随工具调用一起死掉。
- SEGGER RTT 中间件放在 `project\Port\SEGGER_RTT\`。
  **故意不放 `Middlewares/`** —— CubeMX 重新生成工程会把整个 `Middlewares/` 删掉。
- 通道 0 = JustFloat 流（每点 `[float32][00 00 80 7F]`），512 点一帧。
  现在是 **2 通道**：ch0 = 电压，ch1 = 当前扫频频率（12 字节/点）。
- VOFA+：数据引擎 **JustFloat**，数据接口 **TCP Client**，`127.0.0.1:5005`，
  **通道数改成 2**，Δt ≈ 0.05ms。
  （VOFA+ 重启后波形控件会丢，要重新添加并绑定通道。）

**RTT 的血泪坑**：
- J-Link 允许第二个进程打开，但第二个进程 `rtt_start()` 会**静默杀掉**已在跑的桥
  —— 不报错，就是 0 字节/秒。所以 `--doctor` 默认只做静态检查，`--online` 才联网探测。
- CPU 被 halt 时 RTT 断流。

---

## 6. 相关技能

- `jlink-rtt-bridge` —— RTT / J-Scope / VOFA+ 传数据的完整用法与本机现场记忆。
- `scope-sds1052a` —— 示波器读数（LeCroy 老式命令集，不是标准 SCPI）。

可选的下一步（尚未做）：PA2 后面加 RC 重构滤波（1kΩ + 1nF，fc ≈ 159kHz）。
