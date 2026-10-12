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
>
> **带中文的 `.ps1` 必须存成 UTF-8 *带 BOM*。**
> Windows PowerShell 5.1 对无 BOM 的 `.ps1` 按 ANSI(GBK) 读，
> 中文注释会被解成乱码，连字符串引号都会被吃掉，
> 报错是 `The string is missing the terminator` / `Missing closing '}'`，
> 看起来像语法错误，其实是编码。`write` 工具写出来的是无 BOM 的，
> 补一句字节级操作即可（不要用文本管道，会二次转码）：
> ```powershell
> $b = [IO.File]::ReadAllBytes($p); $bom = [byte[]](0xEF,0xBB,0xBF)
> $n = New-Object byte[] ($bom.Length + $b.Length)
> [Array]::Copy($bom,0,$n,0,3); [Array]::Copy($b,0,$n,3,$b.Length)
> [IO.File]::WriteAllBytes($p, $n)
> ```
> 纯 ASCII 的 `.bat` / `.ps1` 不受影响（所以 `install.bat` 那类才写成全英文）。

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

## 6. 频率跟踪 / 波形复现（`Core/Src/track.c`）

目标：输出与对方"**同频 + 同波形 + 同电压 + 同相位**"（做李萨如图形演示用）。

### 数据流

```
对方信号 ──┬─> PB15 (ADC2_IN15)  ─> adc_buf[2048] ─> 过零插值测频 ─> Signal_SetFreq()
           │    （模拟通路）                             │
           └─> PB15 (EXTI15)     ─> 上升沿中断 ─────────┴─> Signal_Realign()  ★相位对齐★
                （数字通路）
本机输出 <── PA2 (DAC3 → OPAMP1)
```

### 四件事分别怎么实现的

| 要素 | 做法 |
|---|---|
| **同频** | 上升沿过零 + 线性插值 + 滑窗跨多周期。开环标定实测误差 **0.000~0.004%**（1k~50kHz） |
| **同波形** | 判据 = `min(贴下轨比例, 贴上轨比例)`：方波≈500‰、正弦≈205‰、三角≈100‰、恒定电平≈0 |
| **同电压** | 把包络量出的对方低/高电平码值**原样**交给 `Signal_SetLevels()`，不归一化 |
| **同相位** | **PB15 一芯两用**：ADC 走 pad 的模拟通路、EXTI 走数字通路，互不干扰 → 不用多加线 |

### ★ PB15 一芯两用（本项目最取巧的一处）

参考工程 `test4_prj` 用 COMP2(PA3)+TIM2/TIM3 输入捕获(PA15/PB4) 测频，
但它的 `sarg`（相位角）是**手填的固定值**，并没有闭环相位对齐。
本工程用更省的做法：把 PB15 从 `GPIO_MODE_ANALOG` 改成 `GPIO_MODE_IT_RISING`，
**ADC2_IN15 照样能采**（两条通路在芯片里是分开的），白得一个精确的上升沿事件。

已实测验证：把 MODER 用 J-Link 写回"模拟"后 `adc_buf` 依然全 0
（说明当时引脚本来就是 0V），即 EXTI 配置**不会**破坏模拟通路。

### 关键常量（`track.c`）

- `TRK_ENV_PERIOD 32` —— 峰值包络衰减周期。**必须用包络检波，不能用"每 512 点统计 min/max"**：
  方波半周期可能远长于 512 点，整段落在一个电平里会让峰峰值算成 0 → 误判"无信号"（踩过）
- `TRK_MIN_CROSS 3` / `TRK_MAX_CROSS 12` / `TRK_SPAN_SAMPLES 1500`
- `TRK_EDGE_GUARD_CYC 3400` —— EXTI 去抖 20µs（DWT 周期计数）
- `TRK_LEVEL_EPS 16` / `TRK_DUTY_EPS 0.01` —— 电平/占空比变化阈值，
  因为每次写都可能触发重铺表（0.5ms 冻结输出），不能跟着包络抖动乱写

### 踩过的四个坑（都在这里）

1. **自监控闭环失控**：`signal_apply()` 换频时写 `DHR12R1 = sine_tab[0]`（中点），
   这个台阶被自己的 ADC 采到成了假上升沿 → 一路从 1kHz 爬到 17kHz。
   修法：换频保持相位连续（读 DMA 的 CNDTR 反推相位再重铺表）。**接真实外部信号时不会发生。**
2. **换频冻结输出**：`sine_fill_n()` 每次换频跑 N 次 `sinf()`（N=1022 约 0.5ms），
   期间 TIM6 停着、输出被冻住；跟踪器每秒调几十次 → 测量窗口经常整窗落在冻结点上。
   修法：**点数没变时走快路径，只动 TIM6 的 PSC/ARR，完全不碰 DMA 和表**。
   另外短暂丢信号**不要**清 `s_set_hz`，否则"丢信号→强制换频→输出被冻→更像丢信号"滚成正反馈。
3. **方波锁不上**：见上面 `TRK_ENV_PERIOD` 那条。
4. **恒定电平误判成方波**：只统计"贴任意一轨"的总比例时，恒高电平算出来是 1000‰。
   **必须取上下轨比例的 min。** 后果很隐蔽：对方信号一断，输出会从正弦悄悄变方波。

### 调试工具

- `Port\adc_live.bat` —— 实时显示 PB15 上的 min/max/pk 电压。
  **排查"跟踪器读不到信号"的第一件事就是跑它**：pk≈4000 有信号，pk≈0 是条直线。
- `g_trk_hold_hz` 写非 0 值 → 开环标定模式：锁定输出频率，只报告测量值，用来单独验证测频精度。
- 全局量：`g_trk_in_hz / out_hz / pk / valid / cross_n / edges / wave / vmin / vmax / duty_ppm`
  （地址从 `arm-none-eabi-nm build/Debug/project.elf` 查）

### 已知物理限制

- **方波边沿** = 一个 DAC 更新周期（N=1024、150Hz 时约 6.5µs）。DAC 物理上做不到更陡。
  真要 ns 级边沿只能改用定时器 PWM 输出，但那样就出不了正弦了（需求要同时支持，所以不用）。
- **相位精度** ≈ 0.03~0.1°（EXTI 中断响应抖动）。要更高得加线走 TIM 硬件从模式 + 改 CubeMX 的
  DAC 触发源，代价大收益小。
- 方波测频时线性插值把沿定位在两个样本中点，但这是**恒定偏差**，首尾相减时自动抵消。

## 7. 相关技能

- `jlink-rtt-bridge` —— RTT / J-Scope / VOFA+ 传数据的完整用法与本机现场记忆。
- `scope-sds1052a` —— 示波器读数（LeCroy 老式命令集，不是标准 SCPI）。

可选的下一步（尚未做）：PA2 后面加 RC 重构滤波（1kΩ + 1nF，fc ≈ 159kHz）。
