/**
  ******************************************************************************
  * @file    signal.c
  * @brief   DAC3 + TIM6 + DMA 循环 + 内部运放跟随 → 持续正弦波，频率可调（PA2 输出）
  *
  * ============================ 为什么是 DAC3 不是 DAC1 ============================
  *   STM32G4 的运放内部直连只支持 DAC3/DAC4，**DAC1/DAC2 不参与**
  *   （CubeMX 芯片数据库原文：OPAMP1 的条件是 "Only active when DAC3 channel
  *    out1 is enabled"）。而且 G431(LQFP48) 上 DAC3 没有任何外部引脚 ——
  *   它就是一个专门喂运放的内部 DAC。
  *   通路：DAC3_OUT1 ─(片内)─> OPAMP1_VINP ─> OPAMP1 跟随器 ─> PA2
  *
  * ============================ 工作原理 ============================
  *   TIM6 每 (ARR+1) 个计数发一次 TRGO 脉冲
  *        → DAC3_OUT1 被硬件触发一次转换
  *        → DMA1_Channel1(Circular) 自动把正弦表"下一个点"搬进 DAC 数据寄存器
  *        → 搬完整张表自动回到表头，波形永续输出，CPU 全程不参与
  *
  * ============================ 频率公式 ============================
  *   f_out = 170000000 / ((PSC+1) * (ARR+1) * N)
  *           其中 N = 本次使用的正弦表点数
  *
  * ========================= 为什么表点数要变 =========================
  *   如果永远用固定的 64 点表：20kHz 时 (PSC+1)(ARR+1) 只剩 133，
  *   分给 ARR 只有 8 个台阶 → 频率步进 ±3.8%，频率根本对不上。
  *   所以按倍频程分 14 段，每段换一组 (PSC, N)，让
  *       (PSC+1)*(ARR+1) = f_TIM/(f*N) ≈ 518 ~ 1037
  *   恒定，ARR 就稳定落在 518~1037，全频段频率误差 < 0.1%，
  *   采样率始终远高于输出频率（最少 16 点/周期）。
  *
  * ======================== DMA 位宽必须对齐 ========================
  *   CubeMX 里 DAC3_CH1 的 DMA 配的是 Peripheral / Memory 都是 Word(32位)，
  *   而 DAC 用 DAC_ALIGN_12B_R 时目标地址是 DHR12R1（数据在低 12 位）。
  *   因此正弦表元素必须是 uint32_t：
  *     若用 uint16_t，DMA 每次搬运在内存里前进 4 字节 = 跳过两个点，
  *     1024 点的表只用到前 512 点 → 输出"后半周被折叠"且频率减半
  *     （默认 1kHz 会变成 500Hz、周期 2ms）。
  ******************************************************************************
  */

#include "signal.h"
#include "dac.h"
#include "tim.h"
#include <math.h>

/* ============================ 可调参数 ============================ */

/* TIM6 输入时钟 = APB1 定时器时钟 = 170MHz（改主频必须同步改这里！）*/
#define TIM6_CLK_HZ             170000000UL

/* ==================== 波形生成方式（对比实验开关）====================
 * SIGNAL_ADAPTIVE_N = 1 : 自适应点数（本工程原方案）
 *     低频用 1024 点，高频逐段减少（50kHz 时 32 点）
 *     → 波形最平滑的频段很宽，但高频点数少、阶梯明显
 *     → 好处：DAC 更新率始终 ≤ ~2MHz，50kHz 也扛得住
 *
 * SIGNAL_ADAPTIVE_N = 0 : 固定点数（参考工程 test4_prj 的方案）
 *     **全频段都用 SIGNAL_FIXED_N 个点**，频率只靠 TIM6 的 PSC/ARR 调。
 *     参考工程就是 dots=100 定死，然后改 htim7 的 Prescaler/Period。
 *     → 好处：每个周期点数恒定，低频到中频波形质量完全一致
 *     → 代价：DAC 更新率 = f_out × N
 *              50kHz × 100 = 5MHz  ← 远超 DAC 能力，高频会崩
 *            而且 (ARR+1) = 170e6/(f×N) 变小 → 频率分辨率变差
 *              50kHz 时 (ARR+1)=34  →  频率误差 2.9%
 *
 * 做失真度对比时把下面这个开关切一下重新编译即可。
 *
 * SIGNAL_ADAPTIVE_N = 2 : 【推荐】自动选最优 N
 *     两个约束一起考虑：
 *       ① N 取"该频率下 DAC 更新率不超过上限"的最大值 → 波形尽量平滑
 *          低频自动回到 1024 点，高频自动降到 ~100 点
 *       ② 在 N_max 以内扫描，挑"实际能输出、且最接近目标频率"的组合
 *          → 同时把频率误差压到最小
 *     这一档比前两档都好：低频比参考工程平滑，高频和参考工程一样干净，
 *     而且频率精度反而更高（因为 N 也可以参与凑数）。 */
#define SIGNAL_ADAPTIVE_N       2
#define SIGNAL_FIXED_N          100U    /* 参考工程的 dots=100 */

/* DAC 更新率上限（Hz）—— 实测出来的硬指标
 *   50kHz × 100 点 = 5MHz 时幅度仍然满幅、阶梯消失，说明 5MHz 没问题。
 *   ⚠️ 这个值直接决定高频能有多少点/周期（N_max = 本值 / f），
 *      调大了高频更平滑但可能超出 DAC 能力（表现为幅度塌陷、波形畸变）。 */
#define SIGNAL_DAC_MAX_RATE     5000000UL

/* ====================== 1024 点正弦表（12 位，0~4095）=====================
 * ★ 必须是 RAM 里的可写数组，且在 Signal_Init() 里运行时生成 ★
 *   实测踩坑：把正弦表写成 Flash 里的 `static const` 数组时，波形会被截断
 *   （示波器上只看到 1/4 个正弦）；换成运行时在 RAM 里用 sinf() 生成后，
 *   波形立刻变成完整的漂亮正弦。原因未完全定论（怀疑与 DMA 对 Flash 读取
 *   的位宽/时序有关），但**实测结论明确**，所以采用运行时生成方案。
 *
 * 表值定义：sine_tab[i] = round(2048 + 2047.5 * sin(2*pi*(i/N + phase0)))
 *   中心 2048 -> 1.65V；满幅 0~4095 -> 0~3.3V
 *   phase0 = 0 时首点 = 2048（过零点）；非 0 时按当前输出相位生成，
 *   换频时波形无缝接上，**不再有相位台阶**（原因见下面 sine_fill_n 的说明）。
 * 元素类型必须是 uint32_t —— 和 DMA 的 Word 传输宽度严格一致。
 */
/* ================== 多波形输出参数（2026-10-12 加，为"跟频+同波形+同电压+同相位"服务）==================
 * ★ 这几个必须定义在 wave_fill_n() **之前** —— 表生成函数要用它们。
 *   （原来工程里所有 static 状态都堆在 sine_fill_n 后面，加多波形时踩过一次编译错误。） */
static uint8_t  s_wave_mode  = SIG_WAVE_SINE;
static uint32_t s_vmin       = 0U;      /* 波形低电平码值（对方实测值直接搬过来） */
static uint32_t s_vmax       = 4095U;   /* 波形高电平码值 */
static float    s_duty       = 0.5f;    /* 方波占空比（高电平占比） */
static float    s_phase_off  = 0.0f;    /* 相位微调，单位"周期"（= 度/360） */
static uint8_t  s_phaselock  = 0U;      /* 1 = 锁相模式：表头恒为相位 0，配合 Signal_Realign() */
static uint8_t  s_dirty      = 1U;      /* 波形/电平/占空比/相位 变了 → 下次必须重铺表 */

static uint32_t sine_tab[SIN_MAX_POINTS];

/**
  * @brief  按"每周期 n 个点"生成正弦表到 RAM
  *
  * ★★★ 这里是本工程最深的那个坑（2026-10-12 定位）★★★
  *
  *   原来的写法是：一次性生成 1024 点完整正弦，之后 DMA 只搬**前 N 个点**，
  *   还注释说"N 是 1024 的约数，所以一张表通吃所有频段"。
  *   **这是错的。** 取前 N 个点 = 只取了那 1024 点正弦的前 N/1024 个周期，
  *   播放出来根本不是正弦，而是一段**重复的上升斜坡**：
  *
  *       N=1024 -> 0°~360°  完整正弦       幅度 100%
  *       N= 256 -> 0°~ 90°  四分之一段      幅度 = sin(90°)  = 100%  ← 看着还像样
  *       N=  64 -> 0°~22.5° 十六分之一段    幅度 = sin(22.5°)=  38%
  *       N=  32 -> 0°~11.25°                幅度 = sin(11.25°)= 20%
  *       N=  16 -> 0°~5.6°                  幅度 = sin(5.6°)  = 10%
  *       N=   8 -> 0°~2.8°                  幅度 = sin(2.8°)  =  5%
  *
  *   实测完全对上：500Hz/1kHz 1.66Vpp，2kHz 0.64V，3kHz 0.33V，8kHz 0.19V，
  *   而且波形最小值永远钉在中点 1.65V（斜坡起点就是中点，下不去）——
  *   这就是之前反复出现的"只有 1/4 个正弦""高频掉电压"的真身。
  *
  *   正确做法：**每换一个 N，就按 sin(2*pi*i/N) 重新铺满前 N 个点**，
  *   这样 N 个点恰好是一个完整周期，配合
  *       f_out = f_TIM / ((PSC+1) * (ARR+1) * N)
  *   就永远输出完整正弦。（频率公式本来就是对的，错的是表的内容。）
  *
  * 表值：round(2048 + 2047.5*sin(...))，中心 2048 -> 1.65V，满幅 0~4095 -> 0~3.3V。
  *
  * ================ ★ 相位连续（2026-10-12 加，为频率跟踪而改）★ ================
  *   原来每次换频都从相位 0 重新起振，配合 signal_apply() 里那句
  *       DAC3->DHR12R1 = sine_tab[0];       （= 中点 1.65V）
  *   会在输出上留下一个**直接跳到中点的台阶**。
  *
  *   以前扫频测失真时看不出来（1.5 秒才换一次档）；但做频率跟踪之后换频可能
  *   每秒几百次，而且——自监控（PB15 接回 PA2）时那个台阶会被自己的 ADC 采到：
  *   信号当时若在负半周，台阶就是一次"上升沿穿越阈值"，于是多数出一个沿
  *   → 测得的频率偏高 → 输出调高 → 换频更频繁 → 更多假沿。
  *   实测就是一路从 1kHz 爬到 17kHz 的**正反馈失控**。
  *
  *   改法：铺表时带上**起始相位** phase0（单位"周期"，0~1）：
  *       sine_tab[i] = 2048 + 2047.5 * sin(2*pi*(i/n + phase0))
  *   换频前先从 DMA 的 CNDTR 反推当前播到第几个点，换算成 phase0 再重铺，
  *   这样表首点恰好等于"输出此刻的值"，重启后波形无缝接上，台阶消失。
  *   phase0 = 0 时行为与改动前完全一致。
  *
  * @param  n      每周期点数
  * @param  phase0 起始相位，单位"周期"，任意实数（内部归一化到 [0,1)）
  */
static void wave_fill_n(uint32_t n, float phase0)
{
    uint32_t i;
    float    lo = (float)s_vmin;
    float    hi = (float)s_vmax;

    if (n < 2U)
    {
        n = 2U;
    }
    if (n > SIN_MAX_POINTS)
    {
        n = SIN_MAX_POINTS;
    }

    /* 归一化到 [0,1)，防止调用方传进来越界的相位 */
    phase0 = phase0 - (float)((int32_t)phase0);
    if (phase0 < 0.0f)
    {
        phase0 += 1.0f;
    }

    for (i = 0U; i < n; i++)
    {
        /* 本点在一个周期里的位置，[0,1) */
        float ph = (float)i / (float)n + phase0;
        float v;

        ph = ph - (float)((int32_t)ph);
        if (ph < 0.0f)
        {
            ph += 1.0f;
        }

        switch (s_wave_mode)
        {
            case SIG_WAVE_SQUARE:
                /* 方波：ph < duty 时是高电平。
                 * ★ 表第 0 点（ph=0）就是高电平 = 上升沿所在位置，
                 *   所以 Signal_Realign() 把 DMA 挪回表头时，
                 *   输出的上升沿与对方的上升沿对齐。
                 * 边沿陡度：DAC 一个更新周期一步到位，这是 DAC 物理上能做到的最陡，
                 *   想再陡只能换定时器 PWM 输出（本工程不用，因为要同时支持正弦）。 */
                v = (ph < s_duty) ? hi : lo;
                break;

            case SIG_WAVE_TRIANGLE:
                /* 三角波：ph=0 从最低点开始上升，ph=0.5 到顶，ph=1 回到底 */
                v = lo + (hi - lo) * ((ph < 0.5f) ? (ph * 2.0f) : (2.0f - ph * 2.0f));
                break;

            case SIG_WAVE_SAWTOOTH:
                /* 上升锯齿：ph=0 从最低点线性升到最高点 */
                v = lo + (hi - lo) * ph;
                break;

            case SIG_WAVE_SINE:
            default:
                /* 正弦：ph=0 在正中、且正在上升 —— 也就是"从负穿正的过零点"，
                 * 与方波的上升沿在语义上对齐（都是"信号开始上升的那一刻"）。 */
                v = (lo + hi) * 0.5f + (hi - lo) * 0.5f * sinf(6.28318530718f * ph);
                break;
        }

        if (v < 0.0f)
        {
            v = 0.0f;
        }
        if (v > 4095.0f)
        {
            v = 4095.0f;
        }
        sine_tab[i] = (uint32_t)(v + 0.5f);
    }
}

/* ============================== 频段表 ==============================
 * 频段划分：k 段覆盖 [20*2^(k-2) , 40*2^(k-2)) Hz，k = 2..10
 *   等价于 k = max{ k | 10*2^(k-1) <= f }，由 band_index() 计算。
 *
 * ★ 关于 N 的两件事（2026-10-12 更新）★
 *
 *   ① N 不再需要是 1024 的约数了！
 *      以前 N 必须是 SIN_MAX_POINTS 的约数，因为"一张 1024 点表通吃所有频段"
 *      （那本身就是 bug，见 sine_fill_n）。现在每次换 N 都按 sin(2*pi*i/N)
 *      重铺表，所以 N 可以是**任意整数**。
 *
 *   ② N 越大波形越平滑，但 DAC 的更新率 = f_out × N 不能超过器件上限。
 *      实测（本机 + 示波器/ADC 双重确认）：
 *        DAC 更新率 ≤ 800kHz  波形干净
 *        1.3MHz 附近  开始出现台阶圆滑（更新跟不上）
 *      所以 N 按"更新率不超过 ~800kHz"来配。
 *
 *   下表把高频段的 N 整体翻倍（原来的 N 太小，高频阶梯肉眼可见）：
 *      频段        范围          旧 N   新 N   DAC 更新率(段顶)
 *      k=8    1280~2560 Hz      64    128      328 kHz
 *      k=9    2560~5120 Hz      32     64      328 kHz
 *      k=10   5120~10240 Hz     16     64      655 kHz
 *      k=11  10240~20480 Hz     16     32      655 kHz
 *      k=12  20480~40960 Hz      8     32     1.31 MHz
 *      k=13  40960~81920 Hz      8     16     1.31 MHz
 *      k=14  81920~163840 Hz     4      8     1.31 MHz
 *
 *   代价：N 变大 → (ARR+1) 变小 → 频率分辨率变差。
 *   本工程要求 (ARR+1) ≥ 100 才能保证 <1% 频率误差，
 *   50kHz/N=16 时 (ARR+1)=212，仍然满足。
 *   ⚠️ 再往上堆 N 就要同时检查频率精度了，别只看波形好看。
 */
#define SIGNAL_BANDS   14U

static const uint32_t band_psc[SIGNAL_BANDS] =
{
     7U,    /* k=1  [  10,   20) Hz   N=1024   20Hz 以下被钳位，此段备用 */
     7U,    /* k=2  [  20,   40) Hz   N=1024 */
     3U,    /* k=3  [  40,   80) Hz   N=1024 */
     3U,    /* k=4  [  80,  160) Hz   N=1024 */
     1U,    /* k=5  [ 160,  320) Hz   N=512  */
     1U,    /* k=6  [ 320,  640) Hz   N=256  */
     0U,    /* k=7  [ 640, 1280) Hz   N=256  */
     0U,    /* k=8  [1280, 2560) Hz   N=128  */
     0U,    /* k=9  [2560, 5120) Hz   N=64   */
     0U,    /* k=10 [5120,10240) Hz   N=64   */
     0U,    /* k=11 [10240,20480) Hz  N=32   */
     0U,    /* k=12 [20480,40960) Hz  N=32   */
     0U,    /* k=13 [40960,81920) Hz  N=16   */
     0U     /* k=14 [81920,163840) Hz N=8    */
};

static const uint16_t band_n[SIGNAL_BANDS] =
{
    1024U, 1024U, 1024U, 512U, 512U, 256U, 256U, 256U, 128U, 128U, 64U, 64U, 32U, 16U
};

/* ============================ 内部状态 ============================ */
static uint8_t  s_running = 0U;         /* 是否已启动 */
static uint32_t s_psc     = 0U;         /* 当前预分频值 */
static uint32_t s_arr     = 0U;         /* 当前重装载值 */
static uint32_t s_npts    = 1024U;      /* 当前使用的表点数 */
static uint32_t s_live_n  = 0U;         /* ★ 正在 DMA 上跑的表长，用来反推当前相位 */
static float    s_tab_phase = 0.0f;     /* ★ 当前正弦表里已经烘焙进去的相位偏移(0~1)
                                         *   输出相位 = dma下标/N + s_tab_phase
                                         *   重铺表时忘了加它就会跳一下，见 signal_apply 步骤 0 */

/**
  * @brief  求 k = max{ k | 10*2^(k-1) <= freq_hz }，纯整数运算，避免引入 math.h
  * @param  freq_hz 目标频率（必须是四舍五入后的值，保证与频率公式自洽）
  * @retval 频段号 k，已钳位到 [1, SIGNAL_BANDS]
  * @note   用 thr=10*2^(k-1) 逐步翻倍试探，k<=10 时 thr 最大 5120，不会溢出。
  */
static uint32_t band_index(float freq_hz)
{
    uint32_t k = 1U;
    uint32_t thr = 10U;                 /* thr = 10 * 2^(k-1) */

    while ((k < SIGNAL_BANDS) && (freq_hz >= (float)(thr << 1)))
    {
        thr <<= 1;
        k++;
    }
    return k;
}

/**
  * @brief  把 s_psc / s_arr / s_npts 写进硬件（换频、重启 DMA 时调用）
  *
  * ★★ 两个实测踩出来的大坑，改动前务必读完 ★★
  *
  * 坑 1（会导致死锁、完全没波形）：
  *   不要用 HAL_DAC_Stop_DMA()。在 DMA 正在循环搬运时调它，内部的
  *   HAL_DMA_Abort 会一直等 DMA 的 EN 位清零，等不到就死在超时循环里；
  *   结果是 TIM6 停在 CEN=0、DAC 恒定输出正弦表第 0 点（1.65V 直流），
  *   而且事后连 HAL_Delay 都返回不了。
  *   → 改为直接操作寄存器：清 DAC_CR_DMAEN1、清 DMA_CCR_EN、清标志。
  *
  * 坑 2（会导致输出被截断、只看到 1/4 个正弦，最隐蔽）：
  *   不要在这里调用 HAL_DAC_Start() / HAL_DAC_ConfigChannel()。
  *   每次换频都重调 HAL_DAC_Start() 会把 DAC 内部状态机搞乱，
  *   表现为输出级行为异常：满摆时波形只剩 1/4 个正弦（幅度被削掉一半），
  *   把幅度压到 25% 才勉强正常 —— 很容易误判成"输出缓冲到不了轨到轨"。
  *   → DAC 的启动与通道配置**只在 Signal_Init() 里做一次**，
  *     换频时只动 TIM6(频率) 和 DMA(表长度)，绝不重配 DAC。
  */
static void signal_apply(void)
{
    DMA_Channel_TypeDef *ch = NULL;
    float    phase0 = 0.0f;             /* 本刻的输出相位（单位"周期"） */

    if (s_running == 0U)
    {
        return;                         /* 还没启动过，交给 Signal_Init 里的 SetFreq 去做 */
    }

    if (hdac3.DMA_Handle1 != NULL)
    {
        ch = (DMA_Channel_TypeDef *)hdac3.DMA_Handle1->Instance;
    }

    /* ==================== ★★ 快路径：点数没变，就只动定时器 ★★ ====================
     *
     * 为什么必须要有这条快路径（2026-10-12 实测定位）：
     *
     *   走下面的慢路径时，每次换频都要 sine_fill_n() 重铺整张表，
     *   而表里每个点都要调一次 sinf()。N=1022 时这一步要跑约 0.5ms，
     *   期间 TIM6 是停的、DMA 是停的 → **DAC 输出被冻在最后一个值上**。
     *
     *   扫频时无所谓（1.5 秒才换一次档）。但频率跟踪每秒会调几十次
     *   Signal_SetFreq()，等于每秒有几十毫秒输出是"死"的。
     *   自监控时测量窗口只有 1.02ms（512 点 @500kSPS），经常整窗落在冻结点上，
     *   于是峰峰值骤降到几百个码值 → 判定"没信号" → 触发重置 → 又强制换频 → 又冻结，
     *   形成第二个正反馈。实测现象就是 pk 在 220 和 4095 之间乱跳、resets 每秒涨 125。
     *
     *   关键观察：**只要点数 N 没变，表的内容就一个字都不用改** ——
     *       f = f_TIM / ((PSC+1) * (ARR+1) * N)
     *   频率只跟 PSC/ARR 有关。DMA 一直在循环搬同一张表，相位天然就是连续的，
     *   DMA 通道、CMAR、CNDTR **统统不用碰**。
     *   所以这条路径只重设 TIM6 的 PSC/ARR，耗时几微秒，输出几乎没有空档。
     *
     *   代价：停表/重启会截断当前这个 TIM6 周期，相当于输出相位偏了不到
     *   1/N 个周期（N=1022 时是 0.1%），肉眼看不出；
     *   换来的是"换频不再冻结输出"，这个交换非常值。 */
    if ((ch != NULL) && (s_live_n > 0U) && (s_npts == s_live_n) && (s_dirty == 0U))
    {
        __HAL_TIM_DISABLE(&htim6);
        __HAL_TIM_SET_PRESCALER(&htim6, s_psc);
        __HAL_TIM_SET_AUTORELOAD(&htim6, s_arr);
        __HAL_TIM_SET_COUNTER(&htim6, 0U);
        htim6.Instance->EGR = TIM_EGR_UG;
        __HAL_TIM_ENABLE(&htim6);
        return;
    }

    /* 0) ★ 动任何寄存器之前，先把"当前输出相位"取出来 ★
     *    DMA 循环模式下 CNDTR 从 s_live_n 递减到 0 再回绕，所以
     *        已传输点数 = s_live_n - CNDTR
     *    就是"下一次要搬的表下标"，也就是此刻在本表里的位置。
     *
     *    ★ 别漏了 s_tab_phase ★
     *      输出相位 = 表内位置/N + 表自身烘焙的相位偏移。
     *      只算 done/N 的话，等于假设表是从相位 0 开始的 ——
     *      而重铺过的表并不是。漏掉它的后果是每次重铺都跳一下，
     *      幅度最大可达满量程，正是我们要消掉的那个台阶。 */
    if (s_phaselock != 0U)
    {
        /* ★ 锁相模式：表头必须恒等于"相位 0"，不能做相位连续 ★
         *   因为 Signal_Realign() 每次把 DMA 挪回表头时，都假定表头就是
         *   波形的上升沿位置。若这时表里烘焙着任意相位，复位后的相位就乱了。 */
        phase0 = s_phase_off;
    }
    else
    {
        if ((ch != NULL) && (s_live_n > 0U) && (ch->CNDTR <= s_live_n))
        {
            uint32_t done = s_live_n - ch->CNDTR;

            if (done < s_live_n)
            {
                phase0 = (float)done / (float)s_live_n;
            }
        }
        phase0 += s_tab_phase;          /* 加上表里已有的相位偏移 */
    }

    /* 1) 先掐掉触发源，避免改配置的过程中有半截触发进来 */
    __HAL_TIM_DISABLE(&htim6);

    /* 2) 关掉 DAC 的 DMA 请求，并直接清 DMA 通道的 EN 位复位数据流。
     *    这里刻意绕开 HAL_DAC_Stop_DMA()，见坑 1。 */
    DAC3->CR &= ~DAC_CR_DMAEN1;
    if (ch != NULL)
    {
        ch->CCR &= ~DMA_CCR_EN;
        DMA1->IFCR = 0x0FUL;            /* 清通道1 全部中断标志 */
    }
    DAC3->SR = DAC_SR_DMAUDR1;          /* 清 DMA 欠载标志，避免卡住后续传输 */

    /* 2.5) ★ 按刚才取到的相位重铺正弦表 ★
     *   必须在这里（DMA 已停、表还没被引用）重铺，而且**每次换频都要铺**，
     *   不能只在 N 变化时铺 —— 因为相位每刻都在变。
     *   代价是每次换频多 N 次 sinf()（N 最大 1024，约几十微秒），
     *   换来的是换频时输出**没有任何台阶**，这个交换在跟频场景下非常值。 */
    wave_fill_n(s_npts, phase0);
    s_live_n = s_npts;
    s_dirty  = 0U;

    /* 把归一化后的相位记下来，下次重铺要加上它（见步骤 0） */
    s_tab_phase = phase0 - (float)((int32_t)phase0);
    if (s_tab_phase < 0.0f)
    {
        s_tab_phase += 1.0f;
    }

    /* 3) 改 TIM6 的 PSC/ARR，并手动发一次 UG 事件把影子寄存器刷新成新值。
     *    必须停表再改，否则 PSC 是带缓冲的，会出现一整个周期的错拍。 */
    __HAL_TIM_SET_PRESCALER(&htim6, s_psc);
    __HAL_TIM_SET_AUTORELOAD(&htim6, s_arr);
    __HAL_TIM_SET_COUNTER(&htim6, 0U);
    htim6.Instance->EGR = TIM_EGR_UG;

    /* 4) 重设 DMA 通道：目的地址 + 源地址 + 传输长度，然后启动通道。
     *    传输长度 = s_npts，所以正弦表只有前 s_npts 个点被使用；
     *    取 16/32/64/256/512/1024 这些 1024 的约数，保证相位首尾严格相接。 */
    if (ch != NULL)
    {
        ch->CPAR  = (uint32_t)&(DAC3->DHR12R1);
        ch->CMAR  = (uint32_t)sine_tab;
        ch->CNDTR = s_npts;
        ch->CCR  |= DMA_CCR_EN;

        /* 循环模式下 DMA 不需要 CPU 参与，关掉它的中断，免得每个半周期都白跑
         * 一次 IRQ（采样率最高 10MHz，半传输中断会吃掉大量 CPU）。
         * hdac3.DMA_Handle1 就是 dac.c 里 __HAL_LINKDMA 挂上来的 hdma_dac3_ch1，
         * 本身就是 DMA_HandleTypeDef*，所以直接传、不要再取地址。 */
        __HAL_DMA_DISABLE_IT(hdac3.DMA_Handle1, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);
    }

    /* 5) 预置首点，再使能 DMA 请求，最后放开时钟。
     *    ★ 因为表已经按"当前相位"重铺（见步骤 0 / 2.5），sine_tab[0] 恰好等于
     *      输出此刻的值，所以这次预置**不产生台阶** —— 正是靠这一点，
     *      自监控闭环才不会因为换频台阶多数出沿而失控。
     *    注意这里**不碰 HAL_DAC_Start()** —— 见坑 2。 */
    DAC3->DHR12R1 = sine_tab[0];
    DAC3->CR |= DAC_CR_DMAEN1;
    __HAL_TIM_ENABLE(&htim6);
}

/**
  * @brief  为给定频率挑一组 (N, ARR+1)：波形尽量平滑 + 频率尽量准
  *
  * 【为什么要扫】
  *   输出频率 f = f_TIM / ((PSC+1)*(ARR+1)*N)，三个量都必须是整数，
  *   而目标频率是连续的浮点数 —— 大多数频率**凑不出精确值**。
  *   只固定 N、只四舍五入 ARR 的话，误差最大能到 f/(ARR+1)：
  *       50kHz 固定 100 点时 (ARR+1)=34  →  最坏误差 1.5%
  *       实测 30kHz 就偏成了 29825（0.58%）
  *
  *   但 N 是可以自由选的（每次换频都会重铺表），所以把 N 也拿来凑：
  *   在 [4, N_max] 里扫一遍，对每个 N 算出最近的可达频率，
  *   取误差最小的那一组。乘积 (ARR+1)*N 越接近 f_TIM/f 就越准。
  *
  * 【为什么从大到小扫】
  *   N 越大波形越平滑，所以优先保留大 N：
  *   一旦误差已经 ≤0.05% 就立刻停手，不再为了那点精度牺牲波形。
  *
  * @param  freq_hz  目标频率（已四舍五入到 0.1Hz）
  * @param  n_max    N 的上限（= min(SIN_MAX_POINTS, DAC_MAX_RATE/f)）
  * @param  out_n    输出：选中的点数
  * @param  out_prd  输出：选中的 (ARR+1)
  */
static void signal_pick_n(float freq_hz, uint32_t n_max,
                          uint32_t *out_n, uint32_t *out_prd)
{
    uint32_t n;
    uint32_t prd;
    uint32_t best_n   = 4U;
    uint32_t best_prd = 1U;
    float    best_err = 1.0e9f;

    if (n_max < 4U)
    {
        n_max = 4U;
    }
    if (n_max > SIN_MAX_POINTS)
    {
        n_max = SIN_MAX_POINTS;
    }

    /* ==================== ★ 先试"当前正在用的 N"，能沿用就沿用 ★ ====================
     *
     * 为什么必须加这一条（2026-10-12 实测定位）：
     *   换 N 会重铺整张表，而正弦表每个点都要一次 sinf()，
     *   N=1022 时这一步约 1ms，期间 TIM6 停着、输出被冻结。
     *   频率跟踪时 s_out_hz 一直在小幅抖动（低通滤波的残差），
     *   下面那个"扫出误差最小的 N"就会跟着反复改 N ——
     *   实测每秒重铺几十次，主循环被拖过 4.1ms，
     *   于是 DMA 套圈丢样本（g_trk_laps 从 0 涨到 53/s），频率跟着算错。
     *
     *   所以：如果当前这个 N 凑出来的频率已经够准（误差 ≤0.1%），就直接沿用，
     *   不为了一点点精度去付整张表重铺的代价。
     *   0.1% 对"同频"完全够用，而重铺导致的丢样本才是真正致命的。 */
    if ((s_live_n >= 4U) && (s_live_n <= n_max))
    {
        float    target   = (float)TIM6_CLK_HZ / (freq_hz * (float)s_live_n);
        uint32_t keep_prd = (uint32_t)(target + 0.5f);

        if ((keep_prd >= 2U) && (keep_prd <= 65536U))
        {
            float fa  = (float)TIM6_CLK_HZ / ((float)keep_prd * (float)s_live_n);
            float err = fabsf(fa - freq_hz) / freq_hz;

            if (err <= 0.001f)              /* 0.1% 以内 → 沿用，不重铺表 */
            {
                *out_n   = s_live_n;
                *out_prd = keep_prd;
                return;
            }
        }
    }

    for (n = n_max; n >= 4U; n--)
    {
        float target = (float)TIM6_CLK_HZ / (freq_hz * (float)n);
        float fa;
        float err;

        prd = (uint32_t)(target + 0.5f);
        if (prd < 2U)
        {
            continue;                   /* 触发太快，定时器做不到 */
        }
        if (prd > 65536U)
        {
            continue;                   /* 16 位定时器装不下 */
        }

        fa  = (float)TIM6_CLK_HZ / ((float)prd * (float)n);
        err = (fa > freq_hz) ? (fa - freq_hz) : (freq_hz - fa);
        err = err / freq_hz;

        if (err < best_err)
        {
            best_err = err;
            best_n   = n;
            best_prd = prd;
        }
        if (err <= 0.0005f)             /* ≤0.05% 就够了，保住这个大 N */
        {
            break;
        }
    }

    *out_n   = best_n;
    *out_prd = best_prd;
}

/* ============================= 对外接口 ============================= */

void Signal_Init(void)
{
    if (s_running != 0U)
    {
        return;                         /* 防止重复初始化 */
    }
    s_running = 1U;
    wave_fill_n(SIN_MAX_POINTS, 0.0f);  /* 先铺一张满表（相位 0），SetFreq 里会按 N 重铺 */
    s_psc     = 0U;
    s_arr     = 0U;
    s_npts    = 1024U;
    s_live_n  = 0U;                     /* DMA 还没跑过，相位视为 0 */

    /* ★ 这里**不要**去改 DAC3->MCR。实测结论（2026-10-12）：
     *   CubeMX 生成的 dac.c 里是
     *       DAC_OutputBuffer            = DISABLE
     *       DAC_ConnectOnChipPeripheral = INTERNAL
     *   也就是 MODE1 = 0b011。我一度以为"缓冲禁用"是高频塌陷的原因，
     *   手动改成 0b001（缓冲使能），结果内部通路直接断掉（只剩 98mV 耦合）。
     *   后来查 ST 官方 OPAMP_InternalFollower 例程的 MX_DAC3_Init()，
     *   **用的正是 OUTPUTBUFFER_DISABLE + CHIPCONNECT_INTERNAL** —— 原配置就是对的。
     *   真正的病根是正弦表只铺了 1024 点却按 N 个点播放（见 sine_fill_n）。 */

    /* ★ DAC 的启动与通道配置**全程只做这一次**。
     *   后面每次换频都只动 TIM6 和 DMA，绝不再调 HAL_DAC_Start() ——
     *   重复启动会让 DAC 状态机错乱、输出被截断（见 signal_apply 的坑 2）。 */
    (void)HAL_DAC_Start(&hdac3, DAC_CHANNEL_1);

    Signal_SetFreq(1000.0f);            /* 默认输出 1kHz 正弦 */
}

void Signal_SetFreq(float freq_hz)
{
    uint32_t k;
    uint32_t psc;
    uint32_t npts;
    uint32_t prd;                       /* = ARR + 1 */

    /* 钳位到支持范围 */
    if (freq_hz < SIGNAL_FREQ_MIN)
    {
        freq_hz = SIGNAL_FREQ_MIN;
    }
    if (freq_hz > SIGNAL_FREQ_MAX)
    {
        freq_hz = SIGNAL_FREQ_MAX;
    }

    /* 关键：先把目标频率四舍五入到 0.1Hz，再据此选频段。
     * 不能用整数频率 —— 低频段 1Hz 的量化就是 2.4% 误差（20.5Hz 会被
     * 硬拉成 21Hz）。0.1Hz 网格对任何频段都远细于 ARR 本身的步进。 */
    freq_hz = (float)((uint32_t)(freq_hz * 10.0f + 0.5f)) * 0.1f;

    /* 选频段 */
    k    = band_index(freq_hz);
    psc  = band_psc[k - 1U];
    npts = band_n[k - 1U];

#if (SIGNAL_ADAPTIVE_N == 0)
    /* ---- 参考工程方案：全频段固定点数，只用 TIM6 改频率 ----
     * 参考工程 test4_prj 就是 dots=100 定死，然后改 htim7 的 PSC/Period。
     * PSC 固定 0（不预分频，让 ARR 尽量大，频率分辨率好一点）。 */
    psc  = 0U;
    npts = SIGNAL_FIXED_N;
    prd  = (uint32_t)(((float)TIM6_CLK_HZ / (freq_hz * (float)npts))
                      / (float)(psc + 1U) + 0.5f);
    if (prd < 1U)    { prd = 1U; }
    if (prd > 65536U){ prd = 65536U; }
#elif (SIGNAL_ADAPTIVE_N == 2)
    /* ---- 推荐方案：N 取 DAC 更新率允许的最大值，再扫描凑准频率 ---- */
    psc = 0U;
    {
        uint32_t n_max;
        uint32_t raw = (uint32_t)((float)SIGNAL_DAC_MAX_RATE / freq_hz);

        n_max = (raw < 4U) ? 4U : raw;          /* 至少 4 点/周期 */
        signal_pick_n(freq_hz, n_max, &npts, &prd);
        (void)k;
    }
#else
    /* ---- 原方案：按频段表取点数 ---- */
    prd = (uint32_t)(((float)TIM6_CLK_HZ / (freq_hz * (float)npts))
                     / (float)(psc + 1U)
                     + 0.5f);
    if (prd < 1U)
    {
        prd = 1U;
    }
    if (prd > 65536U)                   /* 16 位定时器的物理上限 */
    {
        prd = 65536U;
    }
#endif

    /* (ARR+1) = f_TIM / (f * N * (PSC+1))，四舍五入。
     * 必须四舍五入而不是截断：20kHz 时差 1 个 tick 就是 75Hz 的偏差。 */
    prd = (uint32_t)(((float)TIM6_CLK_HZ / (freq_hz * (float)npts))
                     / (float)(psc + 1U)
                     + 0.5f);
    if (prd < 1U)
    {
        prd = 1U;
    }
    if (prd > 65536U)                   /* 16 位定时器的物理上限 */
    {
        prd = 65536U;
    }

    s_psc  = psc;
    s_arr  = prd - 1U;
    s_npts = npts;

    /* ★ 铺表的活儿已经挪到 signal_apply() 里了 ★
     *   原来这里写的是"点数变了才重铺"，现在不够用了 ——
     *   要做到换频无台阶，必须**每次都按当前相位重铺**，
     *   而相位只有在 signal_apply() 里读 DMA 的 CNDTR 才拿得到。
     *   见 signal_apply() 的步骤 0（取相位）和 2.5（重铺）。 */
    signal_apply();
}

float Signal_GetFreq(void)
{
    float f_samp = (float)TIM6_CLK_HZ
                 / (float)(s_psc + 1U)
                 / (float)(s_arr + 1U);
    return f_samp / (float)s_npts;
}

uint32_t Signal_GetPoints(void)
{
    return s_npts;
}

uint32_t *Signal_GetTable(void)
{
    return sine_tab;
}

/* ==================== 多波形 / 电平 / 相位 接口（2026-10-12 加）====================
 *
 * 这一组是为了"跟频 + 同波形 + 同电压 + 同相位"服务的：
 *   波形  -> Signal_SetWave()    方波/正弦/三角/锯齿
 *   电压  -> Signal_SetLevels()  直接把对方实测的低/高电平码值搬过来
 *   相位  -> Signal_SetPhaseLock() + Signal_Realign()  （由输入上升沿触发）
 *
 * 每个 setter 都遵循同一套流程：
 *   参数真的变了 → 置 s_dirty → 立刻 signal_apply()
 * signal_apply() 看到 s_dirty 就会走慢路径重铺表；s_dirty==0 且点数没变时走快路径，
 * 只动 TIM6 的 PSC/ARR（几微秒，不冻结输出）。
 */

void Signal_SetWave(uint8_t mode)
{
    if (s_wave_mode != mode)
    {
        s_wave_mode = mode;
        s_dirty     = 1U;
        if (s_running != 0U)
        {
            signal_apply();
        }
    }
}

void Signal_SetLevels(uint32_t vmin, uint32_t vmax)
{
    uint32_t t;

    if (vmin > 4095U) { vmin = 4095U; }
    if (vmax > 4095U) { vmax = 4095U; }
    if (vmin > vmax)                            /* 容错：传反了也认 */
    {
        t    = vmin;
        vmin = vmax;
        vmax = t;
    }

    if ((s_vmin != vmin) || (s_vmax != vmax))
    {
        s_vmin  = vmin;
        s_vmax  = vmax;
        s_dirty = 1U;
        if (s_running != 0U)
        {
            signal_apply();
        }
    }
}

void Signal_SetDuty(float duty)
{
    if (duty < 0.01f) { duty = 0.01f; }
    if (duty > 0.99f) { duty = 0.99f; }

    if (fabsf(duty - s_duty) > 0.0005f)
    {
        s_duty  = duty;
        s_dirty = 1U;
        if (s_running != 0U)
        {
            signal_apply();
        }
    }
}

void Signal_SetPhaseDeg(float deg)
{
    float p = deg * (1.0f / 360.0f);

    p = p - (float)((int32_t)p);
    if (p < 0.0f)
    {
        p += 1.0f;
    }

    if (fabsf(p - s_phase_off) > 0.0002f)
    {
        s_phase_off = p;
        s_dirty     = 1U;
        if (s_running != 0U)
        {
            signal_apply();
        }
    }
}

void Signal_SetPhaseLock(uint8_t on)
{
    uint8_t v = (on != 0U) ? 1U : 0U;

    if (s_phaselock != v)
    {
        s_phaselock = v;
        s_dirty     = 1U;
        if (s_running != 0U)
        {
            signal_apply();
        }
    }
}

uint8_t Signal_GetWave(void)
{
    return s_wave_mode;
}

void Signal_Realign(void)
{
    DMA_Channel_TypeDef *ch;

    if ((s_running == 0U) || (s_live_n == 0U) || (hdac3.DMA_Handle1 == NULL))
    {
        return;
    }
    ch = (DMA_Channel_TypeDef *)hdac3.DMA_Handle1->Instance;

    /* ---- 1) 停表：此刻起输出冻结，到重新起振大约 1~2us ---- */
    __HAL_TIM_DISABLE(&htim6);

    /* ---- 2) 关 DMA 请求 + 通道，清标志（绕开 HAL_DAC_Stop_DMA，见 signal_apply 坑 1）---- */
    DAC3->CR &= ~DAC_CR_DMAEN1;
    ch->CCR  &= ~DMA_CCR_EN;
    DMA1->IFCR = 0x0FUL;                        /* 只清 DMA1 通道1 的中断标志 */
    DAC3->SR   = DAC_SR_DMAUDR1;

    /* ---- 3) 指针拉回表头。表头 = 相位0 = 上升沿，见 SIG_WAVE_xxx 的约定 ---- */
    ch->CPAR  = (uint32_t)&(DAC3->DHR12R1);
    ch->CMAR  = (uint32_t)sine_tab;
    ch->CNDTR = s_live_n;
    ch->CCR  |= DMA_CCR_EN;

    DAC3->DHR12R1 = sine_tab[0];                /* 预置成表头值（方波就是高电平） */

    /* ---- 4) ★ 把 CNT 直接设成 ARR，让第一次 Update 在下一个时钟就发生 ★
     *   如果这里老老实实 CNT=0，第一次触发要等 (ARR+1) 个时钟 = 一个 DAC 更新周期
     *   （N=1024 时约 6us），那就成了一个固定的相位滞后，白白浪费精度。
     *   设成 ARR 后，下一个 tick(5.9ns) 就溢出 → 产生 TRGO → DAC 立刻输出表头点，
     *   所以"检测到对方的沿"到"我们的沿"之间的延迟基本只剩下中断响应时间。 */
    htim6.Instance->EGR = TIM_EGR_UG;           /* 先把 PSC/ARR 影子寄存器刷新成新值 */
    __HAL_TIM_SET_COUNTER(&htim6, s_arr);       /* 再顶到 ARR，准备好立刻溢出 */
    DAC3->CR |= DAC_CR_DMAEN1;
    __HAL_TIM_ENABLE(&htim6);
}
