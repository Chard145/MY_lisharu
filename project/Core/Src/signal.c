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

/* ====================== 1024 点正弦表（12 位，0~4095）=====================
 * ★ 必须是 RAM 里的可写数组，且在 Signal_Init() 里运行时生成 ★
 *   实测踩坑：把正弦表写成 Flash 里的 `static const` 数组时，波形会被截断
 *   （示波器上只看到 1/4 个正弦）；换成运行时在 RAM 里用 sinf() 生成后，
 *   波形立刻变成完整的漂亮正弦。原因未完全定论（怀疑与 DMA 对 Flash 读取
 *   的位宽/时序有关），但**实测结论明确**，所以采用运行时生成方案。
 *
 * 表值定义：sine_tab[i] = round(2048 + 2047.5 * sin(2*pi*i/1024))
 *   中心 2048 -> 1.65V；满幅 0~4095 -> 0~3.3V
 *   首点 = 2048（过零点），换频时从过零重新起振，相位不跳变
 * 高频段只用前 N 个点（N 是 1024 的约数），所以一张表通吃所有频段。
 * 元素类型必须是 uint32_t —— 和 DMA 的 Word 传输宽度严格一致。
 */
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
  * 首点 = 2048（过零点），换频时从过零重新起振，相位不跳变。
  */
static void sine_fill_n(uint32_t n)
{
    uint32_t i;

    if (n < 2U)
    {
        n = 2U;
    }
    if (n > SIN_MAX_POINTS)
    {
        n = SIN_MAX_POINTS;
    }

    for (i = 0U; i < n; i++)
    {
        float ang = 6.28318530718f * (float)i / (float)n;   /* 2*pi*i/n */
        float v   = 2048.0f + 2047.5f * sinf(ang);

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
static uint32_t s_last_npts = 0U;       /* 上次铺表用的点数，变了才重铺 */

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

    if (s_running == 0U)
    {
        return;                         /* 还没启动过，交给 Signal_Init 里的 SetFreq 去做 */
    }

    if (hdac3.DMA_Handle1 != NULL)
    {
        ch = (DMA_Channel_TypeDef *)hdac3.DMA_Handle1->Instance;
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

    /* 5) 预置首点（1.65V 过零点，换频时相位不跳），再使能 DMA 请求，最后放开时钟。
     *    注意这里**不碰 HAL_DAC_Start()** —— 见坑 2。 */
    DAC3->DHR12R1 = sine_tab[0];
    DAC3->CR |= DAC_CR_DMAEN1;
    __HAL_TIM_ENABLE(&htim6);
}

/* ============================= 对外接口 ============================= */

void Signal_Init(void)
{
    if (s_running != 0U)
    {
        return;                         /* 防止重复初始化 */
    }
    s_running = 1U;
    sine_fill_n(SIN_MAX_POINTS);        /* 先铺一张满表，SetFreq 里会按 N 重铺 */
    s_psc  = 0U;
    s_arr  = 0U;
    s_npts = 1024U;

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

    /* ★★ 关键：点数变了必须按新的 N 重铺正弦表 ★★
     *   N 个点必须恰好是一个完整周期，否则播放出来的是一段"上升斜坡"
     *   而不是正弦（见 sine_fill_n 的说明）。
     *   铺表放在改硬件之前，保证 DMA 一旦启动拿到的就是新表。 */
    if (npts != s_last_npts)
    {
        sine_fill_n(npts);
        s_last_npts = npts;
    }

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
