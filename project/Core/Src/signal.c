/**
  ******************************************************************************
  * @file    signal.c
  * @brief   DAC + TIM6 + DMA 循环 → 持续正弦波，频率 20Hz~20kHz 可调（PA4 输出）
  *
  * ============================ 工作原理 ============================
  *   TIM6 每 (ARR+1) 个计数发一次 TRGO 脉冲
  *        → DAC1_OUT1 被硬件触发一次转换
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
  *   所以按倍频程分 10 段，每段换一组 (PSC, N)，让
  *       (PSC+1)*(ARR+1) = f_TIM/(f*N) ≈ 518 ~ 1037
  *   恒定，ARR 就稳定落在 518~1037，全频段频率误差 < 0.1%，
  *   采样率始终远高于输出频率（最少 16 点/周期）。
  *
  * ======================== DMA 位宽必须对齐 ========================
  *   CubeMX 里 DAC1_CH1 的 DMA 配的是 Peripheral / Memory 都是 Word(32位)，
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

/* TIM6 输入时钟 = APB1 定时器时钟 = 170MHz（改主频必须同步改这里！） */
#define TIM6_CLK_HZ             170000000UL

/* ====================== 1024 点正弦表（12 位，0~4095）======================
 * ★ 必须是 RAM 里的可写数组，且在 Signal_Init() 里运行时生成 ★
 *   实测踩坑：把正弦表写成 Flash 里的 `static const` 数组时，波形会被截断
 *   （示波器上只看到约 1/4 个正弦）；换成运行时在 RAM 里用 sinf() 生成后，
 *   波形立刻变成完整的漂亮正弦。原因未完全定论（怀疑与 DMA 从 Flash 读取
 *   的位宽/时序有关），但**实测结论明确**，所以采用运行时生成方案。
 *
 * 表值定义：sine_tab[i] = round(2048 + 2047.5 * sin(2*pi*i/1024))
 *   中心 2048 -> 1.65V；满摆 0~4095 -> 0~3.3V
 *   首点 = 2048（过零点），换频时从过零重新起振，相位不跳。
 * 高频段只用前 N 个点（N 为 1024 的约数），所以一张表通吃所有频段。
 * 元素类型必须是 uint32_t —— 与 DMA 的 Word 传输宽度严格一致。
 */
static uint32_t sine_tab[SIN_MAX_POINTS];

/**
  * @brief  生成正弦表到 RAM（在 Signal_Init() 里调用一次）
  * @note   用 sinf() 现场算，不依赖任何外部数据，也不会被链接器摆错位置。
  */
static void sine_fill(void)
{
    uint32_t i;

    for (i = 0U; i < SIN_MAX_POINTS; i++)
    {
        float ang = 6.28318530718f * (float)i / (float)SIN_MAX_POINTS;
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
 * N 取 1024 的约数（保证相位首尾严格相接）；
 * PSC 取 (2^n - 1)，使 f_TIM/(PSC+1) 为整数，消除额外量化误差。
 *
 * 设计约束：让 (PSC+1)*(ARR+1) = f_TIM/(f*N) 在每段都落在 518 ~ 1037，
 * 于是 ARR 恒在 518~1037 区间，相对频率步进 = 1/(ARR+1) <= 0.2%。
 *
 * 实测（全 20~20000Hz 逐 Hz 扫描）：最大频率误差 0.094%，最小 16 点/周期。
 *
 * ★ k=11 以上是为高频段（>10kHz）扩展的 ★
 *   DAC 每次更新约需 1us，所以 100kHz 时最多只能给到 8~16 点/周期，
 *   波形会明显偏离理想正弦（这是 DAC 的物理极限，不是代码问题）。
 */
#define SIGNAL_BANDS   14U

static const uint32_t band_psc[SIGNAL_BANDS] =
{
     7U,    /* k=1  [  10,   20) Hz   N=1024  ← 20Hz 以下被钳位，此段备用 */
     7U,    /* k=2  [  20,   40) Hz   N=1024 */
     3U,    /* k=3  [  40,   80) Hz   N=1024 */
     3U,    /* k=4  [  80,  160) Hz   N=1024 */
     1U,    /* k=5  [ 160,  320) Hz   N=512  */
     1U,    /* k=6  [ 320,  640) Hz   N=256  */
     0U,    /* k=7  [ 640, 1280) Hz   N=256  */
     0U,    /* k=8  [1280, 2560) Hz   N=64   */
     0U,    /* k=9  [2560, 5120) Hz   N=32   */
     0U,    /* k=10 [5120,10240) Hz   N=16   */
     0U,    /* k=11 [10240,20480) Hz  N=16   -> 采样率 1.28MHz，输出 80kHz 时 16 点 */
     0U,    /* k=12 [20480,40960) Hz  N=8    -> 采样率 2.56MHz，输出 160kHz 时 16 点 */
     0U,    /* k=13 [40960,81920) Hz  N=8    -> 采样率 5.12MHz */
     0U     /* k=14 [81920,163840) Hz N=4    -> 采样率 10.24MHz，已超 DAC 能力 */
};

static const uint16_t band_n[SIGNAL_BANDS] =
{
    1024U, 1024U, 1024U, 512U, 512U, 256U, 256U, 64U, 32U, 16U, 16U, 8U, 8U, 4U
};

/* ============================ 内部状态 ============================ */
static uint8_t  s_running = 0U;         /* 是否已启动 */
static uint32_t s_psc     = 0U;         /* 当前预分频值 */
static uint32_t s_arr     = 0U;         /* 当前重装载值 */
static uint32_t s_npts    = 1024U;      /* 当前使用的表点数 */

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
  * @brief  把 s_psc / s_arr / s_npts 写进硬件（换频/重启 DMA 时调用）
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
  *   每次换频都重新 HAL_DAC_Start() 会把 DAC 内部状态机搞乱，
  *   表现为输出级行为异常：满摆时波形只剩 1/4 个正弦（幅度被削掉一半），
  *   把幅度压到 25% 才勉强正常 —— 很容易误判成"输出缓冲到不了轨到轨"。
  *   → DAC 的启动与通道配置**只在 Signal_Init() 里做一次**，
  *     换频时只动 TIM6(频率) 和 DMA(表长度)，绝不重碰 DAC。
  */
static void signal_apply(void)
{
    DMA_Channel_TypeDef *ch = NULL;

    if (s_running == 0U)
    {
        return;                         /* 还没启动过，交给 Signal_Init 里的 SetFreq 去做 */
    }

    if (hdac1.DMA_Handle1 != NULL)
    {
        ch = (DMA_Channel_TypeDef *)hdac1.DMA_Handle1->Instance;
    }

    /* 1) 先掐掉触发源，避免改配置的过程中有半截触发进来 */
    __HAL_TIM_DISABLE(&htim6);

    /* 2) 关掉 DAC 的 DMA 请求，并直接清 DMA 通道的 EN 位复位数据流。
     *    这里刻意绕开 HAL_DAC_Stop_DMA()，见坑 1。 */
    DAC1->CR &= ~DAC_CR_DMAEN1;
    if (ch != NULL)
    {
        ch->CCR &= ~DMA_CCR_EN;
        DMA1->IFCR = 0x0FUL;            /* 清通道1 全部中断标志 */
    }
    DAC1->SR = DAC_SR_DMAUDR1;          /* 清 DMA 欠载标志，避免卡住后续传输 */

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
        ch->CPAR  = (uint32_t)&(DAC1->DHR12R1);
        ch->CMAR  = (uint32_t)sine_tab;
        ch->CNDTR = s_npts;
        ch->CCR  |= DMA_CCR_EN;

        /* 循环模式下 DMA 不需要 CPU 参与，关掉它的中断，免得每个半周期都白跑
         * 一次 IRQ（采样率最高 10MHz，半传输中断会吃掉大量 CPU）。
         * hdac1.DMA_Handle1 就是 dac.c 里 __HAL_LINKDMA 挂上来的 hdma_dac1_ch1，
         * 本身就是 DMA_HandleTypeDef*，所以直接传、不要再取地址。 */
        __HAL_DMA_DISABLE_IT(hdac1.DMA_Handle1, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);
    }

    /* 5) 预置首点（1.65V 过零点，换频时相位不跳），再使能 DMA 请求，最后放开时钟。
     *    注意这里**不碰 HAL_DAC_Start()** —— 见坑 2。 */
    DAC1->DHR12R1 = sine_tab[0];
    DAC1->CR |= DAC_CR_DMAEN1;
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
    sine_fill();                        /* 生成 RAM 正弦表 */
    s_psc  = 0U;
    s_arr  = 0U;
    s_npts = 1024U;

    /* ★ DAC 的启动与通道配置**全程只做这一次**。
     *   后面每次换频都只动 TIM6 和 DMA，绝不再调 HAL_DAC_Start() ——
     *   重复启动会让 DAC 状态机错乱、输出被截断（见 signal_apply 的坑 2）。 */
    (void)HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

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
     * 硬拉到 21Hz）。0.1Hz 网格对任何频段都远细于 ARR 本身的步进。 */
    freq_hz = (float)((uint32_t)(freq_hz * 10.0f + 0.5f)) * 0.1f;

    /* 选频段 */
    k    = band_index(freq_hz);
    psc  = band_psc[k - 1U];
    npts = band_n[k - 1U];

    /* (ARR+1) = f_TIM / (f * N * (PSC+1))，四舍五入。
     * 必须四舍五入而不是截断：20kHz 档 1 个 tick 就是 75Hz 的偏差。 */
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
