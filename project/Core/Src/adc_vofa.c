/**
  ******************************************************************************
  * @file    adc_vofa.c
  * @brief   ADC 定时器触发采集 -> 整窗快照 -> 降采样 -> JustFloat 流 -> VOFA+ 示波器
  *
  * ============================== 数据流 ==============================
  *   TIM7(TRGO=Update) ──触发──> ADC2_IN15(PB15) ──DMA循环──> adc_buf[2048]
  *        20kHz 节拍              12位转换          硬件搬运      102.4ms 一整窗
  *                                                                    │
  *                          每帧：找上升沿触发点 -> 按自适应倍数降采样 ─┘
  *                                                                    │
  *              [float32][帧尾]×N  → 传输通路 → VOFA+ 波形图
  *                                     │
  *                    ADC_VOFA_LINK 二选一：
  *                      0 = USART1+DMA → USB-TTL → VOFA+「串口」
  *                      1 = SEGGER RTT  → SWD    → rtt2tcp.py 桥 → VOFA+「TCP Client」
  *
  * ============================== 为什么这样就能出波形 ==============================
  *   VOFA+ 的 JustFloat 是"按帧"解析的：两个帧尾 0x00 0x00 0x80 0x7F 之间的 float
  *   算作"一帧"，帧里 float 的个数就是通道数。所以：
  *     - 一个帧里塞 N 个 float  -> VOFA+ 认为是 N 个通道，每个通道只有 1 个点，
  *       波形控件画出来是 N 条平线 —— 这不是示波器。
  *     - 一个帧里只放 1 个 float，连续发 N 帧 -> 通道 0 就有 N 个点，X 轴是时间，
  *       画出来才是正弦波。★ 本模块用这一种 ★
  *
  * ============================== 三个必须知道的坑 ==============================
  *   坑 1：DMA 数据宽度必须是 Half Word（16位）
  *     ADC_DR 是 16 位寄存器。若配成 Word，DMA 每次前进 4 字节，
  *     2048 个样本会写下 8192 字节 —— 直接越界踩内存，随机死机。
  *     （对比：DAC 的 DHR12R1 是 32 位寄存器，所以那边必须是 Word。）
  *
  *   坑 2：读 adc_buf 时 DMA 正在改写它
  *     本模块每帧把整窗 2048 点扫两遍（求 min/max、找触发点）+ 取 256 点，
  *     总共几十微秒；这段时间 DMA 只前进 1~2 个样本，最多末端 1~2 个点是新旧混合，
  *     肉眼完全看不出来，所以不做加锁、也不用半传输中断。
  *     若以后要精确算有效值/频率，必须改成半传输中断 + 双缓冲。
  *
  *   坑 3：HAL_UART_Transmit_DMA 必须在 gState == READY 时才调
  *     否则返回 HAL_BUSY，数据根本发不出去（而且容易让人误判成"串口坏了"）。
  *     本模块统一"先查状态、再组帧、最后发"，一份 tx_buf 就够，不需要双缓冲。
  ******************************************************************************
  */

#include "adc_vofa.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "signal.h"        /* Signal_GetFreq()：用来算自适应降采样倍数 */
#include "debug_rtt.h"     /* SEGGER RTT：ADC_VOFA_LINK=1 时的数据出口 */
#include "SEGGER_RTT.h"    /* SEGGER_RTT_GetAvailWriteSpace()：RTT 通路的限速判据 */
#include <string.h>

/* ============================== ADC DMA 回调 ==============================
 * ★ 为什么必须写这两个回调 ★
 *   HAL_ADC_Start_DMA() 会打开 DMA 的传输完成(TC)和半传输(HT)中断。
 *   而 HAL 的 ADC_DMAConvCplt() 里，若检测到 HAL_ADC_ConvCpltCallback 没有被
 *   用户实现，就会**转调 HAL_ADC_ErrorCallback 并置 HAL_ADC_STATE_ERROR_DMA**，
 *   之后 ADC 直接罢工。
 *   所以即使本方案不依赖回调，也必须把这两个弱函数实现掉（空函数即可）。
 *
 *   中断频率 = 采样率 / 缓冲长度 = 20000 / 2048 ≈ 10 Hz —— 开销可忽略。
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
}

/* ============================== 缓冲 ============================== */
static uint16_t adc_buf[ADC_BUF_LEN];              /* DMA 目标：全局、uint16_t、自然对齐 */

/* 发送缓冲。只在上一次 DMA 发完(gState==READY)时才写，所以一份就够。
 *   波形流：每点 = 4 字节 float 电压 + 4 字节 float 频率 + 4 字节帧尾 = 12 字节
 *           ★ 帧尾是 4 字节(0x00 0x00 0x80 0x7F)，不是 1 字节！
 *             按 ×5 算过，结果 tx_buf 越界 768 字节踩掉后面的计数器。
 *           ★ 2026-10-12 加了频率通道后是 ×12，改点数的宏时记得同步复核这里。
 *   摘要  ：4 通道 × 12 字节 + 1       = 49 字节，留 96 足够 */
#if (ADC_VOFA_MODE != 0)
#define ADC_TX_BUF_LEN      (ADC_VOFA_POINTS * 12U)
#else
#define ADC_TX_BUF_LEN      96U
#endif
static uint8_t  tx_buf[ADC_TX_BUF_LEN];

/* ============================== 传输通路抽象 ==============================
 * 把"发一帧"从具体外设里抽出来，上面组帧的代码完全不用关心数据走串口还是走 RTT。
 * 切换只改 adc_vofa.h 里的 ADC_VOFA_LINK 一个宏。
 *
 * 两种通路的"忙"判据完全不同，这点是核心：
 *   串口：看 huart1.gState，只关心"上一次 DMA 发完没有"
 *   RTT ：看上行缓冲还剩多少空间。RTT 是 RAM 里的环形缓冲，J-Link 在后台
 *         通过 SWD 轮询取走，所以"剩余空间够不够放整帧"天然就是限速条件。
 *         ★ 好处：永不阻塞、永不写半帧。J-Link 没插时空间恒为 0，
 *           表现是"没有数据"而不是"死机" —— 这正是 NO_BLOCK_SKIP 模式的意义。
 */
#if (ADC_VOFA_LINK != 0)
#define ADC_LINK_NAME       "RTT(SWD)"
static int adc_link_ready(uint32_t len)
{
    return (SEGGER_RTT_GetAvailWriteSpace(0) >= len) ? 1 : 0;
}
static int adc_link_send(const uint8_t *p, uint16_t len)
{
    return (Debug_RTT_Write(p, (unsigned)len) == (unsigned)len) ? 1 : 0;
}
#else
#define ADC_LINK_NAME       "USART1"
static int adc_link_ready(uint32_t len)
{
    (void)len;
    return (huart1.gState == HAL_UART_STATE_READY) ? 1 : 0;
}
static int adc_link_send(const uint8_t *p, uint16_t len)
{
    return (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)p, len) == HAL_OK) ? 1 : 0;
}
#endif

/* ============================== 诊断计数 ============================== */
volatile uint32_t g_adc_frames    = 0U;
volatile uint32_t g_adc_busy      = 0U;
volatile uint32_t g_adc_polls     = 0U;
volatile uint32_t g_adc_last_raw  = 0U;
volatile uint32_t g_adc_last_mv   = 0U;
volatile uint32_t g_adc_min_raw   = 0U;
volatile uint32_t g_adc_max_raw   = 0U;
volatile uint32_t g_adc_decim     = 0U;
volatile uint32_t g_adc_trig      = 0U;
volatile uint32_t g_adc_frame_len = 0U;
volatile uint32_t g_adc_freq_hz   = 0U;

/* ============================== 内部状态 ============================== */
static uint32_t s_last_tick = 0U;

/**
  * @brief  启动 ADC 采集 + DMA + 串口发送链路
  */
void Adc_Vofa_Init(void)
{
    uint32_t i;

    g_adc_frames    = 0U;
    g_adc_busy      = 0U;
    g_adc_polls     = 0U;
    g_adc_last_raw  = 0U;
    g_adc_last_mv   = 0U;
    g_adc_min_raw   = 0U;
    g_adc_max_raw   = 0U;
    g_adc_decim     = 0U;
    g_adc_trig      = 0U;
    g_adc_frame_len = 0U;
    g_adc_freq_hz   = 0U;

    for (i = 0U; i < ADC_BUF_LEN; i++)
    {
        adc_buf[i] = 0U;
    }
    memset(tx_buf, 0, sizeof(tx_buf));

    /* ================= 采集链路重配（放在用户文件里，CubeMX 不会覆盖）=================
     *
     * ★ 1) ADC 采样时间：640.5 周期 → 6.5 周期
     *   CubeMX 里默认是 ADC_SAMPLETIME_640CYCLES_5。看起来"采样久一点更准"，
     *   实际是**致命的时间浪费**：
     *       ADC 时钟 = PCLK2/4 = 42.5MHz
     *       一次转换 = 采样 640.5 + 逐次逼近 12.5 = 653 周期 = 15.4us
     *   → 采样率上限只有 65kSPS，而且 500kHz 触发根本来不及，会大范围丢触发。
     *   本工程的信号源是运放输出（低阻抗），6.5 周期足够充满采样电容：
     *       6.5 + 12.5 = 19 周期 = 0.447us   （500kHz 的周期是 2us，余量充足）
     *   ⚠️ 只有信号源阻抗高（几十 kΩ 以上）时才需要长采样时间，这里不是。
     *
     * ★ 2) TIM7 周期：按 ADC_VOFA_FS_HZ 现算，不用去改 CubeMX 的 tim.c
     *       ARR = f_TIM7 / 目标采样率 - 1
     *   f_TIM7 = 170MHz（APB1 = 170MHz，PSC 已配 0）
     */
    {
        ADC_ChannelConfTypeDef sc = {0};

        sc.Channel      = ADC_CHANNEL_15;           /* PB15 */
        sc.Rank         = ADC_REGULAR_RANK_1;
        sc.SamplingTime = ADC_SAMPLETIME_6CYCLES_5; /* 原 640.5 周期，见上 */
        sc.SingleDiff   = ADC_SINGLE_ENDED;
        sc.OffsetNumber = ADC_OFFSET_NONE;
        sc.Offset       = 0U;
        (void)HAL_ADC_ConfigChannel(&hadc2, &sc);

        /* TIM7 的 ARR 在这儿重设，和 ADC_VOFA_FS_HZ 保持一致。
         * 先停表再改，改完发一次 UG 把影子寄存器刷新成新值，最后再启动 —— 
         * 否则 PSC/ARR 的缓冲会让头一个周期长度不对。 */
        __HAL_TIM_DISABLE(&htim7);
        __HAL_TIM_SET_PRESCALER(&htim7, 0U);
        __HAL_TIM_SET_AUTORELOAD(&htim7,
                                 (170000000UL / ADC_VOFA_FS_HZ) - 1UL);
        __HAL_TIM_SET_COUNTER(&htim7, 0U);
        htim7.Instance->EGR = TIM_EGR_UG;
    }

    /* ---- 1) 启动 ADC 的 DMA（循环模式，长度 = 缓冲样本数）----
     * HAL_ADC_Start_DMA() 的形参签名是 uint32_t*，但底层仍按 DMA 通道配置的
     * Half Word 搬运，所以这个强转是安全的（HAL 历史遗留签名）。 */
    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc_buf, ADC_BUF_LEN) != HAL_OK)
    {
        /* 启动失败：多数是 DMA 没配好，留个计数值方便查 */
        g_adc_busy++;
        return;
    }

    /* ---- 2) 最后开节拍器（先起 ADC 再起定时器，避免头几个触发丢失）---- */
    (void)HAL_TIM_Base_Start(&htim7);

    s_last_tick = HAL_GetTick();
}

/**
  * @brief  取某个缓冲下标的原始码值
  */
uint16_t Adc_Vofa_Raw(uint32_t idx)
{
    if (idx >= ADC_BUF_LEN)
    {
        return 0U;
    }
    return adc_buf[idx];
}

#if (ADC_VOFA_MODE != 0)
/* ========================================================================
 *                        波形流模式（示波器显示）
 * ======================================================================== */

/**
  * @brief  根据 DAC 当前频率算降采样倍数，让一窗大约覆盖 ADC_VOFA_CYCLES 个周期
  * @note   f=700Hz -> 1 倍(20kS/s，窗  6.4ms ≈ 4.5 个周期)
  *         f=100Hz -> 6 倍(3.3kS/s，窗 38.4ms ≈ 3.8 个周期)
  *         f=  4Hz -> 16 倍(1.25kS/s，窗 102.4ms，到达缓冲上限)
  */
static uint32_t adc_calc_decim(float f)
{
    uint32_t d;
    float    target;

    if (f < 1.0f)
    {
        f = 1.0f;                       /* 频率还没设/为 0 时按 1Hz 处理 */
    }

    /* 想要的采样率 = 每帧点数 × 频率 ÷ 每窗周期数 */
    target = ((float)ADC_VOFA_POINTS * f) / (float)ADC_VOFA_CYCLES;
    if (target < 1.0f)
    {
        target = 1.0f;
    }

    d = (uint32_t)(((float)ADC_VOFA_FS_HZ / target) + 0.5f);

    if (d < ADC_VOFA_DECIM_MIN) { d = ADC_VOFA_DECIM_MIN; }
    if (d > ADC_VOFA_DECIM_MAX) { d = ADC_VOFA_DECIM_MAX; }
    return d;
}

/**
  * @brief  在缓冲里找一个"上升沿穿越中值"的点做触发
  * @retval 触发点下标；0xFFFFFFFF 表示没找到（幅度太小或全是直流）
  */
static uint32_t adc_find_trigger(uint32_t mid, uint32_t pk)
{
    uint32_t i;

    if (pk < ADC_VOFA_TRIG_PK)
    {
        return 0xFFFFFFFFU;             /* 峰峰值太小，当没有有效信号 */
    }

    for (i = 0U; (i + 1U) < ADC_BUF_LEN; i++)
    {
        if ((adc_buf[i] < mid) && (adc_buf[i + 1U] >= mid))
        {
            return i;
        }
    }
    return 0xFFFFFFFFU;
}

/**
  * @brief  组一帧波形：从触发点开始，按 decim 取 ADC_VOFA_POINTS 个点。
  *
  * ★ 每帧是 2 个通道（2026-10-12 加）★
  *     通道0 = 电压(V)，通道1 = 当前 DAC 设定频率(Hz)
  *   每点 = [float 电压][float 频率][帧尾 0x00 0x00 0x80 0x7F] = 12 字节
  *
  *   为什么要带上频率：扫频测幅度时，必须知道每个采样点对应哪个频点，
  *   否则只能靠"数时间"去对齐扫频步进，一有丢帧就全乱。
  *   频率直接进数据流后，VOFA+ 能看见（做成阶梯线），
  *   自检脚本也能直接按频率分组统计峰峰值 —— 不用猜。
  *   （代价：帧从 8 字节/点涨到 12 字节/点，RTT 通路带宽完全够。）
  *
  * @retval 帧字节数
  */
static uint16_t adc_build_wave(uint32_t start, uint32_t decim)
{
    uint8_t *p = tx_buf;
    uint32_t n = 0U;
    uint32_t idx = start;
    uint32_t i;
    float    freq = (float)g_adc_freq_hz;   /* 本帧内频率恒定，反映当前扫频档位 */

    for (i = 0U; i < ADC_VOFA_POINTS; i++)
    {
        float v = (float)adc_buf[idx] * (ADC_VREF_MV / 1000.0f) / 4095.0f;

        if ((n + 12U) > (uint32_t)ADC_TX_BUF_LEN)
        {
            break;                      /* 保险：绝不越界（改 POINTS 后也不会再踩内存） */
        }

        memcpy(&p[n], &v, 4U);          /* 小端 float32，与 JustFloat 一致 */
        n += 4U;
        memcpy(&p[n], &freq, 4U);       /* 通道1：DAC 设定频率 */
        n += 4U;
        p[n++] = 0x00U;                 /* 帧尾：0x00 0x00 0x80 0x7F（4 字节） */
        p[n++] = 0x00U;
        p[n++] = 0x80U;
        p[n++] = 0x7FU;

        idx += decim;                   /* 环形前进（decim <= 16，减一次必够） */
        if (idx >= ADC_BUF_LEN)
        {
            idx -= ADC_BUF_LEN;
        }
    }
    return (uint16_t)n;
}

/**
  * @brief  轮询：上一帧发完就立刻取新的一窗，发送节奏由串口自己限住
  */
void Adc_Vofa_Poll(void)
{
    uint32_t i;
    uint32_t mn = 0xFFFFFFFFU;
    uint32_t mx = 0U;
    uint32_t last = 0U;
    uint32_t mid;
    uint32_t decim;
    uint32_t start;
    uint16_t len;

    /* ---- 0) 上一帧还没发完？直接返回。这就是天然的限速，不用 HAL_Delay ---- */
    if (!adc_link_ready((uint32_t)ADC_TX_BUF_LEN))
    {
        g_adc_busy++;
        return;
    }

    /* ---- 1) 扫一遍整窗，求最小/最大（也是触发判据和诊断值）---- */
    for (i = 0U; i < ADC_BUF_LEN; i++)
    {
        uint32_t r = (uint32_t)adc_buf[i];

        if (r < mn) { mn = r; }
        if (r > mx) { mx = r; }
        last = r;
    }
    if (mn == 0xFFFFFFFFU) { mn = 0U; }

    g_adc_last_raw = last;
    g_adc_last_mv  = (last * ADC_VREF_MV) / 4095U;
    g_adc_min_raw  = mn;
    g_adc_max_raw  = mx;

    /* ---- 2) 自适应降采样倍数（跟着 DAC 频率走）---- */
    {
        float f = Signal_GetFreq();
        g_adc_freq_hz = (uint32_t)(f + 0.5f);
        decim = adc_calc_decim(f);
    }
    g_adc_decim = decim;

    /* ---- 3) 找触发点，让波形在 VOFA+ 上是静止的 ---- */
    mid   = (mn + mx) / 2U;
    start = 0U;
#if (ADC_VOFA_TRIGGER != 0)
    {
        uint32_t t = adc_find_trigger(mid, (mx - mn));
        if (t != 0xFFFFFFFFU)
        {
            start = t;
            g_adc_trig = t;
        }
        else
        {
            g_adc_trig = 0xFFFFFFFFU;   /* 未触发：从头取，会缓慢滚动 */
        }
    }
#else
    g_adc_trig = 0xFFFFFFFFU;
#endif

    /* ---- 4) 组帧并 DMA 发走 ---- */
    len = adc_build_wave(start, decim);
    g_adc_frame_len = (uint32_t)len;
    g_adc_polls++;

    if (adc_link_send(tx_buf, len))
    {
        g_adc_frames++;
    }
    else
    {
        g_adc_busy++;
    }
}

#else
/* ========================================================================
 *                     摘要模式（4 通道，备用/排查）
 * ======================================================================== */

/**
  * @brief  把一帧数据打包并 DMA 发走
  */
static void adc_send_frame(const float *v)
{
    uint8_t *p = tx_buf;
    uint16_t len;

#if (ADC_VOFA_USE_ASCII != 0)
    /* ---------- 文本模式（FireWater）：好调试 ---------- */
    {
        uint32_t i;
        uint32_t n = 0U;

        for (i = 0U; i < ADC_VOFA_NCH; i++)
        {
            /* 手写定点转字符串：保留 4 位小数，不依赖 printf 的 %f
             * （工程用 newlib，%f 会拉进几 KB 的浮点打印库） */
            int32_t  mv = (int32_t)(v[i] * 10000.0f + ((v[i] >= 0.0f) ? 0.5f : -0.5f));
            int32_t  ip = mv / 10000;
            int32_t  fp = mv % 10000;
            uint32_t div;

            if (mv < 0)
            {
                p[n++] = (uint8_t)'-';
                ip = -ip;
                fp = -fp;
            }

            {
                char     tmp[12];
                uint32_t k = 0U;
                uint32_t u = (uint32_t)ip;
                if (u == 0U)
                {
                    tmp[k++] = '0';
                }
                while ((u > 0U) && (k < sizeof(tmp)))
                {
                    tmp[k++] = (char)('0' + (u % 10U));
                    u /= 10U;
                }
                while (k > 0U)
                {
                    p[n++] = (uint8_t)tmp[--k];
                }
            }

            p[n++] = (uint8_t)'.';

            for (div = 1000U; div > 0U; div /= 10U)
            {
                p[n++] = (uint8_t)('0' + (uint32_t)(fp / (int32_t)div) % 10U);
            }

            p[n++] = (i + 1U < ADC_VOFA_NCH) ? (uint8_t)',' : (uint8_t)'\n';

            if (n >= (sizeof(tx_buf) - 12U))
            {
                break;
            }
        }
        len = (uint16_t)n;
    }
#else
    /* ---------- 二进制模式（JustFloat）：NCH 个 float + 一个帧尾 ---------- */
    memcpy(p, v, ADC_VOFA_NCH * 4U);
    p[ADC_VOFA_NCH * 4U + 0U] = 0x00U;
    p[ADC_VOFA_NCH * 4U + 1U] = 0x00U;
    p[ADC_VOFA_NCH * 4U + 2U] = 0x80U;
    p[ADC_VOFA_NCH * 4U + 3U] = 0x7FU;
    len = (uint16_t)(ADC_VOFA_NCH * 4U + 4U);
#endif

    g_adc_frame_len = (uint32_t)len;
    g_adc_decim     = 0U;
    g_adc_trig      = 0U;

    if (adc_link_send(p, len))
    {
        g_adc_frames++;
    }
    else
    {
        g_adc_busy++;
    }
}

/**
  * @brief  轮询：按 ADC_VOFA_PERIOD_MS 取一窗，算摘要发走
  */
void Adc_Vofa_Poll(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t i;
    uint32_t mn;
    uint32_t mx;
    uint32_t last;
    float    v[ADC_VOFA_NCH];

    if ((now - s_last_tick) < ADC_VOFA_PERIOD_MS)
    {
        return;
    }
    s_last_tick = now;

    if (!adc_link_ready((uint32_t)ADC_TX_BUF_LEN))
    {
        g_adc_busy++;
        return;
    }
    g_adc_polls++;

    mn   = 0xFFFFFFFFU;
    mx   = 0U;
    last = 0U;

    for (i = 0U; i < ADC_BUF_LEN; i++)
    {
        uint32_t r = (uint32_t)adc_buf[i];

        if (r < mn) { mn = r; }
        if (r > mx) { mx = r; }
        last = r;
    }
    if (mn == 0xFFFFFFFFU) { mn = 0U; }

    g_adc_last_raw = last;
    g_adc_last_mv  = (last * ADC_VREF_MV) / 4095U;
    g_adc_min_raw  = mn;
    g_adc_max_raw  = mx;
    g_adc_freq_hz  = (uint32_t)(Signal_GetFreq() + 0.5f);

    /*   通道0：ADC 实时电压(V)     通道1：窗口内最小电压(V)
     *   通道2：窗口内最大电压(V)   通道3：DAC 当前设定频率(Hz) */
    v[0] = (float)last * (float)ADC_VREF_MV / 4095.0f / 1000.0f;
    v[1] = (float)mn   * (float)ADC_VREF_MV / 4095.0f / 1000.0f;
    v[2] = (float)mx   * (float)ADC_VREF_MV / 4095.0f / 1000.0f;
    v[3] = Signal_GetFreq();

    adc_send_frame(v);
}

#endif /* ADC_VOFA_MODE */
