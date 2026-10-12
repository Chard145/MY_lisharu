/**
  ******************************************************************************
  * @file    adc_vofa.h
  * @brief   ADC 定时器触发采集 -> 打包 -> DMA 发到 VOFA+（示波器波形显示）
  *
  * ============================== 两种显示模式 ==============================
  *   ADC_VOFA_MODE = 1  【波形流模式，默认】把一整窗采样点连续发出去
  *          VOFA+ 里是一个「示波器」：X 轴是时间，Y 轴是电压(V)，能看见正弦形状。
  *          1 个通道，每帧 ADC_VOFA_POINTS 个点，点间隔 = 降采样后的采样周期。
  *
  *   ADC_VOFA_MODE = 0  【摘要模式】每个周期只发 4 个数：实时/最小/最大/频率
  *          数据量极小，但只能看包络，看不出波形形状。留着做备用/排查。
  *
  * ============================== 为什么不能"原样全发" ==============================
  *   20kHz 采样 = 20000 点/s，JustFloat 每点 4 字节 + 1 字节帧尾 = 100KB/s，
  *   而 USART1 @115200 只有约 11.5KB/s —— 差 8.7 倍，必然堵死。
  *   所以本模块用「整窗快照 + 降采样」：
  *     缓冲 2048 点 = 102.4ms 的一整窗，降采样成 256 点发出去，
  *     带宽刚好落在 115200 能承受的范围内。
  *
  * ============================== 降采样倍数是自适应的 ==============================
  *   固定倍数不行：倍数大，700Hz 一个周期只有几个点（画不出形状）；
  *               倍数小，4Hz 只看到零点几个周期（看不出是正弦）。
  *   所以每帧根据 Signal_GetFreq() 现算：让一窗大约覆盖 4 个信号周期。
  *       f=700Hz -> 不降采样，20kS/s，窗口  6.4ms ≈ 4.5 个周期
  *       f=100Hz -> 降 6 倍， 3.3kS/s，窗口 38.4ms ≈ 3.8 个周期
  *       f=  4Hz -> 降 16 倍（缓冲长度上限），1.25kS/s，窗口 102.4ms
  *
  * ============================== 触发对齐（波形才不抖） ==============================
  *   每帧都从"上升沿穿越中值"的那一点开始取，所以波形在屏上是**静止**的。
  *   若信号幅度太小（直流/没接信号），自动退化为不触发，直接从头取。
  *
  * ============================== VOFA+ 设置（照着选） ==============================
  *   1) 串口：选对应 COM 口，波特率与 MCU 一致（115200）
  *   2) 数据格式：JustFloat
  *   3) 通道数：1        ← 波形流模式固定 1
  *   4) 添加控件 -> 波形图（Waveform），把通道 0 拖进去
  *   5) 波形图的显示点数/X 轴长度设成 128（= 每帧点数）：
  *      一帧正好占满一屏，配合触发对齐，波形在屏上是**静止**的，像真示波器。
  *      设成 256/512 也能看，只是屏上会并排显示 2/4 帧。
  *   ⚠️ 数据格式选错（比如选成 FireWater）会显示成一堆乱码符号。
  *   ⚠️ 想看到正弦波，必须把 PB15(ADC) 用杜邦线接到 PA4(DAC 输出)。
  *
  * ============================== 用法 ==============================
  *      Adc_Vofa_Init();        // 放在 MX_ADC2_Init()/MX_TIM7_Init() 之后，调一次
  *      ...
  *      while (1) {
  *          Adc_Vofa_Poll();    // 主循环里反复调
  *      }
  *
  * 注意：本模块不改任何 CubeMX 生成的文件，全部逻辑在 adc_vofa.c 里。
  ******************************************************************************
  */

#ifndef __ADC_VOFA_H
#define __ADC_VOFA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ============================== 模式选择 ============================== */
/* 1 = 波形流（示波器显示，默认）   0 = 4 通道摘要 */
#define ADC_VOFA_MODE       1

/* ============================== 传输通路 ==============================
 * ADC_VOFA_LINK = 0 : USART1 + DMA  ->  USB-TTL  ->  VOFA+ 选「串口」
 *                    带宽 ~11.5KB/s(115200)，是唯一瓶颈，所以点数只能给到 128
 *
 * ADC_VOFA_LINK = 1 : SEGGER RTT（走 SWD，不占引脚、不占串口）
 *                    带宽是串口的十几~几十倍，点数可以给到 512
 *                    ⚠️ VOFA+ 读不了 RTT，中间要跑 Port/rtt2tcp.py 这个桥，
 *                       VOFA+ 的「数据接口」要改成「TCP Client」连 127.0.0.1:5005
 *                    ⚠️ 必须全程插着 J-Link，拔了就没有数据（但不死机）
 *
 * 两条路互不影响：切回 0 就完全恢复成原来的串口方案。
 */
#define ADC_VOFA_LINK       1

/* ============================== 可调参数 ============================== */

/* DMA 环形缓冲长度（样本数）。DMA 的 CNDTR 是 16 位，最大 65535，安全。
 * 2048 点 @20kHz = 102.4ms，就是一个"采集窗口"的宽度。
 * ★ 加大它 = 低频信号能看更多周期，代价是 2 字节/点 的 RAM。 */
#define ADC_BUF_LEN         2048U

/* 波形流模式：每帧发多少个点（= VOFA+ 里波形图一帧新增的点数）
 *
 * ★★ 帧字节数 = POINTS × 8 ★★
 *     每个点 = 4 字节 float32 + 4 字节帧尾(0x00 0x00 0x80 0x7F)。
 *     ⚠️ 这里踩过一次坑：帧尾是 4 字节不是 1 字节，按 ×5 算会把 tx_buf 越界，
 *        多出来的字节正好压在后面的 .bss（计数器）上，现象是"数据看着像波形、
 *        但计数器全是垃圾"。改 POINTS 时务必同步复核 tx_buf 的长度。
 *
 *   点数   帧字节    @115200(11.5kB/s)   @RTT(自限速)   每周期点数
 *    64     512        44ms(23 帧/s)        ~2ms          16
 *   128    1024        89ms(11 帧/s)        ~5ms          32   <- 串口通路用这个
 *   512    4096       356ms( 2.8帧/s)      ~20ms         128   <- RTT 通路用这个
 *
 * 想要更流畅：把 CubeMX 里 USART1 波特率提到 921600，帧时间直接 /8。
 * ★ 一帧的原始跨度 = POINTS × 降采样倍数，不能超过 ADC_BUF_LEN。 */
#if (ADC_VOFA_LINK != 0)
#define ADC_VOFA_POINTS     512U
#else
#define ADC_VOFA_POINTS     128U
#endif

/* 一窗希望覆盖几个信号周期（自适应降采样的目标值）。3~5 比较好看 */
#define ADC_VOFA_CYCLES     4U

/* 自适应降采样倍数的上下限。上限 = ADC_BUF_LEN / ADC_VOFA_POINTS */
#define ADC_VOFA_DECIM_MIN  1U
#define ADC_VOFA_DECIM_MAX  (ADC_BUF_LEN / ADC_VOFA_POINTS)

/* 触发对齐开关：1 = 从上升沿开始取（波形静止）  0 = 随意取（波形会滚动） */
#define ADC_VOFA_TRIGGER    1

/* 触发门限：峰峰值小于这个码值就当没有有效信号，不触发（防止直流时乱抖） */
#define ADC_VOFA_TRIG_PK    150U

/* ADC 采样率(Hz)
 * ★ 2026-10-12 从 20kHz 提到 500kHz ★
 *   原因：高频扫频（500Hz~50kHz）用 20kSPS 测，10kHz 只有 2 点/周期，
 *   看到的幅度完全不可信。500kSPS 下最差的 50kHz 也有 10 点/周期。
 *
 *   ★ 本宏必须和 Adc_Vofa_Init() 里给 TIM7 设的 ARR 保持一致 ★
 *     ARR = 170000000 / ADC_VOFA_FS_HZ - 1
 *   改这里的话，Init 里的重配会自动跟着变，不用动 tim.c。
 *
 *   ADC 时间预算：ADC 时钟 = PCLK2/4 = 42.5MHz。
 *   Adc_Vofa_Init() 会把采样时间从 CubeMX 默认的 640.5 周期改成 6.5 周期
 *   （见那里的说明），一次转换 = 6.5+12.5 = 19 周期 = 0.447us，
 *   远小于 500kHz 的 2us 周期，来得及。 */
#define ADC_VOFA_FS_HZ      500000U

/* 摘要模式(ADC_VOFA_MODE=0) 每帧几个通道。VOFA+ 里要设成一样的值 */
#define ADC_VOFA_NCH        4U

/* 摘要模式发送周期（毫秒） */
#define ADC_VOFA_PERIOD_MS  10U

/* 参考电压（mV），用于把 12 位码值换算成电压 */
#define ADC_VREF_MV         3300U

/* 输出编码：1 = 文本 FireWater（能用串口助手直接看数字，仅摘要模式用）
 *           0 = 二进制 JustFloat（波形流模式必须用 0） */
#define ADC_VOFA_USE_ASCII  0

/* ============================== 接口 ============================== */

/**
  * @brief  启动 ADC 采集 + DMA + 串口发送链路
  * @note   顺序很重要：先开 DMA，再开节拍器。
  *         放在 MX_ADC2_Init() / MX_TIM7_Init() / MX_USART1_UART_Init() 之后调用。
  */
void Adc_Vofa_Init(void);

/**
  * @brief  轮询：主循环反复调用
  * @note   波形流模式：上一帧没发完就直接返回（发送速率由串口自己限住），
  *         发得完就立刻取新的一窗。
  *         摘要模式：按 ADC_VOFA_PERIOD_MS 限速。
  */
void Adc_Vofa_Poll(void);

/**
  * @brief  取某个缓冲下标的原始码值（0~4095）
  */
uint16_t Adc_Vofa_Raw(uint32_t idx);

/* ============================== 诊断计数（J-Link 可直接读）============================== */
extern volatile uint32_t g_adc_frames;      /* 成功发出的帧数 */
extern volatile uint32_t g_adc_busy;        /* 因上次未发完而跳过的帧数（波形流模式下应持续增长，正常） */
extern volatile uint32_t g_adc_polls;       /* 真正组帧的次数 */
extern volatile uint32_t g_adc_last_raw;    /* 最近一次采到的原始码值 */
extern volatile uint32_t g_adc_last_mv;     /* 最近一次采到的电压(mV) */
extern volatile uint32_t g_adc_min_raw;     /* 本窗最小值 */
extern volatile uint32_t g_adc_max_raw;     /* 本窗最大值 */
extern volatile uint32_t g_adc_decim;       /* 本帧实际用的降采样倍数 */
extern volatile uint32_t g_adc_trig;        /* 本帧触发点下标（0xFFFFFFFF = 未触发） */
extern volatile uint32_t g_adc_frame_len;   /* 本帧字节数 */
extern volatile uint32_t g_adc_freq_hz;     /* 本帧按 Signal_GetFreq() 读到的频率(Hz) */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_VOFA_H */
