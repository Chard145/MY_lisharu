/**
  ******************************************************************************
  * @file    track.h
  * @brief   频率跟踪（锁频）：ADC 采对方的波 → 测频 → DAC 输出同频正弦
  *
  * ============================== 接线（只有一根线要动）==============================
  *   对方信号 ──> PB15 (ADC2_IN15)      ← 原来这根线接在 PA2 上做自监控，改接对方输出
  *   本机输出 <── PA2  (DAC3→OPAMP1)    ← 不变，接示波器/负载
  *
  *   ★ 不用新增任何引脚、不用改 CubeMX 配置。PB15 本来就是采样脚，
  *     只是采样对象从"自己"变成"对方"。
  *
  * ============================== 工作原理 ==============================
  *   ADC2 由 TIM7 触发、DMA 循环搬进 adc_buf[2048]（500kSPS）。
  *   本模块**增量**扫描这块缓冲（用 DMA 的 CNDTR 定位写入位置），逐点做
  *   「上升沿过零检测 + 线性插值」，把每个上升沿的时刻精确到亚采样点，
  *   再用滑动窗口里最早/最新的上升沿算：
  *
  *       f_in = (上升沿个数 - 1) × fs / (首尾上升沿的时间跨度)
  *
  *   然后经「换频检测 → 一阶低通 → 相对死区」三级处理后交给 Signal_SetFreq()。
  *
  * ============================== 为什么是"过零插值"而不是 FFT ==============================
  *   ① 精度：插值定位单个上升沿的误差在纳秒量级（正弦斜率大），
  *      跨多个周期平均后相对误差 <0.01%，FFT 在 512 点下 Δf 就有 977Hz，没法用。
  *   ② 延迟：FFT 要攒满一窗再做蝶变；本方案只要 3 个上升沿就能出结果。
  *   ③ 对方可能是方波/三角波，取"上升沿"而不是"整周期"，
  *      占空比不对称也不影响（只数上升沿）。
  *
  * ============================== 为什么能跨缓冲回绕累积 ==============================
  *   低频（比如 20Hz）一个周期要 50ms，而 2048 点 @500kSPS 只有 4.1ms，
  *   一窗连一个周期都装不下。所以本模块**不按窗算**，而是维护一个全局样本
  *   计数器 + 环形上升沿时间戳，攒够跨度再算 —— 于是同一个采样率下
  *   20Hz 到 100kHz 都能测，代价只是低频响应慢（这是物理决定的，至少要看到一个周期）。
  *
  * ============================== 无信号时的行为 ==============================
  *   峰峰值小于 TRK_MIN_PK 就认为"没接信号"，**保持当前输出频率不动**，
  *   不会乱跳、不会静音。这样即使忘了接线，DAC 也照常出正弦。
  ******************************************************************************
  */

#ifndef __TRACK_H
#define __TRACK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
  * @brief  初始化跟踪模块
  * @note   ★ 必须在 Adc_Vofa_Init() **之后**调用 —— 本模块复用它的 adc_buf
  *         和已经跑起来的 ADC/DMA 链路，不自己启动采集。
  *         调用前请先用 Signal_Init()（或 Signal_SetFreq()）给出一个初始频率。
  */
void  Track_Init(void);

/**
  * @brief  跟踪主循环：主循环里反复调用
  * @note   每次只处理"上一次调用之后新到的那批样本"，O(1) 摊销，
  *         不阻塞、不等待，可以和 Uart_Echo_Poll()/Adc_Vofa_Poll() 共存。
  */
void  Track_Poll(void);

/* ============================== 读数接口 ============================== */

/** @brief 测到的输入频率(Hz)；没测到时返回 0 */
float Track_GetInFreq(void);

/** @brief 正在输出给 DAC 的频率(Hz) */
float Track_GetOutFreq(void);

/** @brief 输入信号峰峰值(码值，4095 = 3.3V) */
uint32_t Track_GetPk(void);

/** @brief 1 = 输入有有效信号，0 = 没信号（此时输出保持不动） */
uint32_t Track_IsValid(void);

/* ============================== 开环测试模式（标定用）==============================
 *   g_trk_hold_hz >= 1.0f 时进入"锁定输出、只测量"模式：
 *       DAC 固定输出 g_trk_hold_hz，跟踪器**不再**去改输出频率，只报测量值。
 *   这样就能把"检测器准不准"和"闭环稳不稳"分开验证 ——
 *   给一个已知频率，看 in_hz 差多少，误差就是检测器自身的误差。
 *
 *   用法（J-Link 直接写内存，不用重新烧录）：
 *       w4 <地址>, 1000      // 锁 1kHz
 *       w4 <地址>, 10000     // 锁 10kHz
 *       w4 <地址>, 0         // 恢复正常跟踪
 *   地址从 project.map / nm 里查 g_trk_hold_hz。
 *
 *   ⚠️ 自监控（PB15 接回 PA2）时闭环会失控，原因见 track.c 里 g_trk_hold_hz 的注释。
 */
extern volatile float g_trk_hold_hz;

/* ============================== 波形匹配 / 相位对齐 诊断 ==============================
 * 2026-10-12 加：目标是输出与对方"同频 + 同波形 + 同电压 + 同相位"。
 *   g_trk_wave      : 识别出的波形（0=正弦 1=三角 2=方波），自动判据见 track.c
 *   g_trk_sq_ratio  : "贴轨样本比例"×1000 —— 方波≈1000，正弦≈410，三角≈200
 *   g_trk_vmin/vmax : 对方波形的低/高电平码值，**原样**搬给 DAC 当输出上下电平
 *   g_trk_duty_ppm  : 对方方波的占空比（ppm）
 *   g_trk_edges     : ADC 模拟看门狗(AWD) 实际执行相位复位的次数（≈ 输入频率）
 *   g_trk_edge_skip : 历史遗留，EXTI 方案弃用后一直是 0
 *   g_trk_laps      : ★ 因主循环太慢、DMA 整整绕一圈而丢样本的次数，正常**应该一直是 0**
 */
extern volatile uint32_t g_trk_edges;
extern volatile uint32_t g_trk_edge_skip;
extern volatile uint32_t g_trk_wave;
extern volatile uint32_t g_trk_sq_ratio;
extern volatile uint32_t g_trk_duty_ppm;
extern volatile uint32_t g_trk_vmin;
extern volatile uint32_t g_trk_vmax;
extern volatile uint32_t g_trk_laps;

/* ★ 被"假沿校验"丢弃的过零点个数。
 *   假过零点（丢样本 / 读到 DMA 正在写的样本造成的）会让窗口内
 *   "沿数不变但跨度变短" → 频率偏高，而且是成串影响（会在窗口里滞留 20 次测量），
 *   事后中值滤波挡不住，只能在源头丢弃。这个计数就是丢弃了多少个。 */
extern volatile uint32_t g_trk_reject;

/* ★ 原始测量值（只过 5 点中值，未做时间常数低通、未做斜率限制）。
 *   Track_GetInFreq() 返回的是**平滑后**的值，两者会明显不同：
 *   原始值里有离群尖峰，但那些尖峰已经被挡在输出之外。
 *   想看"测频本身干不干净"就读这个；想看"实际用了多少"就读 Track_GetInFreq()。 */
extern volatile uint32_t g_trk_raw_hz;

/* ★ AWD 中断总次数。正常应该正好 = 输入频率 × 2（每周期一次上升沿 + 一次下降沿触发）。
 *   明显偏高 = 有多余中断，会多执行 Signal_Realign() → 输出相位被莫名复位 → 跳相。
 *   历史上就是因为主循环也去写 AWD 窗口、和 ISR 抢寄存器，造成窗口状态反掉。 */
extern volatile uint32_t g_trk_awd_irq;

/* ★ AWD 自愈次数。正常应**一直是 0**。
 *   在涨说明阈值/窗口曾经错位到"AWD 完全收不到中断"的程度
 *   （那样相位就完全没在对齐，但不会有任何报错，只是输出看着一直在跳相）。 */
extern volatile uint32_t g_trk_awd_heal;

/* ============================== 诊断计数（J-Link 可直接读）============================== */
extern volatile uint32_t g_trk_measures;   /* 算出频率结果的次数 */
extern volatile uint32_t g_trk_updates;    /* 真正调用 Signal_SetFreq() 的次数 */
extern volatile uint32_t g_trk_valid;      /* 1 = 输入信号有效 */
extern volatile uint32_t g_trk_cross_n;    /* 当前滑动窗口里的上升沿个数 */
extern volatile uint32_t g_trk_in_hz;      /* 测到的输入频率(Hz) */
extern volatile uint32_t g_trk_out_hz;     /* 输出频率(Hz) */
extern volatile uint32_t g_trk_pk;         /* 输入峰峰值(码值) */
extern volatile uint32_t g_trk_resets;     /* 因丢失信号而清空历史的次数 */
extern volatile uint32_t g_trk_samples;    /* 累计处理过的样本数 */

#ifdef __cplusplus
}
#endif

#endif /* __TRACK_H */
