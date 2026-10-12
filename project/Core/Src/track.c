/**
  ******************************************************************************
  * @file    track.c
  * @brief   频率跟踪：ADC 采对方的波 → 上升沿过零插值测频 → DAC 输出同频正弦
  *
  * 详细原理见 track.h 的模块头注释。这里只讲实现里几个"必须这么写"的点。
  *
  * ========================== 点 1：为什么用 DMA 的 CNDTR 定位新样本 ==========================
  *   adc_buf 是 DMA 循环搬运的目标，主循环和 DMA 是并发的，没有"一窗采完"的信号。
  *   但 DMA 的 CNDTR 从 ADC_BUF_LEN 递减到 0 再回绕，所以
  *       fill = ADC_BUF_LEN - CNDTR        （0..ADC_BUF_LEN）
  *   就是"本轮已经写进去的样本数"，也就是下一个要写的位置。
  *   于是本模块只要记住"上次处理到哪个下标"，就能算出中间新增了多少个样本：
  *       avail = (fill - s_rd + ADC_BUF_LEN) % ADC_BUF_LEN
  *   这样不需要中断、不需要双缓冲，也不会有"等一整窗"的延迟。
  *
  *   ⚠️ 末尾留 TRK_SAFE_MARGIN 个样本不碰：那几个可能正被 DMA 改写，
  *      读到的半新半旧值会产生一个假过零点。留 8 个样本（16us）就够 DMA 走完。
  *
  * ========================== 点 2：为什么阈值要自适应 ==========================
  *   对方的信号可能是：带偏置的正弦、0~3.3V 的方波、幅度只有 100mV 的小信号。
  *   写死一个 1.65V 阈值的话，带偏置的信号会一直"过不了零"，方波会漏掉。
  *   所以用**峰值包络检波**（快攻慢放，见 trk_env_update()）持续估计：
  *       阈值 dc = (包络上沿 + 下沿) / 2      迟滞 hyst = 峰峰值 / 4
  *   ★ 一开始写的是"每 512 个样本统计一段 min/max"，对**方波**是错的 ——
  *     方波半周期可能远长于 512 点，整段落在一个电平里会让峰峰值算成 0，
  *     被误判成"没信号"。这就是锁不上 150Hz 方波的真身，详见 trk_env_update()。
  *
  * ========================== 点 3：为什么要迟滞 + "武装"状态 ==========================
  *   噪声大的时候，信号在阈值附近来回穿越，一个周期会数出好几个上升沿，
  *   频率直接翻倍。所以要求：**必须先跌到 dc - hyst 以下，才允许检测下一个上升沿**。
  *   这就是 s_armed 的作用，等价于施密特触发器。
  *
  * ========================== 点 4：为什么时间戳要拆成整数+小数 ==========================
  *   上升沿时刻 = 样本序号 + 插值小数部分。
  *   样本序号到 2^24 之后 float32 就存不下小数了（500kSPS 下只要 33 秒），
  *   所以整数部分用 uint32_t 单独存，小数部分才用 float。
  *   跨度用 (int32_t)(末 - 首) 计算，天然对 uint32 回绕安全。
  ******************************************************************************
  */

#include "track.h"
#include "adc_vofa.h"      /* adc_buf 访问接口 + ADC_BUF_LEN */
#include "adc.h"           /* hadc2 */
#include "signal.h"        /* Signal_SetFreq() / SIGNAL_FREQ_MIN/MAX */
#include "tim.h"
#include <math.h>

/* ============================== 可调参数 ============================== */

/* 采样率，必须和 ADC_VOFA_FS_HZ 一致（本模块不改 TIM7，只按这个值换算时间） */
#define TRK_FS_HZ           ((float)ADC_VOFA_FS_HZ)

/* 至少攒够几个上升沿才算一次频率（= 周期数 + 1）。
 * 取 3 是为了低频也能在 2 个周期内出结果，兼顾响应速度。 */
#define TRK_MIN_CROSS       3U

/* 滑动窗口最多保留几个上升沿。
 * ★ 2026-10-12 从 12 → 20 → 40 ★
 *   用户明确要求："可以慢，但不能不稳定"。判定依据就是多跨几个波，
 *   跨的周期越多，单次测量的相对误差越小（误差 ≈ 1/跨的周期数）。
 *   代价只是响应变慢，而这正是可以接受的。
 *      50kHz 时 40 个沿 = 0.8ms
 *      1kHz  时 40 个沿 = 39ms
 *      150Hz 时 40 个沿 = 260ms
 *      50Hz  时 40 个沿 = 780ms   ← 最慢的一档，仍可接受 */
#define TRK_MAX_CROSS       40U

/* 目标跨度（样本数）。达到了就立刻算，不再等更多上升沿。
 * 500kSPS 下 1500 样本 = 3ms：1kHz 时正好 3 个周期，
 * 高频（50kHz）时根本攒不满，会一直用满 TRK_MAX_CROSS 个沿。 */
#define TRK_SPAN_SAMPLES    1500U

/* 判定"有信号"的最小峰峰值（码值，4095 = 3.3V）。40 ≈ 32mV。
 * ★ 这个值不能太小：空脚悬空时的工频耦合就有几十个码值，
 *   会把输出频率带到 50Hz 上去。 */
#define TRK_MIN_PK          40

/* 峰值包络的衰减周期（样本数）—— 每这么多样本，包络两端各朝中间收缩 1 个码值。
 *   32 → 500kSPS 下 15625 码值/秒，满量程 4095 收缩一遍约 262ms。
 *   - 太小：低频信号会在两个峰之间塌掉，峰峰值虚低
 *   - 太大：信号断开后迟迟不报"无信号"
 *   20Hz 方波（周期 50ms）一个周期只衰减约 780 码值，峰峰值仍稳在 3300 以上。
 *   见 trk_env_update() 的说明：为什么不能再用"每段统计 min/max"。 */
#define TRK_ENV_PERIOD      32U

/* 缓冲区尾部不碰的样本数（等 DMA 写完） */
#define TRK_SAFE_MARGIN     8U

/* ---- 输出三级处理 ---- */
/* 变化超过这个比例视为"对方真的换频了"，直接跟上去，不走低通。
 * ★ 从 0.25 提到 0.6 ★ 免得一次偶然的偏大测量被当成"换频"而瞬间拖走输出。 */
#define TRK_JUMP_RATIO      0.6f

/* ★★ 低通用"时间常数"而不是"固定系数"（2026-10-12 改）★★
 *
 *   原来写的是 s_out_hz += 0.35f * (f - s_out_hz)，固定系数。
 *   问题是测量次数跟频率成正比：1kHz 时每秒测 1000 次，50Hz 时每秒 50 次 ——
 *   同样的系数，低频几乎没滤波、高频滤得很狠，行为完全不一致。
 *
 *   改成按时间算系数（一阶低通的标准离散化）：
 *       alpha = dt / (TAU + dt)
 *   这样不管测量多快多慢，时间常数都是 TAU 秒，平滑程度一致。
 *
 *   TAU 取 0.5s：真实换频大约 1.5s 后跟上 —— 用户说了"可以慢，但不能不稳定"。 */
#define TRK_LP_TAU_S        0.5f

/* ★★ 斜率限制 —— 治"抖动"最狠的一招（2026-10-12 加）★★
 *
 *   用户的原始需求："主要假设对短期不变……可以慢，但不能不稳定"。
 *   既然对方信号短期内不会变，输出频率就**没有理由快速变化** ——
 *   任何比这个上限更快的"变化"都是测量噪声，不是真的换频。
 *
 *   所以限定 s_out_hz 的**移动量**：
 *       max_move = s_out_hz × TRK_SLEW_PCT_S × dt
 *   1kHz 测量率下 dt≈1ms、取 0.1 时每次最多移动 0.1Hz（即 100Hz/s）。
 *   一次尖峰只持续约 1ms，最多把输出拉走 0.1Hz —— 彻底挡在门外；
 *   而真实换频由低通算出的移动量远小于这个上限，不受任何影响
 *   （对方变 10% 约 1 秒跟上）。
 *
 *   ⚠️ 千万别把限制加在 f 上 —— 那样斜率上限会再乘一次低通系数，
 *      收敛速度掉到约 1Hz/s，真实换频要几十秒才跟上（踩过）。
 *
 *   真实大换频（>60%）走 TRK_JUMP_RATIO 那条快速通道，也不受影响。 */
#define TRK_SLEW_PCT_S      0.1f
/* 相对死区：输出值变化小于 0.2% 就不去动 DAC。
 * ★ 必须要有：Signal_SetFreq() 内部可能重铺正弦表（1ms，期间输出冻结），
 *   调用太频繁会在输出上留下周期性毛刺。
 * ★ 从 0.1% 提到 0.2%：配合上面按时间常数的低通，
 *   让 DAC 的频率只在 s_out_hz 真的漂出 0.2% 时才变，输出更稳。
 *   相位由 AWD 每周期硬复位，所以 0.2% 的频率差不会累积、看不出来。 */
#define TRK_DEADBAND        0.002f
/* 两次真正 SetFreq 之间的最小间隔(ms)，进一步限制相位复位带来的扰动 */
#define TRK_MIN_UPDATE_MS   1U

/* 算出来的频率超出这个范围就当作误检丢弃 */
#define TRK_FREQ_LO         (SIGNAL_FREQ_MIN * 0.5f)
#define TRK_FREQ_HI         (SIGNAL_FREQ_MAX * 1.5f)

/* ==================== 波形识别 / 电平搬运 / 相位对齐 参数 ==================== */

/* "贴轨"判据的带宽：峰峰值 / 这个数。5 → 上下各 20%。
 * ★ 判据是 min(贴下轨比例, 贴上轨比例) —— 取 min 而不是取总数，原因见 trk_wave_detect()。
 *   各波形按 min 算出来的理论值：
 *       方波  上下轨各 ~500‰ → min ~500   ← 判为方波
 *       正弦  |x|>0.8 各 ~205‰ → min ~205
 *       三角  各 ~100‰        → min ~100
 *       恒定电平  只有一轨有样本 → min ~0
 *   所以阈值取 0.35(350‰)：500 过线、205 不过线，干净分开。
 *   ⚠️ 踩过一次：判据从"总贴轨比例"改成 min 之后忘了同步改这个宏，
 *      结果方波算出来 461‰ 卡在 700‰ 阈值下面，被误判成正弦。 */
#define TRK_RAIL_BAND_DIV   5
#define TRK_SQUARE_RATIO    0.35f

/* 每这么多样本判一次波形类型（500kSPS 下 4096 点 ≈ 8.2ms，够快也不至于抖） */
#define TRK_DETECT_LEN      4096U

/* AWD 阈值（= 对方波形的直流中点）的允许范围。
 * 太靠近两端就没法可靠判边沿了，所以要夹住。 */
#define TRK_AWD_THR_MIN     64U
#define TRK_AWD_THR_MAX     4031U

/* 电平/波形参数只在变化超过这些阈值时才真正写进 signal.c ——
 * 因为每次写都可能触发重铺表（0.5ms 冻结输出），不能让它跟着包络抖动乱写。 */
#define TRK_LEVEL_EPS       16U      /* 电平码值变化超过 16 才更新 */
#define TRK_DUTY_EPS        0.03f    /* 占空比变化超过 3% 才更新 */

/* ★★ 慢变量的更新节流（2026-10-12 加，这是"丢样本→频率算错"的最后一环）★★
 *
 *   波形类型 / 电平 / 占空比都是**慢变量**，但每写一次都可能触发重铺整张表
 *   （1024 次 sinf() ≈ 0.5~1ms，期间 TIM6 停着、输出被冻结）。
 *   实测踩到：占空比估计在 47%~52% 之间抖（真值约 50%），
 *   每次都超过 TRK_DUTY_EPS → 每秒触发几十次重铺 →
 *   主循环单次超过 4.1ms → DMA 套圈丢样本 → s_total 落后 → 频率算错。
 *   现象是 g_trk_laps 从 0 涨到 21.8/s。
 *
 *   所以这里硬性限速：最快每 TRK_PARAM_MIN_MS 毫秒才允许写一次。
 *   代价是"对方幅度/占空比变了"最多延迟 250ms 才跟上 —— 完全可以接受。 */
#define TRK_PARAM_MIN_MS    250U

/* 李萨如图形模式：输出频率相对输入的倍频系数。
 *   1.0 = 斜线/圆发生（同频）
 *   2.0 = 无穷发生（x=sin(wt), y=sin(2wt)）
 * 相位仍由输入上升沿 Realign 到表头相位 0。 */
#define TRK_OUTPUT_FREQ_MULT  2.0f

/* ============================== 开环测试模式 ==============================
 * ★★ 为什么需要这个开关：自监控闭环会失控 ★★
 *
 *   把 PB15 接回 PA2（自己看自己）时，闭环是"单位增益正反馈"：
 *       f_out ← f_measured(f_out)
 *   理想情况下 f_measured == f_out，这是个**临界稳定**的环，
 *   任何一点系统性偏差都会被自己放大。而 Signal_SetFreq() 每次换频都会：
 *       ① 停 TIM6 / 停 DMA
 *       ② DAC3->DHR12R1 = sine_tab[0]   ← 强制把输出拉回中点 1.65V
 *       ③ 重新起振（相位归零）
 *   第 ② 步在输出上是一个**直接跳到中点的台阶**。自监控时这个台阶会被
 *   自己的 ADC 采到，如果信号当时正好在负半周，台阶就是一次"上升沿穿越阈值"
 *   → 多数出一个沿 → 测得的频率偏高 → 输出调高 → 换频更频繁 → 更多假沿。
 *   实测就是这样一路从 1kHz 爬到 17kHz。
 *
 *   ★ 但接**对方**的真实信号时不存在这个问题 ★
 *     自己的输出台阶不会跑进对方的信号里，测量对象始终是干净的，
 *     闭环是"外部基准 → 输出"，稳定收敛。
 *
 *   所以自监控只适合做**开环标定**，不适合做闭环演示 —— 这个变量就是为此准备的。
 */
volatile float g_trk_hold_hz = 0.0f;

/* ============================== 诊断计数 ============================== */
volatile uint32_t g_trk_measures = 0U;
volatile uint32_t g_trk_updates  = 0U;
volatile uint32_t g_trk_valid    = 0U;
volatile uint32_t g_trk_cross_n  = 0U;
volatile uint32_t g_trk_in_hz    = 0U;
volatile uint32_t g_trk_out_hz   = 0U;
volatile uint32_t g_trk_pk       = 0U;
volatile uint32_t g_trk_resets   = 0U;
volatile uint32_t g_trk_samples  = 0U;

/* ---- 波形匹配 / 相位对齐 诊断 ---- */
volatile uint32_t g_trk_edges    = 0U;   /* ADC 模拟看门狗(AWD)实际执行 Realign 的次数 */
volatile uint32_t g_trk_edge_skip= 0U;   /* 被去抖滤掉的沿（振铃/毛刺） */
volatile uint32_t g_trk_wave     = 0U;   /* 识别出的波形：0=正弦 1=三角 2=方波 */
volatile uint32_t g_trk_sq_ratio = 0U;   /* 贴轨样本比例 ×1000（方波≈1000，正弦≈410） */
volatile uint32_t g_trk_duty_ppm = 0U;   /* 对方方波占空比，单位 ppm */
volatile uint32_t g_trk_vmin     = 0U;   /* 对方低电平码值（直接搬给 DAC 用） */
volatile uint32_t g_trk_vmax     = 0U;   /* 对方高电平码值 */
volatile uint32_t g_trk_laps     = 0U;   /* ★ 因主循环太慢而丢掉整圈样本的次数（应该一直是 0）*/
volatile uint32_t g_trk_reject   = 0U;   /* ★ 被假沿校验丢弃的过零点个数（假沿会直接拉偏频率）*/
volatile uint32_t g_trk_raw_hz   = 0U;   /* ★ 原始测量值（只过中值，未平滑）—— 抖动诊断用 */
volatile uint32_t g_trk_awd_irq  = 0U;   /* ★ AWD 中断总次数。正常 = 输入频率 × 2
                                          *   （每周期一次上升沿 + 一次下降沿）。
                                          *   明显偏高 = 有多余中断在触发假 Realign → 会跳相。 */
volatile uint32_t g_trk_awd_heal = 0U;   /* ★ AWD 自愈次数。正常应一直是 0；
                                          *   在涨说明阈值/窗口曾经错位（AWD 一度完全没中断）。 */
volatile uint32_t g_trk_awd_glitch = 0U; /* ★ 被判为"假事件"而整条丢弃的 AWD 中断。
                                          *   这些是阈值附近抖动/改窗口引出的、间隔不足半个周期的
                                          *   伪过零。**丢弃它们才能保住状态机的奇偶性**，
                                          *   否则一次假事件就会让输出永久差 180°。 */
volatile uint32_t g_trk_awd_miss = 0U;   /* ★ 检出"漏了一个沿"并已把奇偶性补回来的次数。
                                          *   见 ADC1_2_IRQHandler 里的时间守卫。 */
volatile uint32_t g_trk_awd_flip = 0U;   /* ★ 真正执行的"奇偶性纠正"次数（含漏沿和离谱间隔）。 */
volatile uint32_t g_trk_awd_skip = 0U;   /* ★ 因"刚改过 AWD 窗口"而被丢弃相位复位的次数。
                                          *   见 trk_awd_set_thr() 坑 C：改窗口会凭空造出假过零。 */

/* ============================== 内部状态 ============================== */

static uint32_t s_rd;               /* 已处理到的缓冲下标 */
static uint32_t s_total;            /* 全局样本计数（可回绕） */
static int32_t  s_prev;             /* 上一个样本值 */
static uint32_t s_prev_g;           /* 上一个样本的全局序号 */
static uint8_t  s_armed;            /* 1 = 已回到阈值下方，允许检测下一个上升沿 */

static int32_t  s_dc;               /* 直流中点估计 */
static int32_t  s_hyst;             /* 迟滞半宽 */
static uint8_t  s_have_dc;          /* 直流估计是否已建立 */

static int32_t  s_env_min;          /* 峰值包络下沿（快攻慢放） */
static int32_t  s_env_max;          /* 峰值包络上沿 */
static uint32_t s_env_cnt;          /* 包络衰减计时 */

/* 环形上升沿时间戳：整数部分是全局样本序号，小数部分是插值结果 */
static uint32_t s_cidx[TRK_MAX_CROSS];
static float    s_cfrac[TRK_MAX_CROSS];
static uint8_t  s_cn;               /* 有效个数（<= TRK_MAX_CROSS） */
static uint8_t  s_cw;               /* 下一个写入位置 */

static float    s_in_hz;            /* 本次测得的输入频率 */
static float    s_out_hz;           /* 低通后的输出频率 */
static float    s_set_hz;           /* 上次真正设给 DAC 的值 */
static uint8_t  s_have_out;         /* 输出频率是否已建立 */
static uint32_t s_last_upd_ms;      /* 上次 SetFreq 的时刻 */
static uint32_t s_lp_cyc;           /* 上次低通更新的 DWT 周期数（算 dt 用） */

/* ---------- 波形识别 / 电平搬运 / 相位对齐 的内部状态 ---------- */
static uint32_t s_lo_rail;          /* 当前统计窗里贴**下轨**的样本数 */
static uint32_t s_hi_rail;          /* 当前统计窗里贴**上轨**的样本数 */
static uint32_t s_win_cnt;          /* 当前统计窗里的总样本数 */

/* ★ 每周期极值 —— "稳定电平"（2026-10-12 加，修"电平抖动疯狂重铺表"的 bug）★
 *
 *   为什么不能直接拿 s_env_min/s_env_max 当输出电平：
 *   包络是"快攻慢放"的（每 TRK_ENV_PERIOD 个样本两端各朝中间收 1 码），
 *   它在一个信号周期内会漂移好几个百分点，然后被下一周期的真极值复位 ——
 *   也就是一条锯齿波。而 trk_apply_waveform() 的更新判据是
 *   "变化 > TRK_LEVEL_EPS 码"，于是**每次测量都判定成"变了"**，
 *   每秒调 170 次 Signal_SetLevels() → 每次都重铺整张表
 *   （1024 次 sinf() ≈ 0.5~1ms，期间 TIM6 停着、输出冻结）。
 *   实测后果：主循环被拖慢 → DMA 绕圈丢样本 → s_total 比真实时间走得慢
 *   → f = (沿数-1)*fs/span 里的 span 偏小 → 频率偏高，
 *   实际观察到在 156Hz 和 310Hz 之间来回跳（约 2 倍）。
 *
 *   改法：输出电平取"相邻两个上升沿之间的真实 min/max"，这是**稳定**的，
 *   只有对方幅度真的变了才会动。包络仍然保留，给 dc/hyst/valid 用
 *   （它必须会衰减，才能检测出"信号没了"）。 */
static int32_t  s_per_min;          /* 本周期内（自上个上升沿起）的最小值 */
static int32_t  s_per_max;          /* 本周期内的最大值 */
static uint8_t  s_lvl_valid;        /* 下面的"稳定电平"是否已建立 */
static uint32_t s_lvl_min;          /* 稳定的低电平码值（给 Signal_SetLevels 用） */
static uint32_t s_lvl_max;          /* 稳定的高电平码值 */

/* DMA 绕圈检测：主循环太慢时 DMA 会整整绕一圈，样本就被丢掉了，
 * 这时 s_total 与实际时间不再对应，必须清空过零历史重新开始，
 * 否则算出来的频率是错的（这个计数器用来观察发生了多少次）。 */
static uint32_t s_last_fill;        /* 上次 poll 时 DMA 的写入位置 */
static uint8_t  s_fill_valid;
static uint32_t s_cyc_base;         /* 起始 CPU 周期数（DWT），用作采样时间基准的零点 */

/* ★ 时间基准用 CPU 周期数，不要用"数 DMA 绕了几圈"（2026-10-12 踩坑）★
 *
 *   TIM7 是 ARR=339，也就是每 **340** 个 170MHz 周期触发一次 ADC 转换。
 *   而 CPU 和 TIM7 同一个 170MHz 时钟，所以
 *
 *       从 Init 到现在"应该已经采了多少样本"  =  (DWT->CYCCNT - s_cyc_base) / 340
 *
 *   这个数是**精确**的，与主循环快慢完全无关，也不可能漏数。
 *
 *   之前写成"观察 fill 变小就 s_wraps++"来累计写入量，有个致命漏洞：
 *   如果两次 poll 之间 DMA 走了**一整圈还多一点**（2048+k 个样本），
 *   fill 结束时反而比上次**大**，绕圈就看不见了 → 凭空少算 2048 个样本
 *   → s_total 落后于真实时间 → f=(沿数-1)*fs/span 里的 span 偏小 → **频率偏高**。
 *   实测现象：真正的输入是 158Hz（g_trk_edges 速率 158/s 是对的），
 *   算出来却是 205~241Hz，而且 samples 速率只有 365k/s（应该是 500k/s）。
 */
#define TRK_CYC_PER_SAMPLE  340U
static uint32_t s_hi_cnt;           /* 自上个上升沿以来大于 dc 的样本数 */
static uint32_t s_lo_cnt;           /* 自上个上升沿以来小于 dc 的样本数 */
static uint8_t  s_duty_done;        /* 占空比是否已经测到过一次 */
static float    s_duty = 0.5f;      /* 平滑后的占空比 */
static uint8_t  s_detected_wave = SIG_WAVE_SINE;

static uint8_t  s_awd_arm_up;       /* AWD 窗口状态：1 = 等上升沿  0 = 等下降沿 */
static uint32_t s_awd_thr = 2048U;  /* AWD 阈值 = 对方波形的直流中点（码值） */
static uint32_t s_last_awd_cyc = 0U;/* 上次 AWD 中断的 DWT 周期数，供自愈安全网判断 */
static uint8_t  s_phaselock_on;     /* 1 = 允许 AWD 中断去复位输出相位（有有效信号时才开） */
static uint8_t  s_awd_skip;         /* ★ 刚重写过窗口：紧接着的第一次事件不可信（见 trk_awd_set_thr 的坑 C） */
static uint32_t s_awd_half_cyc;     /* ★ 半个输入周期的 CPU 周期数（主循环按输出频率算，只给自愈超时用） */
static uint8_t  s_awd_have_cyc;     /* ★ 是否已经有了一次可信的 AWD 事件时间戳 */
static uint32_t s_awd_ema;          /* ★ 相邻 AWD 事件间隔的滑动平均（≈半周期），时间守卫的基准。
                                     *   **故意用事件自身的间隔而不是主循环给的周期** ——
                                     *   对方换频时主循环那个值会过期，守卫就会误判"漏沿"而错误翻转奇偶性。 */
static uint32_t s_awd_rearm_cyc;    /* ★ 上次重写 AWD 窗口的时刻，用来界定"改写引出的事件"的时间窗 */
static uint32_t s_last_realign_cyc; /* ★ 上次真正复位相位的时刻，时间守卫用（相邻复位间隔 ≈ 输入周期） */

/* 重写窗口后多久以内到来的 AWD 事件算"改写引出的假边沿"（170MHz × 30us）。
 * 取 30us：ADC 采样周期 2us + AWDFILT 两次 + 中断响应，满打满算不到 8us，
 * 留 4 倍余量；而 1kHz 时它只占周期的 3%，误吞真沿的概率极低。 */
#define TRK_AWD_REARM_GRACE_CYC   5100U

static uint8_t  s_app_valid;        /* 下面几个"已经写进 signal.c 的值"是否有效 */
static uint32_t s_param_ms;         /* 上次写慢变量（波形/电平/占空比）的时刻，用于节流 */
static uint32_t s_app_vmin;
static uint32_t s_app_vmax;
static uint8_t  s_app_wave = 0xFFU;
static float    s_app_duty = -1.0f;

/* 测频结果的中值滤波（5 点），见 trk_median5() */
static float    s_med[5];
static uint8_t  s_med_n;
static uint8_t  s_med_i;

/* 前置声明：这几个函数定义在 trk_env_update / trk_feed / trk_maybe_measure 之后，
 * 但它们要被前面的函数调用，所以必须先声明，否则编译器按隐式声明处理会报错。 */
static void trk_duty_update(void);
static void trk_wave_detect(void);
static void trk_apply_waveform(void);
static void trk_awd_set_thr(uint32_t thr);
static void trk_awd_force(uint32_t thr);

/* ============================== 内部函数 ============================== */

/**
  * @brief  只清"测量侧"的状态：过零历史 + 阈值锁定
  *
  * ★ 故意**不动** s_have_out / s_out_hz / s_set_hz ★
  *   信号短暂丢失时（对方换频的过渡期、波形瞬时畸变等），输出频率必须原地保持。
  *   一旦把 s_set_hz 清零，trk_maybe_measure() 里的 need 判据立刻为真，
  *   就会强制调用 Signal_SetFreq()；而每次调用都可能重铺整张表、把输出冻住
  *   零点几毫秒（见 signal.c 快路径的说明），于是滚成
  *       "丢信号 → 强制换频 → 输出被冻 → 更像丢信号"
  *   的正反馈。这是实测踩到的第二个正反馈（第一个是换频台阶）。
  *   实测：不清输出状态后 resets 会自然停下来，清了就一路涨到几千。
  */
static void trk_clear_meas(void)
{
    s_cn          = 0U;
    s_cw          = 0U;
    s_armed       = 0U;
    s_have_dc     = 0U;
    g_trk_cross_n = 0U;
}

/**
  * @brief  完全复位（只在 Track_Init 里用）
  */
static void trk_clear(void)
{
    trk_clear_meas();
    s_have_out = 0U;
    s_out_hz   = 0.0f;
    s_set_hz   = 0.0f;
}

/**
  * @brief  包络收缩一拍：两端各朝中间收 1 个码值，然后重算直流/迟滞
  *
  * ★★★ 为什么不用"每段统计 min/max"（2026-10-12 踩的坑）★★★
  *
  *   最初的写法是每 512 个样本统计一次 min/max 来估直流电平：
  *       这一段的峰峰值 < 40  →  判定"没信号"
  *   这对**正弦**没问题（512 点 @500kSPS = 1.02ms，1kHz 正弦能装 1 个周期）。
  *   但对方的波可能是**方波**，而且半周期可能比 1.02ms 长得多 ——
  *   于是大量统计段整段落在一个电平里，min == max，峰峰值算出来是 0，
  *   被误判成"没信号"，进而把过零历史清掉、把输出状态也清掉。
  *   实测现象：pk 在 0 和 4095 之间乱跳，resets 每秒涨 300，
  *   频率死活锁不上（in_hz 一直是 0）。信号本身完全正常。
  *
  *   换成**峰值包络检波**（就是收音机 AGC 那套）：
  *       遇到更极端的值  →  立刻更新（快攻，一个样本就够）
  *       否则每 TRK_ENV_PERIOD 个样本  →  朝中间收缩 1 个码值（慢放）
  *   它不关心信号周期长短，只看"最近见过多高、多低"，
  *   方波、正弦、三角、带偏置的信号一律适用；信号真断了包络会自然塌回去，
  *   峰峰值变小 → 自动报"无信号"，不需要额外的超时逻辑。
  */
static void trk_env_update(void)
{
    int32_t pk;

    /* 慢放：两端朝中间收 1 码。收到相遇为止（中间值就是直流电平） */
    if (s_env_min < s_env_max)
    {
        s_env_min++;
        s_env_max--;
    }

    pk = s_env_max - s_env_min;
    g_trk_pk = (pk > 0) ? (uint32_t)pk : 0U;

    if (pk < TRK_MIN_PK)
    {
        /* 包络塌了 → 没接信号 / 只有直流。保持输出不动，清掉测量侧历史。 */
        if (g_trk_valid != 0U)
        {
            g_trk_resets++;
        }
        g_trk_valid = 0U;
        s_phaselock_on = 0U;            /* ★ 没信号时禁止 AWD 中断复位相位 ★
                                         *   否则阈值附近的噪声会不停地把输出波形打断成碎片。 */
        s_lvl_valid = 0U;               /* 稳定电平也作废，等信号回来重新量 */
        trk_clear_meas();
        return;
    }

    {
        int32_t dc = (s_env_min + s_env_max) / 2;

        if (s_have_dc == 0U)
        {
            s_dc      = dc;             /* 首次直接采纳，立刻能开始过零检测 */
            s_have_dc = 1U;
        }
        else
        {
            s_dc += (dc - s_dc) / 8;    /* 慢跟踪，避免阈值跟着波形抖 */
        }

        s_hyst = pk / 4;                /* 迟滞半宽 = 峰峰/4，等价施密特触发器 */
        if (s_hyst < 4)
        {
            s_hyst = 4;
        }
    }

    /* AWD 的阈值取"稳定电平"的中点，比会衰减的 s_dc 更稳，
     * 阈值不抖 → AWD 不会在阈值附近反复触发。还没量够一个周期时退回 s_dc。 */
    {
        uint32_t thr = (s_lvl_valid != 0U)
                     ? ((s_lvl_min + s_lvl_max) / 2U)
                     : (uint32_t)s_dc;

        /* ★ 时间守卫要用的"半个周期"（CPU 周期数）★
         *   输出频率 == 输入频率，所以直接用 s_out_hz 算，不需要额外测量。
         *   中断里拿它判别"这次事件是不是假的/是不是漏了一个沿"。 */
        if (s_out_hz > (float)SIGNAL_FREQ_MIN)
        {
            uint32_t h = (uint32_t)(170000000.0f / (2.0f * s_out_hz));
            if (h < 2000U) { h = 2000U; }       /* 兜底：最快按 42.5kHz 算，防除零/离群 */
            s_awd_half_cyc = h;
        }

        /* ★ 自愈安全网 ★
         *   信号有效、却已经超过"10 个周期"没收到 AWD 中断 → 阈值/窗口一定错位了
         *   （比如阈值比信号峰值还高，样本永远进不了窗口，AWD 永远不中断，
         *    也就永远没机会自己恢复 —— 死锁）。
         *   超时门限**跟着周期走**：固定 20ms 的写法在低频输入（<50Hz）下会一直误触发，
         *   而 force 会按可能过期的 s_prev 重定状态 → 反而制造奇偶性错误。 */
        {
            uint32_t to = (s_awd_half_cyc > 0U) ? (s_awd_half_cyc * 20U) : (170000000U / 50U);
            if (to < (170000000U / 200U)) { to = 170000000U / 200U; }   /* 下限 5ms */
            if ((uint32_t)(DWT->CYCCNT - s_last_awd_cyc) > to)
            {
                trk_awd_force(thr);
            }
            else
            {
                trk_awd_set_thr(thr);
            }
        }
    }

    g_trk_valid    = 1U;
    s_phaselock_on = 1U;                /* 确认有有效信号了，才允许 AWD 中断复位输出相位 */
}

/**
  * @brief  取环形历史里第 i 个（0 = 最老的）时间戳下标
  */
static uint8_t trk_slot(uint8_t i)
{
    return (uint8_t)((s_cw + TRK_MAX_CROSS - s_cn + i) % TRK_MAX_CROSS);
}

/**
  * @brief  记录一个上升沿时刻（带假沿校验）
  * @param  idx  该沿所在的前一个样本的全局序号（沿位于 idx 和 idx+1 之间）
  * @param  frac 插值得到的小数位置，[0,1)
  *
  * ★★ 假沿校验 —— 修"输出乱跳"的关键（2026-10-12 实测定位）★★
  *
  *   症状：测频结果里约 25% 是比真值高 13%~22% 的离群值，
  *         输出频率跟着跳（VOFA+ 上 ch1/ch2 峰峰有 300Hz）。
  *
  *   为什么 3 点中值挡不住：假过零点一旦进了滑动窗口，会在里面**滞留
  *   最多 TRK_MAX_CROSS 次测量**（每次窗口只滑动一个沿），
  *   所以离群值不是孤立的、而是成串出现，中值滤波无能为力。
  *
  *   假沿从哪来：主循环被拖慢时 DMA 套圈丢样本，或者读到 DMA 正在写的
  *   那个半新半旧的样本 —— 两者都会凭空造出一个过零点。
  *
  *   假沿的特征很干净：**它和上一个沿的间隔远小于正常周期**。
  *   正常过零点之间至少隔一个周期，假沿则紧贴着真沿。
  *   所以只要拿当前输出频率当参考，间隔 < 半个周期就直接丢弃。
  *   比"事后滤波"强得多：脏数据根本进不了窗口。
  */
static void trk_push_cross(uint32_t idx, float frac)
{
    /* ---- 假沿校验：和上一个沿太近就丢弃 ---- */
    if (s_cn > 0U)
    {
        uint8_t  prev_i = trk_slot((uint8_t)(s_cn - 1U));
        uint32_t d      = (uint32_t)((int32_t)idx - (int32_t)s_cidx[prev_i]);
        float    dt     = (float)d + (frac - s_cfrac[prev_i]);
        float    fref   = (s_out_hz > 1.0f) ? s_out_hz : 1000.0f;
        float    expect = TRK_FS_HZ / fref;      /* 预期周期（样本数） */

        if (dt < (expect * 0.5f))
        {
            g_trk_reject++;
            return;                              /* 判为假沿，不记入窗口 */
        }
    }

    s_cidx[s_cw]  = idx;
    s_cfrac[s_cw] = frac;

    s_cw = (uint8_t)((s_cw + 1U) % TRK_MAX_CROSS);
    if (s_cn < TRK_MAX_CROSS)
    {
        s_cn++;
    }
    g_trk_cross_n = (uint32_t)s_cn;
}

/**
  * @brief  5 点中值滤波 —— 抑制测频结果里的离群值
  *
  * ★ 为什么需要它（2026-10-12 实测）★
  *   测频结果里会混进离群值（比真值偏高或偏低十几~几十个百分点）。
  *   来源：主循环被拖慢时 DMA 套圈丢样本、读到 DMA 正在写的半新半旧样本、
  *   或者丢失/多出一个过零点 —— 都会让"沿数与跨度"不再匹配。
  *
  *   真正的第一道防线是 trk_push_cross() 里的假沿校验（在源头丢弃），
  *   这里的中值只是第二道防线：即使漏进来一两个，也不会被均值那样拉偏。
  *
  *   ★ 从 3 点扩到 5 点：假沿有时会连续漏进两个，3 点中值挡不住，5 点可以。
  *     代价是真实换频慢一拍 —— 用户说了"可以慢，但不能不稳定"，正合适。
  */
static float trk_median5(float x)
{
    float   t[5];
    uint8_t i;
    uint8_t j;

    s_med[s_med_i] = x;
    s_med_i = (uint8_t)((s_med_i + 1U) % 5U);
    if (s_med_n < 5U)
    {
        s_med_n++;
    }
    if (s_med_n < 5U)
    {
        return x;                       /* 还没攒够 5 个，先原样返回 */
    }

    for (i = 0U; i < 5U; i++)
    {
        t[i] = s_med[i];
    }

    /* 5 个元素的插入排序，够用 */
    for (i = 1U; i < 5U; i++)
    {
        float key = t[i];

        j = i;
        while ((j > 0U) && (t[j - 1U] > key))
        {
            t[j] = t[j - 1U];
            j--;
        }
        t[j] = key;
    }

    return t[2];                        /* 中值 */
}

/**
  * @brief  够条件就算一次频率，并三级处理后写进 DAC
  */
static void trk_maybe_measure(void)
{
    uint8_t  i_first;
    uint8_t  i_last;
    uint32_t d_idx;
    float    span;
    float    f;

    if (s_cn < TRK_MIN_CROSS)
    {
        return;                         /* 沿还不够 */
    }

    i_first = trk_slot(0U);
    i_last  = trk_slot((uint8_t)(s_cn - 1U));

    /* 整数跨度用有符号差，uint32 回绕安全 */
    d_idx = (uint32_t)((int32_t)s_cidx[i_last] - (int32_t)s_cidx[i_first]);
    span  = (float)d_idx + (s_cfrac[i_last] - s_cfrac[i_first]);

    /* 还没攒够跨度、窗口也没满 → 再等等，让平均的周期数更多、结果更稳 */
    if ((span < (float)TRK_SPAN_SAMPLES) && (s_cn < TRK_MAX_CROSS))
    {
        return;
    }
    if (span <= 1.0f)
    {
        return;                         /* 退化情况，防除零 */
    }

    f = ((float)(s_cn - 1U) * TRK_FS_HZ) / span;

    if ((f < TRK_FREQ_LO) || (f > TRK_FREQ_HI))
    {
        return;                         /* 离谱，当误检丢掉 */
    }

    /* ★ 先过 5 点中值，剔掉离群值（第一道防线是 trk_push_cross 的假沿校验）★ */
    f = trk_median5(f);

    g_trk_raw_hz = (uint32_t)(f + 0.5f);    /* 原始测量值，仅供诊断 */
    s_in_hz     = f;
    g_trk_measures++;
    g_trk_in_hz = (uint32_t)(f + 0.5f);

    /* ---- 开环测试模式：只报告测量值，绝不碰输出 ---- */
    if (g_trk_hold_hz >= 1.0f)
    {
        g_trk_out_hz = (uint32_t)(g_trk_hold_hz + 0.5f);
        return;
    }

    /* ---- 同波形 / 同电压 / 同占空比：把对方的参数搬给 DAC ----
     * 放在频率处理之前：波形和电平是"慢变量"，先更新它们，后面的换频才能基于新表算。 */
    trk_apply_waveform();

    /* ---- 第二级：换频检测 / 按时间常数的一阶低通 ----
     * ★ 用 DWT 周期数算 dt，得到"以秒为单位"的固定时间常数 ★
     *   固定系数的写法在不同频段行为完全不同（测量次数跟频率成正比），
     *   必须按时间算 alpha = dt/(TAU+dt) 才能到处都一样平滑。 */
    {
        uint32_t now_cyc = DWT->CYCCNT;
        float    dt      = (float)(uint32_t)(now_cyc - s_lp_cyc) / 170000000.0f;
        float    alpha;

        s_lp_cyc = now_cyc;
        if (dt > 2.0f)
        {
            dt = 2.0f;                  /* 首次 / 长时间没更新：直接把 alpha 拉到 1 */
        }
        alpha = dt / (TRK_LP_TAU_S + dt);
        if (alpha > 1.0f)
        {
            alpha = 1.0f;
        }

        if (s_have_out == 0U)
        {
            s_out_hz   = f;
            s_have_out = 1U;
        }
        else if (fabsf(f - s_out_hz) > (TRK_JUMP_RATIO * s_out_hz))
        {
            s_out_hz = f;               /* 对方真的换频了（差距很大），直接跟上 */
        }
        else
        {
            /* 1) 先做"按时间常数"的一阶低通 */
            float target = s_out_hz + alpha * (f - s_out_hz);

            /* 2) ★ 再对"这一步的移动量"做斜率限制 ★
             *
             *   ⚠️ 必须限制 s_out_hz 的移动量，**不能限制 f**（踩过一次）：
             *      限制 f 的话，斜率上限会再乘一次 alpha ——
             *      每步移动 = alpha × max_d ≈ 0.002 × 0.5 = 0.001Hz，
             *      收敛速度被压到约 1Hz/s，真实换频要几十秒才跟上，
             *      现象是 s_out_hz 呈**线性**慢爬而不是指数收敛。
             *
             *   限制移动量之后：
             *     真实换频：低通算出的移动量远小于上限 → 完全不受限，正常收敛
             *     测量尖峰：单次只影响一个测量周期（约1ms），最多把输出拉走
             *               s_out_hz×0.1×0.001 ≈ 0.1Hz → 彻底挡在门外 */
            {
                float max_move = s_out_hz * TRK_SLEW_PCT_S * dt + 0.02f;
                float move     = target - s_out_hz;

                if (move >  max_move) { move =  max_move; }
                if (move < -max_move) { move = -max_move; }
                s_out_hz += move;
            }
        }
    }

    /* 图形模式的最后一级：先把跟踪频率乘倍频系数，再限制到 DAC 可输出范围。
     * 频率测量/锁相仍以输入基频为准；这里只改变 DAC 的 y 轴轨迹频率。 */
    {
        float out_cmd = s_out_hz * TRK_OUTPUT_FREQ_MULT;

        if (out_cmd < SIGNAL_FREQ_MIN) { out_cmd = SIGNAL_FREQ_MIN; }
        if (out_cmd > SIGNAL_FREQ_MAX) { out_cmd = SIGNAL_FREQ_MAX; }

        g_trk_out_hz = (uint32_t)(out_cmd + 0.5f);

        /* ---- 第三级：死区 + 最小更新间隔，才真正去动 DAC ---- */
        {
            float    need = (s_set_hz <= 0.0f)
                          || (fabsf(out_cmd - s_set_hz) > (TRK_DEADBAND * s_set_hz));
            uint32_t now  = HAL_GetTick();

            if (need && ((uint32_t)(now - s_last_upd_ms) >= TRK_MIN_UPDATE_MS))
            {
                Signal_SetFreq(out_cmd);
                s_set_hz      = out_cmd;
                s_last_upd_ms = now;
                g_trk_updates++;
            }
        }
    }
}

/**
  * @brief  喂一个样本进检测器
  * @param  g    该样本的全局序号
  * @param  raw  原始码值（0~4095）
  */
static void trk_feed(uint32_t g, uint16_t raw)
{
    int32_t v = (int32_t)raw;

    /* ---- 峰值包络检波：快攻慢放 ----
     * 快攻：只要出现比包络更极端的值就立刻更新，一个样本就能抓住跳变。
     * 慢放：由 trk_env_update() 每 TRK_ENV_PERIOD 个样本收缩 1 码。
     * ★ 这里**不能**再写成"每 512 点的段统计 min/max" ★
     *   方波的半周期可能远长于 512 点，整段落在一个电平里会让峰峰值算成 0，
     *   被误判成"没信号"。详见 trk_env_update() 的说明。 */
    if (v < s_env_min) { s_env_min = v; }
    if (v > s_env_max) { s_env_max = v; }

    /* ---- 每周期极值：给"稳定电平"用（和上面会衰减的包络是两回事）---- */
    if (v < s_per_min) { s_per_min = v; }
    if (v > s_per_max) { s_per_max = v; }

    s_env_cnt++;
    if (s_env_cnt >= TRK_ENV_PERIOD)
    {
        s_env_cnt = 0U;
        trk_env_update();               /* 慢放 + 重算 dc/hyst/valid */
    }

    /* ---- 波形识别用的"贴轨"统计 + 占空比用的高低计数 ----
     * ★ 上轨和下轨要**分开**统计 ★
     *   一开始只统计"贴任意一轨"的总比例，结果输入是恒定电平时（只有上轨有样本）
     *   比例算出来是 1000‰，被误判成方波 —— 实测踩到过，对方信号一断，
     *   输出就悄悄从正弦变成了方波。
     *   方波的特征是"上下两轨都有大量样本"，所以判据取 min(上轨比, 下轨比)。 */
    if (s_have_dc != 0U)
    {
        int32_t pk   = s_env_max - s_env_min;
        int32_t band = pk / TRK_RAIL_BAND_DIV;

        if (pk >= TRK_MIN_PK)
        {
            if (v <= (s_env_min + band)) { s_lo_rail++; }
            if (v >= (s_env_max - band)) { s_hi_rail++; }
            s_win_cnt++;

            /* 占空比：从这次上升沿到下次上升沿之间，高/低电平各占多少样本 */
            if (v > s_dc) { s_hi_cnt++; }
            else          { s_lo_cnt++; }
        }
    }
    trk_wave_detect();

    /* ---- 上升沿过零检测（迟滞 + 武装）---- */
    if (s_have_dc != 0U)
    {
        /* 先武装：信号必须跌到 dc-hyst 以下，才允许检测下一个上升沿 */
        if (v < (s_dc - s_hyst))
        {
            s_armed = 1U;
        }

        if ((s_armed != 0U) && (s_prev <= s_dc) && (v > s_dc))
        {
            int32_t dv = v - s_prev;

            if (dv > 0)
            {
                /* 线性插值：过零点落在 s_prev 和 v 之间，比例 = (dc-prev)/dv */
                float frac = (float)(s_dc - s_prev) / (float)dv;

                if (frac < 0.0f) { frac = 0.0f; }
                if (frac > 1.0f) { frac = 1.0f; }

                trk_duty_update();      /* 一个周期刚走完，先结算占空比 */
                trk_push_cross(s_prev_g, frac);
                trk_maybe_measure();
            }
            s_armed = 0U;
        }
    }

    s_prev   = v;
    s_prev_g = g;
}

/* ==================== 波形识别 / 电平搬运 / 相位对齐 ====================
 *
 * 目标：输出与对方"同频 + 同波形 + 同电压 + 同相位"。
 *
 *   同频   —— 已有：上升沿过零插值跟踪器（开环标定实测误差 0.004%）
 *   同波形 —— 统计"贴在上下轨附近的样本比例"区分方波 / 正弦
 *   同电压 —— 把包络量出来的对方低/高电平码值**原样**交给 Signal_SetLevels()，
 *             不做任何归一化、不做缩放，所以 DAC 输出的电压范围与对方逐码一致
 *   同相位 —— ★核心★ 用 ADC 自己的**模拟看门狗 AWD1** 做硬件边沿检测
 *
 * ============================ ★★★ 一段走错的路（务必先读）★★★ ============================
 *
 *   最初的做法是"PB15 一芯两用"：把 PB15 从 GPIO_MODE_ANALOG 改成
 *   GPIO_MODE_IT_RISING，指望 ADC2_IN15 走模拟通路、EXTI15 走数字通路互不干扰，
 *   这样一根线都不用多加。**实测行不通，而且现象极具迷惑性：**
 *
 *       同一根信号线、同一个固件、只改 GPIOB->MODER 的 PB15 两位：
 *           PB15 = 00（数字输入，EXTI 需要）→ adc_buf 全 0，ADC 完全读不到
 *           PB15 = 11（模拟）              → adc_buf 全 4095，正常采到波形
 *       用 J-Link 现场改 MODER、连续三次读数验证，结果稳定复现。
 *
 *   更坑的是：数字侧看起来一切正常（IDR 读到高电平、EXTI 也真的中断过），
 *   只有 ADC 读不到 —— 所以一开始误判成"对方信号断了"，白白查了很久。
 *   **结论：这颗芯片上 ADC 输入必须保持 GPIO 模拟模式，不能借用 EXTI。**
 *
 * ============================== 改用 AWD1（正解）==============================
 *
 *   AWD（Analog Watchdog）是 ADC 内部的硬件比较器：每次转换完就把结果和
 *   [LT1, HT1] 比一下，超出窗口就置标志并发中断。它工作在模拟模式，
 *   完全不用碰 GPIO，所以和 ADC 采样天然兼容。
 *
 *   单靠 AWD 得到的是"电平"而不是"边沿"（信号在窗口外时**每个样本**都会触发），
 *   所以这里用**窗口翻转**把它变成一个施密特触发器：
 *
 *       状态 A（等上升沿）：窗口 = [0,   thr-1] → 样本 ≥ thr 时触发 = 上升穿越
 *       状态 B（等下降沿）：窗口 = [thr, 4095 ] → 样本 < thr 时触发 = 下降穿越
 *
 *   中断里只看当前是哪个状态：是"上升"就调 Signal_Realign() 把输出相位拉回
 *   上升沿，然后把窗口翻到 B；是"下降"就只把窗口翻回 A。
 *   两个状态互相纠正，即使被干扰打乱也会在 1 个样本内自己回到正确状态。
 *
 *   延迟 = 1 个 ADC 转换周期（500kSPS 下 2us）+ AWD 的 2 次连续滤波（4us）。
 *   166Hz 时相位误差 ≈ 6us/6ms × 360° ≈ 0.36°，对李萨如图形绰绰有余。
 *   ★ 滤波那 2 个样本是必须的：不加的话阈值附近的噪声会让 AWD 每个样本都中断，
 *     500kSPS 就是 50 万次中断/秒，直接压垮 CPU。
 *
 *   阈值 thr 取跟踪器量出来的直流中点 s_dc，会跟着对方信号自动调整。
 */

/**
  * @brief  设置 AWD1 的监视窗口 [lo, hi]
  * @note   整个 TR1 一起重写，所以顺带把 AWDFILT 保持成"连续 2 次"（见宏定义）。
  */
static void trk_awd_window(uint32_t lo, uint32_t hi)
{
    if (lo > 4095U) { lo = 4095U; }
    if (hi > 4095U) { hi = 4095U; }

    ADC2->TR1 = ((hi & 0xFFFU) << ADC_TR1_HT1_Pos)
              | (lo & 0xFFFU)
              | ADC_AWD_FILTERING_2SAMPLES;
}

/**
  * @brief  配置 ADC2 的模拟看门狗做"上升沿检测"
  * @note   ⚠️ 必须在 MX_ADC2_Init() / Adc_Vofa_Init() **之后**调用。
  *         ★ 本函数**不改 PB15 的 GPIO 模式** ★
  *         必须保持 GPIO_MODE_ANALOG（CubeMX 在 HAL_ADC_MspInit 里设的），
  *         改成数字输入会让 ADC 完全读不到这个引脚 —— 详见文件上半部分的说明。
  */
static void trk_awd_init(void)
{
    /* 监视通道 = 15（PB15）；只在规则组使能，注入组不用 */
    ADC2->CFGR &= ~(ADC_CFGR_AWD1CH | ADC_CFGR_JAWD1EN);
    ADC2->CFGR |= ADC_CFGR_AWD1EN | (15U << ADC_CFGR_AWD1CH_Pos);

    s_awd_arm_up = 1U;                          /* 上电先等上升沿 */
    s_awd_rearm_cyc = 0U;                       /* 时间窗关闭；开机第一条由 first 判据挡住 */
    s_awd_skip      = 0U;
    s_awd_have_cyc  = 0U;
    s_awd_ema       = 0U;
    s_last_awd_cyc  = 0U;
    trk_awd_window(0U, (s_awd_thr > 0U) ? (s_awd_thr - 1U) : 0U);

    ADC2->ISR = ADC_ISR_AWD1;                   /* 清一次残留标志 */
    ADC2->IER |= ADC_IER_AWD1IE;

    /* 优先级给最高（0）：相位对齐的精度直接取决于这个中断能多快进来 */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 0U, 0U);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
}

/**
  * @brief  更新 AWD 阈值（= 对方波形的直流中点）并重设窗口
  *
  * ★★★ 必须"改阈值"和"重设窗口"一起做，而且要做成原子的（2026-10-12 两次踩坑）★★★
  *
  *   【坑 A：只改阈值、不重设窗口 → 死锁】
  *     最初这里是"改阈值 + 按当前状态重设窗口"一起做的。
  *     后来为了躲竞态，我把它改成"只改 s_awd_thr 变量，窗口留给 ISR 翻" ——
  *     结果**死锁**：窗口初始是 [0, 2047]（阈值默认 2048），
  *     而小幅度信号（比如 pk 只有 1100）的样本永远到不了 2048，
  *     AWD 就永远不触发 → 窗口永远不翻 → 新阈值永远没机会生效。
  *     现象：g_trk_awd_irq 一直是 0、g_trk_edges 一直是 0，
  *     也就是 **Signal_Realign() 一次都没执行、相位根本没在对齐**，
  *     输出只是自由跑，相对输入一直漂 —— 用户看到的就是"几乎每秒都在跳相"。
  *
  *   【坑 B：改窗口和 ISR 抢寄存器 → 状态反掉】
  *     而如果直接改回去（不加保护），又会和 ISR 抢：
  *     主循环读到 arm==1 准备写"等上升沿"的窗口，此刻 ISR 正好把状态翻成 0，
  *     主循环仍按旧状态写回 → 窗口和状态机反了 → 下一个样本又触发一次 AWD
  *     → 多执行一次 Signal_Realign() → 输出相位被莫名复位。
  *
  *   【正解：短暂关中断，把"改阈值 + 重设窗口"做成原子操作】
  *     主循环与 ISR 不会同时执行，关掉中断（十几个周期，约 0.1us）就能保证
  *     窗口内容和 s_awd_arm_up 严格一致，两个坑同时避开。
  */
static void trk_awd_set_thr(uint32_t thr)
{
    uint32_t d;
    uint32_t primask;

    if (thr < TRK_AWD_THR_MIN) { thr = TRK_AWD_THR_MIN; }
    if (thr > TRK_AWD_THR_MAX) { thr = TRK_AWD_THR_MAX; }

    d = (thr > s_awd_thr) ? (thr - s_awd_thr) : (s_awd_thr - thr);
    if (d < 8U)
    {
        return;                         /* 抖动太小，不动（迟滞），免得频繁改窗口 */
    }

    primask = __get_PRIMASK();
    __disable_irq();                    /* ★ 原子：期间 ISR 不会插进来改状态 ★ */

    s_awd_thr = thr;
    if (s_awd_arm_up != 0U)
    {
        trk_awd_window(0U, thr - 1U);   /* 等上升沿 */
    }
    else
    {
        trk_awd_window(thr, 4095U);     /* 等下降沿 */
    }
    ADC2->ISR = ADC_ISR_AWD1;           /* 清掉这期间可能挂起的标志，保证状态一致 */

    /* ★★★ 坑 C：改窗口本身会产生一次假边沿 → 随机时刻 Signal_Realign() → 相位跳变 ★★★
     *
     *   状态机"是上升沿还是下降沿"的判断，依赖一个隐含前提：
     *     **上一个样本在窗口内**（也就是"上一刻信号还在阈值的另一侧"）。
     *   而我们在这里凭空重写窗口，这个前提就没了：
     *     状态 = 等上升沿、窗口 [0, thr-1]，若新阈值**低于**当前信号电平，
     *     下一个样本直接"超出窗口" → AWD 立刻中断 → ISR 却以为这是上升沿
     *     → 在周期的任意位置调用 Signal_Realign() → **输出相位被硬拉回表头**。
     *
     *   这就是"频率一直对、只有相位在跳"的另一个来源：
     *     trk_env_update() 每 32 个采样跑一次（15625 次/秒），阈值只要动 ≥8 码就重写；
     *     阈值随电平缓慢漂移，每秒命中"状态是等上升 + 新阈值低于当前电平"几次。
     *
     *   修法：记下"刚刚重写过窗口"的时刻。ISR 里只有**紧跟着这次改写**（30us 内）
     *   到来的第一条事件才被认定为改写引出的 → 只同步状态、不复位相位。
     *
     *   ★ 为什么必须加时间窗，而不是"置位到下次事件为止" ★
     *     那样阈值改写频繁时 skip 会常态置位，把**真**上升沿的对齐也一起吞掉，
     *     输出就退化成自由跑、相位慢慢漂。加时间窗后只有改写后 30us 内的事件被吞，
     *     一千次里丢不到两次，对锁相毫无影响。 */
    s_awd_rearm_cyc = DWT->CYCCNT;
    s_awd_skip      = 1U;

    __set_PRIMASK(primask);
}

/**
  * @brief  ★ 安全网：忽略迟滞，强制按给定阈值把窗口和状态机复位到"等上升沿" ★
  *
  *   为什么需要它：阈值/窗口一旦错位（比如阈值比信号峰值还高，样本永远进不了窗口），
  *   AWD 就永远不中断 → 窗口永远不翻 → 正常路径也没机会把它修回来 ——
  *   **死锁**。踩过一次：AWD 一次都没触发，相位完全没在对齐却没有任何报错，
  *   表现就是"几乎每秒都在跳相"。
  *
  *   所以加一道保险：只要"信号有效但 AWD 超过 20ms 没中断"，就强制重设一次。
  *   正常运行时这条永远不会触发（1kHz 信号每 0.5ms 就该中断一次）。
  */
static void trk_awd_force(uint32_t thr)
{
    uint32_t primask;

    if (thr < TRK_AWD_THR_MIN) { thr = TRK_AWD_THR_MIN; }
    if (thr > TRK_AWD_THR_MAX) { thr = TRK_AWD_THR_MAX; }

    primask = __get_PRIMASK();
    __disable_irq();

    s_awd_thr = thr;

    /* ★ 关键：状态必须跟"当前信号电平"一致，不能一律设成"等上升沿" ★
     *   如果一律设成等上升沿，而信号此刻正好是高电平，
     *   下一个样本就会立刻满足"样本 ≥ 阈值" → ISR 把它当成一次上升沿
     *   → 在错误时刻调用 Signal_Realign() → **又一次相位跳变**。
     *   自愈本来是保险，反而制造跳相就本末倒置了。
     *   所以看最后一个已处理样本 s_prev 落在哪边，据此设置状态。 */
    if (s_prev > (int32_t)thr)
    {
        s_awd_arm_up = 0U;              /* 现在就是高电平 → 等下降沿 */
        trk_awd_window(thr, 4095U);
    }
    else
    {
        s_awd_arm_up = 1U;              /* 现在低于阈值 → 等上升沿 */
        trk_awd_window(0U, thr - 1U);
    }
    ADC2->ISR = ADC_ISR_AWD1;
    s_awd_rearm_cyc = DWT->CYCCNT;
    s_awd_skip      = 1U;               /* 同样是"凭空重写窗口"，紧接着的事件不可信（见坑 C） */

    __set_PRIMASK(primask);

    g_trk_awd_heal++;
}

/**
  * @brief  ★ 看最近几个真实 ADC 样本，判定"最近一次穿越阈值"的方向 ★
  *
  * ============ 为什么必须这样做（这才是"下半沿跳上半沿"的真正根因）============
  *
  *   AWD 硬件只给出一个布尔量："这个样本在窗口外"。**它完全没有方向信息。**
  *   原先的代码靠一个状态变量交替翻转来猜方向：
  *         arm_up=1 → 这次算上升沿（复位相位）
  *         arm_up=0 → 这次算下降沿（不复位）
  *   这个推断有个致命弱点：**只要多来一个假事件、或少来一个真事件，奇偶性就永久反掉**。
  *   从此每个"上升沿"都落在真正的下降沿上 → 输出在下降半周被硬拉回表头
  *   → 示波器上就是「下半沿走到一半直接跳成上半沿」，而且**一直错到下一次抖动**。
  *   （用户原话："部分下半沿直接跳转为上半沿" —— 完全对上。）
  *
  *   任何"事后纠正奇偶性"的做法都只是打补丁。正解是**根本不去猜**：
  *   ADC 的 DMA 缓冲区里就躺着真实的样本序列，方向是直接可读的。
  *
  *   ★ 安全性：DMA 正在写的下标是 idx，本函数只读 idx-1 … idx-9 —— 全是已经写完的样本，
  *     而且要 4ms 后 DMA 才会绕回来覆盖，绝对安全（不会读到"半新半旧"的样本）。
  *
  *   ★ 顺带一次性解决两个老问题：
  *     · 主循环重写 AWD 窗口后，新窗口可能立刻把当前样本判成"出界"而凭空造出一次事件 ——
  *       这种假事件往回扫 8 个样本**找不到任何穿越**（真正的沿在半个周期以前）→ 方向不明
  *       → 不复位相位。**结构上不可能再引发跳相。**
  *     · 阈值附近抖动造成的重复事件同理，最多判成"方向不明"，绝不会被当成上升沿。
  *
  * @retval 1 = 确认上升穿越（低→高）；0 = 确认下降穿越（高→低）；0xFF = 判不出
  */
static uint8_t trk_awd_dir(void)
{
    const uint16_t *buf = Adc_Vofa_BufBase();
    uint32_t idx;
    uint32_t k;
    int32_t  thr = (int32_t)s_awd_thr;

    if (buf == NULL)
    {
        return 0xFFU;
    }

    /* 下一个待写下标 = ADC_BUF_LEN - CNDTR；最新已完成的样本是它的前一个。
     * CNDTR 刚回绕到 0 时 idx 会等于 ADC_BUF_LEN，夹到 0（取模后正好指向缓冲区末尾）✓ */
    idx = (uint32_t)ADC_BUF_LEN - (DMA1_Channel4->CNDTR & 0xFFFFU);
    if (idx >= (uint32_t)ADC_BUF_LEN)
    {
        idx = 0U;
    }

    /* 从最新往回扫，第一个跨过阈值的相邻样本对就是"最近一次穿越"。
     * 只看 8 个样本：AWDFILT 两次 + 中断响应最多占 4~6 个；
     * 再往前就会越过真正的沿、误判成上一次穿越了。 */
    for (k = 1U; k <= 8U; k++)
    {
        int32_t cur = (int32_t)buf[(idx + (uint32_t)ADC_BUF_LEN - k) % (uint32_t)ADC_BUF_LEN];
        int32_t prv = (int32_t)buf[(idx + (uint32_t)ADC_BUF_LEN - k - 1U) % (uint32_t)ADC_BUF_LEN];

        if ((prv < thr) && (cur >= thr))
        {
            return 1U;                  /* 低 → 高：确认上升穿越 */
        }
        if ((prv >= thr) && (cur < thr))
        {
            return 0U;                  /* 高 → 低：确认下降穿越 */
        }
    }
    return 0xFFU;                       /* 8 个样本内没有穿越 → 方向不明 */
}

/**
  * @brief  ★ 相位对齐的执行点 ★ ADC 模拟看门狗中断
  * @note   中断里只做寄存器操作（不重铺表），耗时约 1~2us。
  *
  * ★★★ 核心原则（这是本次彻底修好的关键）★★★
  *   **AWD 窗口只用来决定"下一次中断什么时候来"，绝不用来推断方向。
  *     方向一律由 trk_awd_dir() 从真实样本里读。**
  *
  *   于是：无论窗口状态怎么错、无论多来还是少来多少次中断，
  *   都不可能再把下降沿当成上升沿 → 输出永远不会在下降半周被拉回表头
  *   → "下半沿跳上半沿"从**结构上**消失，而不是靠概率去躲。
  */
void ADC1_2_IRQHandler(void)
{
    if ((ADC2->ISR & ADC_ISR_AWD1) != 0U)
    {
        uint32_t now = DWT->CYCCNT;
        uint8_t  dir;

        ADC2->ISR = ADC_ISR_AWD1;               /* 写 1 清标志 */
        g_trk_awd_irq++;

        /* ★ 方向来自真实样本，不来自状态机 ★ */
        dir = trk_awd_dir();

        /* ⚠️ 每个分支都必须让"当前样本落在新窗口内"，否则 AWD 会连续触发成中断风暴 */
        if (dir == 1U)
        {
            /* ---------- 确认上升穿越 → 唯一允许复位相位的时刻 ---------- */
            trk_awd_window(s_awd_thr, 4095U);   /* 当前样本 ≥ thr，落在窗口内 ✓ */
            s_awd_arm_up = 0U;

            {
                uint32_t dt = now - s_last_realign_cyc;

                /* 时间守卫：同一周期内不许复位两次（噪声在阈值附近连出两个上升穿越）。
                 * 基准 = 相邻两次复位的间隔（≈ 输入周期），一阶低通跟着对方换频走。 */
                if ((s_awd_ema > 0U) && (dt < ((s_awd_ema / 4U) * 3U)))
                {
                    g_trk_awd_glitch++;
                }
                else if (s_phaselock_on != 0U)
                {
                    Signal_Realign();           /* ★ 把输出相位硬拉回表头（相位0 = 上升沿）★ */
                    g_trk_edges++;
                    s_last_realign_cyc = now;

                    if (s_awd_ema > 0U)
                    {
                        s_awd_ema = s_awd_ema - (s_awd_ema / 8U) + (dt / 8U);
                    }
                    else
                    {
                        s_awd_ema = dt;
                    }
                }
            }
        }
        else if (dir == 0U)
        {
            /* ---------- 确认下降穿越 → 只重新武装，绝不复位相位 ---------- */
            trk_awd_window(0U, s_awd_thr - 1U); /* 当前样本 < thr，落在窗口内 ✓ */
            s_awd_arm_up = 1U;
        }
        else
        {
            /* ---------- 方向不明（改窗口引出的假事件 / 阈值附近抖动）----------
             * 按"最新一个样本在哪一侧"选窗口，保证当前样本一定在窗内，
             * 否则会立刻再触发一次 → 中断风暴。**绝不复位相位。** */
            const uint16_t *buf = Adc_Vofa_BufBase();
            uint32_t idx = (uint32_t)ADC_BUF_LEN - (DMA1_Channel4->CNDTR & 0xFFFFU);
            int32_t  last = (int32_t)s_awd_thr;

            if (idx >= (uint32_t)ADC_BUF_LEN)
            {
                idx = 0U;
            }
            if (buf != NULL)
            {
                last = (int32_t)buf[(idx + (uint32_t)ADC_BUF_LEN - 1U) % (uint32_t)ADC_BUF_LEN];
            }

            if (last >= (int32_t)s_awd_thr)
            {
                trk_awd_window(s_awd_thr, 4095U);
                s_awd_arm_up = 0U;
            }
            else
            {
                trk_awd_window(0U, s_awd_thr - 1U);
                s_awd_arm_up = 1U;
            }
            g_trk_awd_glitch++;
        }

        s_last_awd_cyc = now;
        s_awd_have_cyc = 1U;
        s_awd_skip = 0U;
    }
}

/**
  * @brief  一个完整周期结束（检测到上升沿）时更新占空比
  */
static void trk_duty_update(void)
{
    uint32_t tot = s_hi_cnt + s_lo_cnt;

    if (tot > 8U)
    {
        float d = (float)s_hi_cnt / (float)tot;

        if (s_duty_done == 0U)
        {
            s_duty      = d;
            s_duty_done = 1U;
        }
        else
        {
            s_duty += 0.25f * (d - s_duty);     /* 一阶低通 */
        }
        g_trk_duty_ppm = (uint32_t)(s_duty * 1000000.0f);
    }

    s_hi_cnt = 0U;
    s_lo_cnt = 0U;

    /* ---- 顺手结算本周期的真实极值 → 稳定的输出电平 ----
     * 这一步是修"电平抖动导致每秒重铺 170 次表"的关键，详见 s_per_min 的说明。 */
    if ((s_per_max - s_per_min) >= TRK_MIN_PK)
    {
        uint32_t lo = (s_per_min > 0) ? (uint32_t)s_per_min : 0U;
        uint32_t hi = (s_per_max > 0) ? (uint32_t)s_per_max : 0U;

        if (s_lvl_valid == 0U)
        {
            s_lvl_min   = lo;
            s_lvl_max   = hi;
            s_lvl_valid = 1U;
        }
        else
        {
            /* 一阶低通（系数 1/2）：抗单周期毛刺，
             * 对方幅度真的变了 2~3 个周期内就跟上。 */
            s_lvl_min = (uint32_t)(((int32_t)s_lvl_min + (int32_t)lo) / 2);
            s_lvl_max = (uint32_t)(((int32_t)s_lvl_max + (int32_t)hi) / 2);
        }
        g_trk_vmin = s_lvl_min;
        g_trk_vmax = s_lvl_max;
    }

    s_per_min = 4095;
    s_per_max = 0;
}

/**
  * @brief  每 TRK_DETECT_LEN 个样本判一次波形类型
  * @note   判据 = min(贴下轨比例, 贴上轨比例)，带宽是包络上下各 20%：
  *             方波  上下轨各 ≈500‰  → min ≈ 500   → 判为方波
  *             正弦  |x|>0.8 各 ≈205‰ → min ≈ 205   → 判为正弦
  *             三角  各 ≈100‰        → min ≈ 100   → 判为正弦
  *             恒定电平  只有一轨有样本 → min ≈ 0   → 判为正弦（不会误判成方波）
  *         阈值 0.35 (350‰) 能干净地把方波分出来，三种波形都成立。
  *
  * ★ 为什么必须取 min 而不是"总贴轨比例" ★
  *   只统计总数时，输入恒为高电平时所有样本都贴在上轨，比例 1000‰ → 误判成方波。
  *   实测踩过：对方信号一断，输出就从正弦悄悄变成方波，而且看不出来。
  */
static void trk_wave_detect(void)
{
    if (s_win_cnt >= TRK_DETECT_LEN)
    {
        uint32_t r_lo = (s_lo_rail * 1000U) / s_win_cnt;
        uint32_t r_hi = (s_hi_rail * 1000U) / s_win_cnt;
        uint32_t r    = (r_lo < r_hi) ? r_lo : r_hi;

        g_trk_sq_ratio = r;

        /* ★ 加迟滞，别让判据在阈值附近横跳 ★
         *   实测踩过：比例在 414‰/428‰ 附近抖，判据一过一不过，
         *   波形就在"方波/正弦"之间反复切，输出跟着一次次重铺表。
         *   所以：判成方波要 ≥350‰，但要掉到 <250‰ 才退回正弦。 */
        {
            const uint32_t hi = (uint32_t)(TRK_SQUARE_RATIO * 1000.0f);   /* 350 */
            const uint32_t lo = hi - 100U;                                /* 250 */

            if (s_detected_wave == SIG_WAVE_SQUARE)
            {
                if (r < lo) { s_detected_wave = SIG_WAVE_SINE; }
            }
            else
            {
                if (r >= hi) { s_detected_wave = SIG_WAVE_SQUARE; }
            }
        }
        g_trk_wave = (uint32_t)s_detected_wave;

        s_lo_rail = 0U;
        s_hi_rail = 0U;
        s_win_cnt = 0U;
    }
}

/**
  * @brief  把"同波形 + 同电压 + 同占空比"真正写进 signal.c
  * @note   每个参数都加了变化阈值：因为写下去可能触发重铺表（0.5ms 冻结输出），
  *         不能让它们跟着包络抖动反复写。
  */
static void trk_apply_waveform(void)
{
    uint32_t vmin;
    uint32_t vmax;
    uint8_t  need_lv = 0U;

    /* ★ 电平取"每周期真实极值"（s_lvl_min/max），不取会衰减的包络 ★
     *   取包络的话它每个周期内都漂移几十个码值，会被判成"电平变了"，
     *   于是每秒重铺上百次表，把主循环拖垮 → 丢样本 → 频率算错。
     *   详见 s_per_min 的说明。 */
    if (s_lvl_valid == 0U)
    {
        return;                         /* 还没量够一个周期，先不动输出 */
    }

    /* ★ 节流：波形/电平/占空比都是慢变量，而每写一次都可能重铺整张表
     *   （0.5~1ms，期间 TIM6 停着、输出冻结）。不节流的话这几个量的小抖动
     *   会每秒触发几十次重铺，主循环被拖过 4.1ms → DMA 套圈丢样本 → 频率算错。
     *   首次（s_app_valid==0）不节流，保证上电后能立刻匹配上。 */
    if (s_app_valid != 0U)
    {
        if ((uint32_t)(HAL_GetTick() - s_param_ms) < TRK_PARAM_MIN_MS)
        {
            return;
        }
    }
    s_param_ms = HAL_GetTick();

    vmin = s_lvl_min;
    vmax = s_lvl_max;

    /* ---- 电压：对方低/高电平码值直接搬过去 ---- */
    if (s_app_valid == 0U)
    {
        need_lv = 1U;
    }
    else
    {
        uint32_t d1 = (vmin > s_app_vmin) ? (vmin - s_app_vmin) : (s_app_vmin - vmin);
        uint32_t d2 = (vmax > s_app_vmax) ? (vmax - s_app_vmax) : (s_app_vmax - vmax);

        if ((d1 > TRK_LEVEL_EPS) || (d2 > TRK_LEVEL_EPS))
        {
            need_lv = 1U;
        }
    }
    if (need_lv != 0U)
    {
        Signal_SetLevels(vmin, vmax);
        s_app_vmin  = vmin;
        s_app_vmax  = vmax;
        s_app_valid = 1U;
    }

    /* ---- 波形类型 ---- */
    if (s_detected_wave != s_app_wave)
    {
        Signal_SetWave(s_detected_wave);
        s_app_wave = s_detected_wave;
    }

    /* ---- 占空比（只对方波有意义）---- */
    if ((s_detected_wave == SIG_WAVE_SQUARE) && (s_duty_done != 0U))
    {
        if (fabsf(s_duty - s_app_duty) > TRK_DUTY_EPS)
        {
            Signal_SetDuty(s_duty);
            s_app_duty = s_duty;
        }
    }
}

/* ============================== 对外接口 ============================== */

void Track_Init(void)
{
    const uint16_t *buf;
    uint32_t        fill;
    uint32_t        i;

    g_trk_measures = 0U;
    g_trk_updates  = 0U;
    g_trk_valid    = 0U;
    g_trk_cross_n  = 0U;
    g_trk_in_hz    = 0U;
    g_trk_out_hz   = 0U;
    g_trk_pk       = 0U;
    g_trk_resets   = 0U;
    g_trk_samples  = 0U;

    s_prev      = 2048;
    s_prev_g    = 0U;
    s_env_min   = 4095;                 /* 包络从"最窄"开始，第一个样本就会拉开 */
    s_env_max   = 0;
    s_env_cnt   = 0U;
    s_total     = 0U;
    s_last_upd_ms = HAL_GetTick();

    trk_clear();

    /* ★ 起点从"当前 DMA 写指针"开始，而不是 0 ——
     *   否则第一次会把缓冲里残留的旧数据当成新样本扫一遍，
     *   可能凑出一个假过零点。 */
    buf  = Adc_Vofa_BufBase();
    fill = ADC_BUF_LEN - __HAL_DMA_GET_COUNTER(hadc2.DMA_Handle);
    if (fill > ADC_BUF_LEN)
    {
        fill = 0U;
    }
    s_rd = fill % ADC_BUF_LEN;

    /* 顺便把 s_prev 初始化成当前最新样本，避免第一个样本就被当成跳变 */
    if (buf != 0)
    {
        uint32_t last = (s_rd + ADC_BUF_LEN - 1U) % ADC_BUF_LEN;
        s_prev = (int32_t)buf[last];
        for (i = 0U; i < 4U; i++)       /* 空转一下让编译器别把 buf 判成未使用 */
        {
            (void)buf[i];
        }
    }

    /* ---- 波形匹配 / 相位对齐 的状态复位 ---- */
    s_lo_rail     = 0U;
    s_hi_rail     = 0U;
    s_win_cnt     = 0U;
    s_hi_cnt      = 0U;
    s_lo_cnt      = 0U;
    s_duty_done   = 0U;
    s_duty        = 0.5f;
    s_detected_wave = SIG_WAVE_SINE;
    s_phaselock_on  = 0U;
    s_app_valid     = 0U;
    s_app_vmin      = 0U;
    s_app_vmax      = 0U;
    s_app_wave      = 0xFFU;
    s_app_duty      = -1.0f;

    /* 中值滤波 */
    s_med[0] = 0.0f;
    s_med[1] = 0.0f;
    s_med[2] = 0.0f;
    s_med[3] = 0.0f;
    s_med[4] = 0.0f;
    s_med_n  = 0U;
    s_med_i  = 0U;

    /* 每周期极值 / 稳定电平 / DMA 绕圈检测 */
    s_per_min    = 4095;
    s_per_max    = 0;
    s_lvl_valid  = 0U;
    s_lvl_min    = 0U;
    s_lvl_max    = 0U;
    /* DMA 落后检测的零点：当前 CPU 周期数 + 当前 DMA 写入位置。
     * ★ 一上电就对齐，别让 backlog 凭空多出来。 */
    s_last_fill  = fill;
    s_fill_valid = 1U;
    g_trk_laps   = 0U;

    /* ★ 打开 DWT 周期计数器 —— 采样时间基准靠它 ★
     *   TIM7 每 340 个 170MHz 周期触发一次 ADC，CPU 同源，
     *   所以"应该采了多少样本 = 周期数/340"是精确的。 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
    s_cyc_base  = DWT->CYCCNT;
    s_lp_cyc    = s_cyc_base;           /* 低通 dt 的起点 */
    s_last_awd_cyc = s_cyc_base;        /* AWD 自愈安全网的计时起点 */

    /* s_rd 是"下一个要处理的缓冲下标"，s_total 是对应的全局样本号。
     * 两者必须同步推进：s_rd == (初始fill + s_total) % ADC_BUF_LEN。 */
    s_total = 0U;

    /* ★ 让 signal.c 进入锁相模式：表头恒为相位 0，Signal_Realign() 才有意义 ★ */
    Signal_SetPhaseLock(1U);

    /* ★ 相位对齐：用 ADC 自己的模拟看门狗做硬件边沿检测 ★
     *   ⚠️ 这里**不碰 PB15 的 GPIO 模式** —— 必须保持模拟模式，
     *      改成数字输入会让 ADC 完全读不到这个引脚（实测，见文件上半部分的说明）。 */
    trk_awd_init();
}

void Track_Poll(void)
{
    const uint16_t *buf;
    uint32_t        fill;
    uint32_t        avail = 0U;
    uint32_t        backlog = 0U;
    uint32_t        s_target = 0U;
    uint32_t        i;
    static uint8_t  s_hold_prev = 0U;
    uint8_t         hold = (g_trk_hold_hz >= 1.0f) ? 1U : 0U;

    /* ---- 开环测试模式的进出处理 ---- */
    if (hold != s_hold_prev)
    {
        /* 进入：立刻把输出锁到指定频率；
         * 退出：清掉输出状态，让跟踪器重新接管（会平滑地跟到新频率）。 */
        if (hold != 0U)
        {
            Signal_SetFreq(g_trk_hold_hz);
            s_set_hz = g_trk_hold_hz;
        }
        s_have_out  = 0U;
        s_hold_prev = hold;
    }
    if (hold != 0U)
    {
        /* 锁定期内持续盯住（万一被别处改了频率，拉回来） */
        if (fabsf(s_set_hz - g_trk_hold_hz) > 0.5f)
        {
            Signal_SetFreq(g_trk_hold_hz);
            s_set_hz = g_trk_hold_hz;
        }
    }

    if (hadc2.DMA_Handle == 0)
    {
        return;                         /* ADC 的 DMA 还没起来 */
    }

    buf = Adc_Vofa_BufBase();
    if (buf == 0)
    {
        return;
    }

    /* DMA 本轮已写入的样本数 = 下一个要写的位置 */
    fill = ADC_BUF_LEN - __HAL_DMA_GET_COUNTER(hadc2.DMA_Handle);
    if (fill > ADC_BUF_LEN)
    {
        fill = 0U;
    }

    /* ---- ★ 落后检测：时间基准取 CPU 周期数，不数 DMA 绕圈 ★ ----
     *   target  = 从 Init 到现在"应该已经采了多少样本"（DWT 周期数 / 340，精确）
     *   backlog = target - s_total = 还没处理的积压量
     *   只要主循环跟得上，backlog 会一直很小；一旦超过一整圈，
     *   说明 DMA 把没处理的样本覆盖掉了，这时 s_total 与实际时间脱节，
     *   f=(沿数-1)*fs/span 会算错，必须清空过零历史重来。
     *
     *   ★ 为什么不能用"fill 变小就 +1 圈"来数写入量 ★
     *     两次 poll 之间若 DMA 走了整整一圈还多一点（2048+k），
     *     fill 结束时反而比上次大，那一圈就看不见了 → 少算 2048 → 频率偏高。 */
    {
        s_target = (DWT->CYCCNT - s_cyc_base) / TRK_CYC_PER_SAMPLE;
        backlog  = s_target - s_total;

        if (backlog >= ADC_BUF_LEN)
        {
            g_trk_laps++;
            trk_clear_meas();
            s_total     = s_target;     /* 时间轴对齐到当前，别让误差累积 */
            s_rd        = fill;
            s_prev      = (int32_t)buf[(fill + ADC_BUF_LEN - 1U) % ADC_BUF_LEN];
            s_prev_g    = s_total;
            s_per_min   = 4095;
            s_per_max   = 0;
            g_trk_samples += ADC_BUF_LEN;
            return;
        }
        /* ★ 留 TRK_SAFE_MARGIN 个样本不碰 ★
         *   DMA 可能正在写最后那几个，读到半新半旧的值会产生**假过零点**，
         *   假过零点会让 span 偏短 → 算出来的频率偏高。
         *   （踩过：把安全余量去掉、直接 avail=backlog 之后，
         *     测得频率里混进了约 25% 的 +13%~+22% 离群值。）
         *   留了余量后 backlog 会稳定停在这个余量上，不会再无限涨。 */
        avail = (backlog > TRK_SAFE_MARGIN) ? (backlog - TRK_SAFE_MARGIN) : 0U;
    }

    for (i = 0U; i < avail; i++)
    {
        trk_feed(s_total, buf[s_rd]);

        s_rd++;
        if (s_rd >= ADC_BUF_LEN)
        {
            s_rd = 0U;
        }
        s_total++;
    }

    g_trk_samples += avail;
}

float Track_GetInFreq(void)
{
    /* ★ 返回"平滑后、真正驱动输出的那个值"，不是原始测量值 ★
     *   原始测量里混着离群值（丢样本/假过零点造成的），
     *   直接显示出来会看着一直在抖，但那个抖动并没有传到输出上
     *   —— 它已经被斜率限制 + 时间常数低通挡掉了。
     *   想让 VOFA+ 上看原始测量做诊断，读 g_trk_raw_hz。 */
    return (s_have_out != 0U) ? s_out_hz : s_in_hz;
}

float Track_GetOutFreq(void)
{
    return s_have_out ? s_out_hz : 0.0f;
}

uint32_t Track_GetPk(void)
{
    return g_trk_pk;
}

uint32_t Track_IsValid(void)
{
    return g_trk_valid;
}
