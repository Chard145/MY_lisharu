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

/* 滑动窗口最多保留几个上升沿。太多会让高频段反应变慢，
 * 12 个 ≈ 11 个周期，对 50kHz 来说只有 220us，够快。 */
#define TRK_MAX_CROSS       12U

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
/* 变化超过这个比例视为"换频"，直接跟上去（不做低通，否则跟不上跳变） */
#define TRK_JUMP_RATIO      0.25f
/* 一阶低通系数。越小越稳但越慢 */
#define TRK_LP_ALPHA        0.35f
/* 相对死区：输出值变化小于 0.1% 就不去动 DAC。
 * ★ 必须要有：Signal_SetFreq() 内部会重铺正弦表并把相位复位，
 *   调用太频繁会在输出上留下周期性毛刺。 */
#define TRK_DEADBAND        0.001f
/* 两次真正 SetFreq 之间的最小间隔(ms)，进一步限制相位复位带来的扰动 */
#define TRK_MIN_UPDATE_MS   1U

/* 算出来的频率超出这个范围就当作误检丢弃 */
#define TRK_FREQ_LO         (SIGNAL_FREQ_MIN * 0.5f)
#define TRK_FREQ_HI         (SIGNAL_FREQ_MAX * 1.5f)

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

    g_trk_valid = 1U;
}

/**
  * @brief  取环形历史里第 i 个（0 = 最老的）时间戳下标
  */
static uint8_t trk_slot(uint8_t i)
{
    return (uint8_t)((s_cw + TRK_MAX_CROSS - s_cn + i) % TRK_MAX_CROSS);
}

/**
  * @brief  记录一个上升沿时刻
  * @param  idx  该沿所在的前一个样本的全局序号（沿位于 idx 和 idx+1 之间）
  * @param  frac 插值得到的小数位置，[0,1)
  */
static void trk_push_cross(uint32_t idx, float frac)
{
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

    s_in_hz     = f;
    g_trk_measures++;
    g_trk_in_hz = (uint32_t)(f + 0.5f);

    /* ---- 开环测试模式：只报告测量值，绝不碰输出 ---- */
    if (g_trk_hold_hz >= 1.0f)
    {
        g_trk_out_hz = (uint32_t)(g_trk_hold_hz + 0.5f);
        return;
    }

    /* ---- 第二级：换频检测 / 一阶低通 ---- */
    if (s_have_out == 0U)
    {
        s_out_hz   = f;
        s_have_out = 1U;
    }
    else if (fabsf(f - s_out_hz) > (TRK_JUMP_RATIO * s_out_hz))
    {
        s_out_hz = f;                   /* 对方换频了，直接跟上，别被低通拖住 */
    }
    else
    {
        s_out_hz += TRK_LP_ALPHA * (f - s_out_hz);
    }

    if (s_out_hz < SIGNAL_FREQ_MIN) { s_out_hz = SIGNAL_FREQ_MIN; }
    if (s_out_hz > SIGNAL_FREQ_MAX) { s_out_hz = SIGNAL_FREQ_MAX; }

    g_trk_out_hz = (uint32_t)(s_out_hz + 0.5f);

    /* ---- 第三级：死区 + 最小更新间隔，才真正去动 DAC ---- */
    {
        float    need = (s_set_hz <= 0.0f)
                      || (fabsf(s_out_hz - s_set_hz) > (TRK_DEADBAND * s_set_hz));
        uint32_t now  = HAL_GetTick();

        if (need && ((uint32_t)(now - s_last_upd_ms) >= TRK_MIN_UPDATE_MS))
        {
            Signal_SetFreq(s_out_hz);
            s_set_hz      = s_out_hz;
            s_last_upd_ms = now;
            g_trk_updates++;
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

    s_env_cnt++;
    if (s_env_cnt >= TRK_ENV_PERIOD)
    {
        s_env_cnt = 0U;
        trk_env_update();               /* 慢放 + 重算 dc/hyst/valid */
    }

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

                trk_push_cross(s_prev_g, frac);
                trk_maybe_measure();
            }
            s_armed = 0U;
        }
    }

    s_prev   = v;
    s_prev_g = g;
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
}

void Track_Poll(void)
{
    const uint16_t *buf;
    uint32_t        fill;
    uint32_t        avail;
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

    /* 从 s_rd 到 fill 之间新增的样本数（环形减法）。
     * ⚠️ 前提是两次调用之间 DMA 没有整整绕一圈以上 ——
     *    主循环最慢也就几百微秒，绕一圈要 4.1ms，安全。 */
    avail = (fill + ADC_BUF_LEN - s_rd) % ADC_BUF_LEN;

    /* 尾部留一点不碰，等 DMA 写完那几个样本 */
    if (avail > (ADC_BUF_LEN - TRK_SAFE_MARGIN))
    {
        avail = ADC_BUF_LEN - TRK_SAFE_MARGIN;
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
    return s_in_hz;
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
