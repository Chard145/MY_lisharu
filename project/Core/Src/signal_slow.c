/**
  ******************************************************************************
  * @file    signal_slow.c
  * @brief   100Hz -> 50kHz 高频扫频（非阻塞版，可在主循环里和串口共存）
  *
  * 为什么改成非阻塞：
  *   原版在函数内死循环，导致后面的 Uart_Echo_Init() 永远执行不到。
  *   现在 Signal_Sweep_Init() 只做初始化，
  *   Signal_Sweep_Step() 在主循环里每次推进一步（到时间才换档），
  *   这样扫频和串口回发可以同时工作。
  *
  * ★ 为什么换成高频扫频（2026-10-12）★
  *   低频段 4~700Hz 已确认满摆，现在要查的是"高频段是否还掉电压"。
  *   为了能和 ADC/VOFA+ 通路配合，ADC 采样率同步提到了 200kHz
  *   （见 adc_vofa.h 的 ADC_VOFA_FS_HZ），否则 20kSPS 测高频只有
  *   两三个点/周期，看到的幅度完全没有意义。
  ******************************************************************************
  */

#include "signal.h"
#include "dac.h"
#include "tim.h"
#include <stdint.h>

#define SW_N        10U
#define SW_DWELL_MS 1500U       /* 每档停留 1.5 秒，一轮 15 秒 */

/* ★ 为什么从 500Hz 起步、不到 100Hz ★
 *   ADC 提到 500kSPS 后，一窗只有 2048 点 = 4.1ms。
 *   100Hz 的周期是 10ms，窗口连一个整周期都装不下 —— 测出来的峰峰值会被
 *   低估（看着像"掉了"，其实是窗口不够）。500Hz 起窗口刚好装 2 个周期，
 *   测量才可信。低频段 4~700Hz 之前已经用 20kSPS 确认过满摆，不用重复。 */
static const float f_list[SW_N] =
{
      500.0f, 1000.0f, 2000.0f, 3000.0f,  5000.0f,
     8000.0f,12000.0f,20000.0f,30000.0f, 50000.0f
};

volatile uint32_t g_freq_hz = 0U;
volatile uint32_t g_npts    = 0U;
volatile uint32_t g_idx     = 0U;

static uint32_t s_last_tick = 0U;
static uint32_t s_idx       = 0U;

/**
  * @brief  扫频初始化：启动正弦输出并设到第一档
  */
void Signal_Sweep_Init(void)
{
    Signal_Init();
    s_idx = 0U;
    Signal_SetFreq(f_list[0]);
    g_freq_hz   = (uint32_t)(f_list[0] + 0.5f);
    g_npts      = Signal_GetPoints();
    g_idx       = 0U;
    s_last_tick = HAL_GetTick();
}

/**
  * @brief  扫频推进：主循环里反复调用，到时间就换下一档
  * @note   非阻塞：不到时间就直接返回
  */
void Signal_Sweep_Step(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - s_last_tick) < SW_DWELL_MS)
    {
        return;
    }
    s_last_tick = now;

    s_idx++;
    if (s_idx >= SW_N)
    {
        s_idx = 0U;
    }

    Signal_SetFreq(f_list[s_idx]);
    g_idx     = s_idx;
    g_freq_hz = (uint32_t)(f_list[s_idx] + 0.5f);
    g_npts    = Signal_GetPoints();
}
