/**
  ******************************************************************************
  * @file    signal_meas.c
  * @brief   【临时诊断】精确测�?DAC 的真实采样率（用 SysTick 当秒表数触发次数�?  *
  * 为什么需要：
  *   J-Link 读寄存器测不出速率（每�?SWD 读约 3ms，搬运动作是 4us 级，追不上）�?  *   所以把"秒表"搬进芯片内部：用 SysTick 计时间窗口，用轮询数 TIM6 的更�?  *   事件（UIF）次数，两者相除就是精确采样率，精度约 0.1%�?  *
  * 原理�?  *   TIM6 每次更新事件�?UIF (TIM6->SR bit0)。轮询并清除该位�?  *   在约 1 秒窗口内累计置位次数 K，则
  *        f_采样 = K * 1000 / 实际毫秒�?  *        f_输出 = f_采样 / N
  *
  * 用法�?  *   main.c �?USER CODE BEGIN 2 里用  Signal_Measure();  替代 Signal_Init();
  *   跑完 g_meas_ready = 1，用 J-Link 读下列变量�?  *
  * 判读�?kHz 设定下）�?  *   g_meas_fs �?256000  �?一切正常，1/4 波是示波器显示问�?  *   g_meas_fs �? 64000  �?定时器实际时钟只�?42.5MHz�?/4 波的真凶
  ******************************************************************************
  */

#include "signal.h"
#include "dac.h"
#include "tim.h"
#include <stdint.h>

/* ===== 测量结果（volatile，J-Link 按符号地址读） ===== */
volatile uint32_t g_meas_ready   = 0U;   /* 测完�?1 */
volatile uint32_t g_meas_count   = 0U;   /* 窗口�?UIF 置位次数 */
volatile uint32_t g_meas_ms      = 0U;   /* 时间窗口（毫秒） */
volatile uint32_t g_meas_fs      = 0U;   /* 实测采样�?Hz */
volatile uint32_t g_meas_npts    = 0U;   /* 当前表点�?N */
volatile uint32_t g_meas_fo      = 0U;   /* 实测输出频率 Hz */
volatile uint32_t g_meas_cpu_hz  = 0U;   /* SystemCoreClock */
volatile uint32_t g_meas_dwt     = 0U;   /* DWT 是否可用�?=可用�?*/

#define MEAS_WINDOW_MS      1000U

/**
  * @brief  使能 DWT 周期计数器（顺带验证内核调试单元可访问）
  */
static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    g_meas_dwt = ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U) ? 1U : 0U;
}

/**
  * @brief  启动正弦输出并测量真实采样率（需要约 1 秒）
  */
void Signal_Measure(void)
{
    uint32_t t0;
    uint32_t elapsed;
    uint32_t count = 0U;

    /* 1) 正常启动整条链路（TIM6 + DAC + DMA�?*/
    Signal_Init();

    /* 2) 关掉 DMA 中断使能，避�?HAL 清标志动作干扰轮�?*/
    if (hdac3.DMA_Handle1 != NULL)
    {
        __HAL_DMA_DISABLE_IT(hdac3.DMA_Handle1, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);
    }
    DMA1->IFCR = 0x0FUL;                /* 清通道1 全部标志，从干净状态起�?*/

    dwt_init();

    /* 3) 开一个精确的毫秒窗口，边轮询边清 UIF 边计�?*/
    t0 = HAL_GetTick();

    while (1)
    {
        if ((TIM6->SR & 0x01U) != 0U)
        {
            TIM6->SR = 0U;              /* �?UIF */
            count++;
        }

        elapsed = HAL_GetTick() - t0;
        if (elapsed >= MEAS_WINDOW_MS)
        {
            break;
        }
    }

    /* 4) 汇�?*/
    g_meas_count  = count;
    g_meas_ms     = elapsed;
    g_meas_npts   = Signal_GetPoints();
    g_meas_cpu_hz = (uint32_t)SystemCoreClock;

    if (elapsed != 0U)
    {
        g_meas_fs = (count * 1000U) / elapsed;
    }
    if (g_meas_npts != 0U)
    {
        g_meas_fo = g_meas_fs / g_meas_npts;
    }

    g_meas_ready = 1U;

    while (1)
    {
        /* 停在这里，方�?J-Link 读变�?*/
    }
}
