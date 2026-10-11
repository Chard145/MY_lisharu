/**
  ******************************************************************************
  * @file    signal_slow.c
  * @brief   【诊断】4Hz -> 700Hz 慢速扫频，每档 2 秒循环
  *
  * 为什么选这个范围：
  *   实测 PA4 的模拟通路带宽只有 fc ≈ 600Hz（一阶低通），
  *   4~700Hz 基本还在满摆区，波形干净好看。
  *
  * 用 J-Link 读 g_freq_hz 可知当前档位。
  ******************************************************************************
  */

#include "signal.h"
#include "dac.h"
#include "tim.h"
#include <stdint.h>

#define SW_N    20U

static const float f_list[SW_N] =
{
      4.0f,   8.0f,  16.0f,  25.0f,  40.0f,
     60.0f,  80.0f, 100.0f, 130.0f, 160.0f,
    200.0f, 240.0f, 280.0f, 320.0f, 380.0f,
    440.0f, 500.0f, 560.0f, 630.0f, 700.0f
};

volatile uint32_t g_freq_hz = 0U;
volatile uint32_t g_npts    = 0U;
volatile uint32_t g_idx     = 0U;

void Signal_Trace(void)
{
    volatile uint32_t spin;

    Signal_Init();

    while (1)
    {
        uint32_t i;
        for (i = 0U; i < SW_N; i++)
        {
            g_idx     = i;
            g_freq_hz = (uint32_t)(f_list[i] + 0.5f);

            Signal_SetFreq(f_list[i]);
            g_npts = Signal_GetPoints();

            /* 每档约 2 秒 */
            for (spin = 0U; spin < 4500000U; spin++)
            {
                __NOP();
            }
        }
    }
}
