/**
  ******************************************************************************
  * @file    uart_echo.h
  * @brief   USART1 不定长收发 —— 纯轮询版（不用任何中断）
  *
  * 用法（main.c 的 USER CODE 区）：
  *      Uart_Echo_Init();        // 放在 MX_USART1_UART_Init() 之后，调一次
  *      ...
  *      while (1) {
  *          Uart_Echo_Poll();    // 主循环里反复调，内部 2ms 才真正动作一次
  *      }
  *
  * ============================== 工作策略 ==============================
  *   策略 ①（本文件默认实现）：透传 / 有多少发多少
  *     - 每 2ms 查一次 DMA 剩余量 CNDTR
  *     - 只要比上次少，就把新增部分立刻原样发出去，然后重挂 DMA
  *     - **不做帧解析**，所以不需要判断"一帧收完没有"
  *     - 好处：不堆积、延迟上限 2ms、绝不会因帧太长而撑爆缓冲
  *     - 代价：一次发送可能被拆成几段发回（PC 端拼起来看是一样的）
  *
  *   策略 ②（需要解析协议时用）：攒片段再解析
  *     把 Uart_OnChunk() 改成"追加进累积缓冲 accum[]"，
  *     等收到结束符('\n')或帧间超时才认为一帧完整，再去解析。
  *     此时才需要关心帧边界。
  *
  * 发送：HAL_UART_Transmit_DMA()，DMA 硬件搬运，不需要 TX 中断。
  *
  * 注意：本模块不改任何 CubeMX 生成的文件，全部逻辑在 uart_echo.c 里。
  ******************************************************************************
  */

#ifndef __UART_ECHO_H
#define __UART_ECHO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* DMA 接收缓冲大小。单次轮询周期内最多能收这么多字节 */
#define UART_RX_BUF_SIZE        256U

/**
  * @brief  启动接收（纯轮询：只配 DMA + DMAR 位，不开任何中断）
  * @note   放在 MX_USART1_UART_Init() 之后调用一次
  */
void Uart_Echo_Init(void);

/**
  * @brief  轮询函数：主循环里反复调用（内部限速 2ms 一次）
  */
void Uart_Echo_Poll(void);

/**
  * @brief  处理一个数据片段
  * @param  data 片段起始指针
  * @param  len  片段长度
  * @note   【策略①】默认实现是原样转发出去。
  *         改成【策略②】时，在这里把 data 追加进累积缓冲即可（挂载点）。
  */
void Uart_OnChunk(const uint8_t *data, uint16_t len);

/* ============================== 诊断计数（J-Link 可直接读）============================== */
extern volatile uint32_t g_uart_chunks;      /* 取走并转发的片段数 */
extern volatile uint32_t g_uart_rx_bytes;    /* 累计收到的字节数 */
extern volatile uint32_t g_uart_last_len;    /* 最近一个片段的长度 */
extern volatile uint32_t g_uart_tx_frames;   /* 实际发出的片段数 */
extern volatile uint32_t g_uart_tx_busy;     /* 上次没发完而丢弃的片段数 */
extern volatile uint32_t g_uart_errors;      /* 溢出/帧错误次数 */
extern volatile uint32_t g_uart_polls;       /* 轮询执行次数 */

#ifdef __cplusplus
}
#endif

#endif /* __UART_ECHO_H */
