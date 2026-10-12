/**
  ******************************************************************************
  * @file    debug_rtt.h
  * @brief   SEGGER RTT 调试输出 —— 走 SWD 内存轮询，不占任何 UART / 引脚
  *
  * 用法（main.c 的 USER CODE 区）：
  *      Debug_RTT_Init();                 // 放在初始化最前，调一次即可
  *      Debug_RTT_Printf("angle=%d\r\n", (int)angle);
  *      printf("也可以直接用 printf\r\n"); // 已重定向到 RTT
  *
  * ============================== 与 UART 的分工 ==============================
  *   UART  (uart_echo.c)  → 产品/现场用的真实数据通道（VOFA+ 波形、协议收发）
  *   RTT   (本文件)       → 调试期的日志通道（printf、状态打印）
  *   两者完全独立，互不干扰：RTT 不需要占引脚，也永远不会和 DMA 抢缓冲。
  *
  * ============================== 为什么用 RTT 而不是串口打印 ==============================
  *   - 不占 USART 外设、不多接一根线
  *   - 双向：PC 可以往目标发命令（改 PID / 启停），比 SWO 单向强
  *   - 带环形缓冲，不丢包；日志可在 CPU 全速运行时被读走
  *
  * ============================== 注意 ==============================
  *   1. 缓冲满了会"静默丢弃"（NO_BLOCK_SKIP 模式）——这是有意为之，
  *      保证绝不阻塞控制环。用 Debug_RTT_GetDropped() 可以查丢了多少。
  *   2. 中断里可以用，但别刷屏：单次写入应远小于缓冲，否则丢包。
  *   3. 本模块不改任何 CubeMX 生成的文件，全部逻辑在 debug_rtt.c 里。
  ******************************************************************************
  */

#ifndef __DEBUG_RTT_H
#define __DEBUG_RTT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 让调用方 include 本头文件后就能直接用 printf（本模块已把它重定向到 RTT），
 * 不必再记着 #include <stdio.h>。*/
#include <stdio.h>

/**
  * @brief  初始化 RTT（调一次，越早越好）
  * @note   放在 main() 里 MX_xxx_Init() 之前或之后都行，
  *         但必须在任何 Debug_RTT_Printf() 之前。
  */
void Debug_RTT_Init(void);

/**
  * @brief  格式化输出到 RTT 通道 0（支持 %f，因为打开了浮点格式化）
  * @param  sFormat  格式串
  * @return 实际写入的字节数
  */
int Debug_RTT_Printf(const char *sFormat, ...);

/**
  * @brief  原样输出一段数据（二进制/无结束符时用，效率高于 Printf）
  */
unsigned Debug_RTT_Write(const void *pData, unsigned NumBytes);

/**
  * @brief  输出一个 C 字符串（自动带换行）
  */
void Debug_RTT_Println(const char *s);

/* ============================== 诊断计数（J-Link 可直接读）============================== */
extern volatile uint32_t g_rtt_printf_calls;   /* Printf 调用次数 */
extern volatile uint32_t g_rtt_bytes_out;      /* 累计写入 RTT 的字节数 */
extern volatile uint32_t g_rtt_dropped;        /* 因缓冲满而丢弃的字节数（>0 说明刷太快了） */

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_RTT_H */
