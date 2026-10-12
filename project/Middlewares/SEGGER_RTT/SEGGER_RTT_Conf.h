/*********************************************************************
*                   (c) SEGGER Microcontroller GmbH                  *
*                        The Embedded Experts                        *
*                           www.segger.com                           *
**********************************************************************
*                                                                    *
*        SEGGER RTT * Real Time Transfer for embedded targets        *
*                  https://github.com/SEGGERMicro/RTT                *
*                                                                    *
**********************************************************************

---------------------------END-OF-HEADER------------------------------
Purpose : User configuration file for RTT.
          For available configuration,
          refer to SEGGER_RTT_ConfDefaults.h.

----------------------------------------------------------------------
*/

#ifndef SEGGER_RTT_CONF_H
#define SEGGER_RTT_CONF_H

/*********************************************************************
*
*       Defines, configurable
*
**********************************************************************
*/

/* -------------------------------------------------------------------
 * 上行缓冲：目标 -> PC
 * 本工程按 20kHz 控制环、每周期 3 相 ×2B 估算约 120 KB/s；
 * 4KB 缓冲在 J-Link 默认 ~1ms 轮询周期下余量充足。
 * ----------------------------------------------------------------- */
#ifndef   BUFFER_SIZE_UP
  #define BUFFER_SIZE_UP                            (4096)
#endif

/* -------------------------------------------------------------------
 * 下行缓冲：PC -> 目标
 * 用于在线下命令（改 PID / 启停 / 切模式）。
 * ----------------------------------------------------------------- */
#ifndef   BUFFER_SIZE_DOWN
  #define BUFFER_SIZE_DOWN                          (1024)
#endif

/* -------------------------------------------------------------------
 * 通道数：本工程只用通道 0（终端），默认 3 上 3 下足够。
 * ----------------------------------------------------------------- */
#ifndef   SEGGER_RTT_MAX_NUM_UP_BUFFERS
  #define SEGGER_RTT_MAX_NUM_UP_BUFFERS             (3)
#endif

#ifndef   SEGGER_RTT_MAX_NUM_DOWN_BUFFERS
  #define SEGGER_RTT_MAX_NUM_DOWN_BUFFERS           (3)
#endif

/* -------------------------------------------------------------------
 * 默认模式必须是 NO_BLOCK_SKIP
 *   中断里调 RTT 时，缓冲满了要"丢弃"而不是"阻塞等待"，
 *   否则调试打印会把控制环卡死（与 ITM_SendChar 忙等是同一类坑）。
 * ----------------------------------------------------------------- */
#ifndef   SEGGER_RTT_MODE_DEFAULT
  #define SEGGER_RTT_MODE_DEFAULT                   SEGGER_RTT_MODE_NO_BLOCK_SKIP
#endif

/* -------------------------------------------------------------------
 * 临界区：Cortex-M4 = __ARM_ARCH_7EM__ → 用 basepri 屏蔽低优先级中断，
 * 保留高优先级中断，避免打印影响实时性。
 * 0x20 = 屏蔽优先级数值 >= 0x20 的中断。
 * ----------------------------------------------------------------- */
#ifndef   SEGGER_RTT_MAX_INTERRUPT_PRIORITY
  #define SEGGER_RTT_MAX_INTERRUPT_PRIORITY         (0x20)
#endif

#endif
/*************************** End of file ****************************/
