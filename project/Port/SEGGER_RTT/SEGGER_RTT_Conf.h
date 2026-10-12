/*********************************************************************
*                   (c) SEGGER Microcontroller GmbH                  *
*                        The Embedded Experts                        *
*                           www.segger.com                           *
**********************************************************************
*        SEGGER RTT * Real Time Transfer for embedded targets        *
*                  https://github.com/SEGGERMicro/RTT                *
**********************************************************************

---------------------------END-OF-HEADER------------------------------
Purpose : User configuration file for RTT  (项目自定义版)
          本文件先定义，SEGGER_RTT_ConfDefaults.h 里的 #ifndef 默认值就不会生效。

★ 放在 Port/SEGGER_RTT/ 而不是 Middlewares/SEGGER_RTT/ ★
  实测（2026-10-12）：CubeMX 重新生成代码时会把整个 Middlewares/ 目录清掉，
  我们放在那里的 RTT 中间件被删得干干净净，工程直接编译不过。
  Port/ 是用户目录，CubeMX 不碰，放这里才安全。
  同理：signal.c / adc_vofa.c / uart_echo.c / debug_rtt.c 也都放在用户目录。
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
 * 本工程走 RTT 传波形时单帧就是 POINTS×8 = 4096 字节，
 * 发送前要判"剩余空间够不够整帧"，所以缓冲必须 >= 2 帧：
 * 8192 时最多允许 2 帧在途，吞吐不会被"必须全空"卡住。
 * （默认 1024 太小，波形会一直丢帧）
 * ----------------------------------------------------------------- */
#ifndef   BUFFER_SIZE_UP
  #define BUFFER_SIZE_UP                            (8192)
#endif

/* -------------------------------------------------------------------
 * 下行缓冲：PC -> 目标（在线下命令用）
 * ----------------------------------------------------------------- */
#ifndef   BUFFER_SIZE_DOWN
  #define BUFFER_SIZE_DOWN                          (1024)
#endif

/* -------------------------------------------------------------------
 * 通道数：本工程用 0=终端日志 / 1=J-Scope / 2=VOFA+，3 上 3 下足够
 * ----------------------------------------------------------------- */
#ifndef   SEGGER_RTT_MAX_NUM_UP_BUFFERS
  #define SEGGER_RTT_MAX_NUM_UP_BUFFERS             (3)
#endif
#ifndef   SEGGER_RTT_MAX_NUM_DOWN_BUFFERS
  #define SEGGER_RTT_MAX_NUM_DOWN_BUFFERS           (3)
#endif

/* -------------------------------------------------------------------
 * 默认模式：必须 NO_BLOCK_SKIP
 * 缓冲满就整条丢弃，**绝不阻塞控制环**。
 * 若用 NO_BLOCK（HAL 默认的另一种），J-Link 一断/读得慢，SEGGER_RTT_Write
 * 就会一直等 → 主循环卡死 → 表现为"没接线就死机、接上就好"的诡异故障。
 * ----------------------------------------------------------------- */
#ifndef   SEGGER_RTT_MODE_DEFAULT
  #define SEGGER_RTT_MODE_DEFAULT                   SEGGER_RTT_MODE_NO_BLOCK_SKIP
#endif

#endif
/*************************** End of file ****************************/
