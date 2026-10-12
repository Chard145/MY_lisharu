/**
  ******************************************************************************
  * @file    debug_rtt.c
  * @brief   SEGGER RTT 调试输出实现 —— 走 SWD 内存轮询，不占 UART / 引脚
  *
  * ============================== 设计策略 ==============================
  *   1. 通道 0 作为终端，与 CMSIS 约定一致（ITM 也是通道 0）。
  *   2. printf 重定向走"强符号覆盖弱符号"：
  *        CubeMX 生成的 syscalls.c 里 _write() 是 __attribute__((weak))，
  *        本文件提供同名的强符号定义 → 链接器自动选用本文件，无需改 syscalls.c。
  *        好处：syscalls.c 被 CubeMX 重新生成也不会冲突。
  *   3. Printf 里先判缓冲余量再写，避免 SEGGER_RTT_printf 内部截断导致日志残缺。
  *
  * ============================== 为什么不改 CubeMX 生成的文件 ==============================
  *   与 uart_echo.c 同样的原则：所有逻辑收在本文件 + debug_rtt.h，
  *   CubeMX 重新生成代码时不会丢。
  ******************************************************************************
  */

#include "debug_rtt.h"
#include "SEGGER_RTT.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ============================== 诊断计数 ============================== */
volatile uint32_t g_rtt_printf_calls = 0U;   /* Printf 调用次数 */
volatile uint32_t g_rtt_bytes_out    = 0U;   /* 累计写入字节数 */
volatile uint32_t g_rtt_dropped      = 0U;   /* 缓冲满丢弃的字节数 */

/* ============================== 初始化 ============================== */
void Debug_RTT_Init(void)
{
  SEGGER_RTT_Init();

  /* 打个开机横幅：既能验证 RTT 通路，也能确认固件版本在跑 */
  SEGGER_RTT_WriteString(0,
      "\r\n"
      "========================================\r\n"
      " STM32G431C8T6  |  SEGGER RTT channel 0 \r\n"
      " 上行 4096B / 下行 1024B / NO_BLOCK_SKIP \r\n"
      "========================================\r\n");
}

/* ============================== 输出 ============================== */
unsigned Debug_RTT_Write(const void *pData, unsigned NumBytes)
{
  unsigned written;

  if ((pData == NULL) || (NumBytes == 0U))
  {
    return 0U;
  }

  written = SEGGER_RTT_Write(0, pData, NumBytes);

  g_rtt_bytes_out += written;
  if (written < NumBytes)
  {
    g_rtt_dropped += (NumBytes - written);   /* 缓冲满：记录丢弃量 */
  }

  return written;
}

int Debug_RTT_Printf(const char *sFormat, ...)
{
  char     buf[256];
  va_list  args;
  int      len;

  if (sFormat == NULL)
  {
    return 0;
  }

  /* ① 先判余量：不够就整条丢弃，绝不半截写（半截日志比没日志还难查） */
  if (SEGGER_RTT_GetAvailWriteSpace(0) < 64U)
  {
    g_rtt_dropped += 1U;
    return 0;
  }

  /* ② 格式化到栈缓冲 */
  va_start(args, sFormat);
  len = vsnprintf(buf, sizeof(buf), sFormat, args);
  va_end(args);

  if (len <= 0)
  {
    return 0;
  }

  /* ③ 截断保护 */
  if ((size_t)len >= sizeof(buf))
  {
    len = (int)sizeof(buf) - 1;
  }

  /* ④ 一次性写入（不是逐字节），效率高且不易被中断打断成碎片 */
  g_rtt_printf_calls++;
  return (int)Debug_RTT_Write(buf, (unsigned)len);
}

void Debug_RTT_Println(const char *s)
{
  if (s == NULL)
  {
    return;
  }

  Debug_RTT_Write(s, (unsigned)strlen(s));
  Debug_RTT_Write("\r\n", 2U);
}

/* ============================== printf 重定向 ==============================
 * syscalls.c 中的 _write() 是弱符号，这里提供强符号把它顶掉。
 * 链接器优先选强符号，因此不需要修改 syscalls.c 一个字。
 * ------------------------------------------------------------------------ */
#ifndef DEBUG_RTT_RETARGET_PRINTF
#define DEBUG_RTT_RETARGET_PRINTF   1
#endif

#if (DEBUG_RTT_RETARGET_PRINTF != 0)
int _write(int file, char *ptr, int len)
{
  (void)file;   /* 1=stdout, 2=stderr，本工程统一送 RTT */

  if ((ptr == NULL) || (len <= 0))
  {
    return 0;
  }

  /* _write 一次拿到整段数据，直接整块写 → 比 __io_putchar 逐字节高效得多 */
  (void)Debug_RTT_Write(ptr, (unsigned)len);

  return len;
}
#endif

/*************************** End of file ****************************/
