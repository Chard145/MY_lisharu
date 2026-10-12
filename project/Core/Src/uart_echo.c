/**
  ******************************************************************************
  * @file    uart_echo.c
  * @brief   USART1 不定长收发 —— 【纯轮询版：轮询 DMA 剩余量，有多少发多少】
  *
  * ============================== 设计策略 ==============================
  *   本文件实现的是「策略 ①：透传 / 无脑转发」，刻意**不做帧解析**：
  *     - 每 2ms 轮询一次 DMA 的剩余搬运量 CNDTR
  *     - 只要发现"比上次少了"，说明又来数据了
  *     - 把新增的这部分**立刻原样发出去**，然后重挂 DMA
  *
  *   ★ 为什么这里可以不管"收完没收完" ★
  *     因为不解析、不需要帧边界。对方发 10 个字节，可能分 2~3 次被取走发回，
  *     但 PC 端串口助手把字节按顺序拼起来，看起来就是完整的 10 个字节。
  *     代价只是"一次发送被拆成几段"，对透传完全无害。
  *
  *   ★ 这样做的好处 ★
  *     - 不会堆积：每次轮询就把 DMA 里的数据清空，内存占用恒定
  *     - 延迟有上限：最坏 2ms（轮询周期），不会越积越多
  *     - 绝不丢帧：不存在"帧太长把缓冲撑爆"的问题
  *
  * ============================== 如果要解析协议 ==============================
  *   请用「策略 ②：攒片段再解析」，做法是：
  *     - 把每次取到的片段追加进一个累积缓冲 accum[]，而不是立刻发走
  *     - 满足下列任一条件才认为"一帧完整"，再去解析：
  *         a) 收到结束符（比如 '\n'）
  *         b) 距最后一次收到数据超过 N ms（帧间超时）
  *         c) 累积长度超过某个上限（保护）
  *     - 解析完清空累积缓冲
  *   这时才需要关心"帧边界"，因为要先拿到完整一帧才能解析。
  *   本文件里的 Uart_OnChunk() 就是留给策略②的挂载点。
  *
  * ============================== 发送 ==============================
  *   用 HAL_UART_Transmit_DMA()，由 DMA 硬件搬运，**不需要 TX 中断**。
  *   发送前查一下 huart1.gState 是否 READY 即可（没发完就丢弃本次，计数）。
  ******************************************************************************
  */

#include "uart_echo.h"
#include "usart.h"
#include <string.h>

/* ============================== 缓冲 ============================== */
static uint8_t rx_buf[UART_RX_BUF_SIZE];     /* DMA 接收缓冲 */
static uint8_t tx_buf[UART_RX_BUF_SIZE];     /* 发送缓冲（与接收分开，避免被 DMA 改写） */

/* ============================== 诊断计数 ============================== */
volatile uint32_t g_uart_chunks    = 0U;     /* 取走并转发的"片段"数 */
volatile uint32_t g_uart_rx_bytes  = 0U;     /* 累计收到的字节数 */
volatile uint32_t g_uart_last_len  = 0U;     /* 最近一个片段的长度 */
volatile uint32_t g_uart_tx_frames = 0U;     /* 实际发出的片段数 */
volatile uint32_t g_uart_tx_busy   = 0U;     /* 上次发送未完成而丢弃的片段数 */
volatile uint32_t g_uart_errors    = 0U;     /* 溢出/帧错误次数 */
volatile uint32_t g_uart_polls     = 0U;     /* 轮询执行次数（确认主循环在跑） */

/* ============================== 内部状态 ============================== */
static uint32_t s_last_poll_tick = 0U;

#define UART_POLL_MS    2U                   /* 轮询周期：2ms */

/**
  * @brief  重新挂起 DMA 接收（重设指针和长度，清标志）
  * @note   必须在取走数据之后立刻调用，否则 DMA 会继续往后写
  */
static void uart_rearm(void)
{
    DMA_Channel_TypeDef *ch = (DMA_Channel_TypeDef *)huart1.hdmarx->Instance;

    ch->CCR  &= ~DMA_CCR_EN;
    ch->CMAR  = (uint32_t)rx_buf;
    ch->CNDTR = UART_RX_BUF_SIZE;
    DMA1->IFCR = 0x0FUL << 4;            /* 清 DMA1 通道2 的全部标志 */
    ch->CCR  |= DMA_CCR_EN;
}

/**
  * @brief  启动接收：配置 DMA + DMAR 位，**不开任何中断**
  */
void Uart_Echo_Init(void)
{
    DMA_Channel_TypeDef *ch = (DMA_Channel_TypeDef *)huart1.hdmarx->Instance;

    g_uart_chunks    = 0U;
    g_uart_rx_bytes  = 0U;
    g_uart_last_len  = 0U;
    g_uart_tx_frames = 0U;
    g_uart_tx_busy   = 0U;
    g_uart_errors    = 0U;
    g_uart_polls     = 0U;

    /* 1) 确保空闲中断关闭 —— 本方案不用中断来判帧，靠轮询 */
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);

    /* 1b) ★ 千万不要关 USART1 的 NVIC 中断使能 ★
     *   实测踩坑记录（同一行代码，两个方向各错一次）：
     *
     *   ① 中断开着时死机：
     *      上一步把 RXNEIE 关了，但 HAL_UART_IRQHandler() 里 RXNE 的处理分支
     *      要求 "(isrflags & RXNE) && (cr1its & RXNEIE)" —— RXNEIE=0 时
     *      这个分支直接跳过，**RXNE 标志清不掉**，中断无限重入。
     *      现象：PC 永久停在 HAL_UART_IRQHandler 里，程序完全不跑。
     *
     *   ② 中断关掉时只能发一帧：
     *      HAL 的 UART_DMATransmitCplt()（DMA Normal 模式分支）只做
     *        "TxXferCount=0; 清 CR3.DMAT; 置 CR1.TCIE"，
     *      **它不把 gState 恢复成 READY** —— 那是靠随后产生的
     *      USART1 TC 中断 -> UART_EndTransmit_IT() 来完成的。
     *      所以一旦关了 USART1 的 NVIC，gState 会永久停在 BUSY_TX，
     *      之后 HAL_UART_Transmit_DMA() 全部返回 HAL_BUSY，只能发出第一帧。
     *
     *   结论：**USART1 的 NVIC 中断必须保持使能**（TC 要用），
     *         而 RXNE 标志由 Uart_Echo_Poll() 里的清标志动作兜住（见那边注释）。
     */
    /* 注意：这里刻意不调用 HAL_NVIC_DisableIRQ(USART1_IRQn) */

    /* 2) 配置 DMA1_Channel2 做 USART1 接收：外设->内存、字节、内存递增 */
    ch->CCR  &= ~DMA_CCR_EN;
    ch->CPAR  = (uint32_t)&(USART1->RDR);
    ch->CMAR  = (uint32_t)rx_buf;
    ch->CNDTR = UART_RX_BUF_SIZE;
    DMA1->IFCR = 0x0FUL << 4;
    ch->CCR  |= DMA_CCR_EN;

    /* 3) 使能 USART1 的 DMA 接收请求，并清掉遗留标志 */
    USART1->ICR = USART_ICR_IDLECF | USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;
    SET_BIT(USART1->CR3, USART_CR3_DMAR);

    s_last_poll_tick = HAL_GetTick();
}

/**
  * @brief  处理一个数据片段（默认：原样转发出去）
  * @param  data 片段起始指针
  * @param  len  片段长度
  * @note   【策略① 透传】直接发走。
  *         若要实现【策略② 攒片段再解析】，把这里改成把 data 追加进
  *         累积缓冲 accum[]，等结束符/超时再解析即可 —— 挂载点就在这里。
  */
void Uart_OnChunk(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U))
    {
        return;
    }
    if (len > UART_RX_BUF_SIZE)
    {
        len = UART_RX_BUF_SIZE;
    }

    /* 拷贝到发送缓冲：重挂接收后 DMA 会立刻改写 rx_buf，不能直接拿它去发 */
    memcpy(tx_buf, data, len);

    /* 发送前确认上一次发送已完成（不依赖 TX 中断，直接查 HAL 状态） */
    if (huart1.gState != HAL_UART_STATE_READY)
    {
        g_uart_tx_busy++;                /* 上一段还在发，本段丢弃并计数 */
        return;
    }

    if (HAL_UART_Transmit_DMA(&huart1, tx_buf, len) == HAL_OK)
    {
        g_uart_tx_frames++;
    }
    else
    {
        g_uart_tx_busy++;
    }
}

/**
  * @brief  轮询函数：主循环里反复调用，每 2ms 执行一次
  * @note   判据 = DMA 剩余搬运量 CNDTR 相比上次是否减少（有数据进来）
  *         取走多少就转发多少，不等帧边界 —— 见文件头的策略说明
  */
void Uart_Echo_Poll(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - s_last_poll_tick) < UART_POLL_MS)
    {
        return;
    }
    s_last_poll_tick = now;
    g_uart_polls++;

    /* ---- 0) ★ 先清能通过 ICR 清除的标志 ★
     *   为什么要在这里清：
     *     USART1 的 NVIC 中断必须开着（DMA 发送完成后靠 TC 中断把 huart1.gState
     *     复位成 READY，不开就永远发不出第二帧）。
     *     但 RXNEIE 被我们关了，HAL_UART_IRQHandler() 里 RXNE 的分支要求
     *     RXNEIE!=0 才处理 —— 于是 RXNE 标志没人清，中断会无限重入把 CPU 卡死。
     *
     *   本步骤的作用：
     *     ICR 的各位是"写 1 清除"，**读 RDR 的动作完全不做**，
     *     所以不会偷走 DMA 正在接收的字节，对 DMA 接收毫无影响。
     *     把 PE/FE/ORE/NE/IDLE/RTOF 这些错误/空闲标志清掉，
     *     可以保证 IRQHandler 里的"错误分支"不会反复进入。
     *
     *   注意：**唯独 RXNE 不能在这里清**（清它只能读 RDR，会破坏 DMA 收数据）。
     *   好在本工程是"PC 发什么就回什么"，RX 侧基本没有真实流量；
     *   而 CR3.DMAR=1 时 RXNE 一旦被 DMA 取走就会自动清除。
     *   真遇到 PC 持续狂发导致 RXNE 常置位，就在 CubeMX 里把 USART1 的
     *   NVIC 中断关掉 —— 但那会牺牲 DMA 发送的完成回调（本文开头①/②两条坑）。 */
    USART1->ICR = USART_ICR_PECF | USART_ICR_FECF | USART_ICR_ORECF
                | USART_ICR_NECF | USART_ICR_IDLECF | USART_ICR_RTOCF;

    /* ---- 1) 溢出/帧错误：清标志并重挂，避免串口"死掉" ---- */
    if ((USART1->ISR & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) != 0U)
    {
        g_uart_errors++;
        USART1->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;
        USART1->ICR = USART_ICR_IDLECF;
        uart_rearm();
        return;
    }

    /* ---- 2) 看 DMA 已经搬进来多少 ---- */
    {
        DMA_Channel_TypeDef *ch = (DMA_Channel_TypeDef *)huart1.hdmarx->Instance;
        uint32_t remain = ch->CNDTR;
        uint32_t len;

        if (remain > UART_RX_BUF_SIZE)
        {
            return;                      /* 异常值，忽略 */
        }

        len = UART_RX_BUF_SIZE - remain; /* 本周期新搬进来的字节数 */

        if (len == 0U)
        {
            return;                      /* 没新数据 */
        }

        /* ---- 3) 有多少发多少，不做帧边界判断 ---- */
        g_uart_chunks++;
        g_uart_rx_bytes += len;
        g_uart_last_len  = len;

        Uart_OnChunk(rx_buf, (uint16_t)len);

        /* ---- 4) 立刻重挂，让 DMA 从头开始收下一批 ----
         * 注意：重挂时机在 Uart_OnChunk 之后。
         * Uart_OnChunk 内部已经把数据拷贝到 tx_buf，
         * 所以这里改写 rx_buf 不会影响正在发送的内容。 */
        uart_rearm();
    }
}
