/**
  ******************************************************************************
  * @file    signal.h
  * @brief   DAC3 + TIM6 + DMA 循环 + 内部运放 → 持续正弦波（PA2 输出）
  *
  * 用法（main.c 的 USER CODE 区）：
  *      Signal_Init();            // 放在 MX_DAC3_Init()/MX_TIM6_Init() 之后调一次
  *      Signal_SetFreq(1000.0f);  // 随时改频率，单位 Hz
  *      HAL_OPAMP_Start(&hopamp1);// ★ 必须在 Signal_Init() 之后（CubeMX 不生成这行）
  *
  * 依赖（全部由 CubeMX 生成，勿手改）：
  *      dac.c   : hdac3 / hdma_dac3_ch1, DAC3_CH1, DAC_TRIGGER_T6_TRGO,
  *                DMA1_Channel1 = Circular + Word + MemInc
  *                DAC3 的 OUT1 mode = "on chip-peripherals only"（无引脚）
  *      opamp.c : hopamp1 = Follower + NonInvertingInput=DAC + HighSpeed, PA2 = OPAMP1_VOUT
  *      tim.c   : htim6, TRGO = Update Event, AutoReloadPreload = ENABLE
  *
  * ★ 为什么是 DAC3 而不是 DAC1 ★
  *   STM32G4 的运放内部直连只支持 DAC3/DAC4，DAC1/DAC2 不参与（CubeMX 芯片库
  *   原文：OPAMP1 的条件是 "Only active when DAC3 channel out1 is enabled"）。
  *   而 G431(LQFP48) 上 DAC3 没有任何外部引脚 —— 它就是专为运放准备的内部 DAC。
  *   通路：DAC3_OUT1 ─(片内)─> OPAMP1_VINP ─> OPAMP1 跟随器 ─> PA2
  *
  * 注意：本工程 TIM6 时钟 = 170MHz（APB1 分频 = 1）。
  *       若日后改主频，必须同步改 signal.c 里的 TIM6_CLK_HZ。
  ******************************************************************************
  */

#ifndef __SIGNAL_H
#define __SIGNAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 正弦表最大点数（低频段用它换分辨率；内存占用 1024*4 = 4KB） */
#define SIN_MAX_POINTS          1024U

/* 输出频率范围 */
#define SIGNAL_FREQ_MIN         20.0f
#define SIGNAL_FREQ_MAX         100000.0f    /* 100kHz 是 DAC 物理上限附近，见下 */

/*
 * ★ 关于高频段的实测结论 ★
 *   DAC 每次更新约需 1us，所以：
 *     1k~20kHz  : 16~256 点/周期，波形漂亮
 *     20k~50kHz : 8~16 点/周期，波形尚可
 *     50k~100kHz: 4~8 点/周期，明显失真（趋近阶梯/方波）
 *   这是 DAC 的物理极限，不是代码限制。
 */

/**
  * @brief  启动正弦波输出，默认频率 1kHz
  * @note   内部自动按目标频率选好 (PSC, ARR, 点数)，再依次启动
  *         TIM6 → DAC → DMA(Circular)。调用一次即可。
  */
void            Signal_Init(void);

/**
  * @brief  设置输出频率（运行中随时可调）
  * @param  freq_hz 目标频率 Hz，自动钳位到 [20, 20000]
  * @note   换频时整条链路以正弦表首点（1.65V 过零点）重新同步，
  *         所以不会跳相、不会出现半周期毛刺。耗时约几微秒。
  */
void            Signal_SetFreq(float freq_hz);

/** @brief 取当前实际输出频率（已含 PSC/ARR 取整误差），单位 Hz */
float           Signal_GetFreq(void);

/** @brief 取当前使用的表点数（1024 / 512 / 256 / 64 / 32 / 16） */
uint32_t        Signal_GetPoints(void);

/** @brief 取正弦表首地址（RAM 表，运行时生成，32 位元素） */
uint32_t       *Signal_GetTable(void);

/**
  * @brief  【临时诊断】测量真实采样率
  * @note   用 1 秒窗口数 TIM6 的更新事件次数，精度约 0.1%。
  *         在 main.c 里替代 Signal_Init() 调用，测完看 g_meas_* 变量。
  *         实现见 signal_meas.c（诊断完可整个删掉）。
  */
void            Signal_Measure(void);

/**
  * @brief  【临时诊断】慢速正弦测试：5Hz/50Hz/1kHz 每 5 秒轮换
  * @note   5Hz 周期 200ms，波形在示波器上"爬行"，肉眼可判定是完整正弦
  *         还是半波/1/4 波。实现见 signal_slow.c（诊断完可删）。
  */
void            Signal_SlowTest(void);

/* ============================== 输出波形类型（2026-10-12 加）==============================
 * 多波形输出的思路参考了 test4_prj 工程的 mak_sin_tab.c：
 * 那里把波形生成参数化成 (mode, dots, vb, vp, sarg)，本工程沿用同样的参数含义，
 * 但表的相位处理接的是本工程自己的"相位连续"机制。
 *
 *   mode      含义
 *   --------  ----------------------------------------------------------
 *   SINE      正弦（默认）
 *   TRIANGLE  三角波
 *   SQUARE    方波（占空比可调，边沿 = 一个 DAC 更新周期，这是 DAC 能做到的最陡）
 *   SAWTOOTH  上升锯齿
 *
 * ★ 约定：表的第 0 点 = 波形相位 0 = **上升沿所在位置**。
 *   方波的表第 0 点是高电平；正弦的表第 0 点是"从负穿正的过零点"。
 *   这样 Signal_Realign() 把 DMA 挪回表头时，输出就正好从上升沿重新开始，
 *   配合 EXTI 检测到的对方上升沿，就能做到同频同相。 */
#define SIG_WAVE_SINE       0U
#define SIG_WAVE_TRIANGLE   1U
#define SIG_WAVE_SQUARE     2U
#define SIG_WAVE_SAWTOOTH   6U

/**
  * @brief  设置输出波形类型
  * @param  mode SIG_WAVE_xxx
  * @note   只置"待重铺"标志，真正的重铺在下次 Signal_SetFreq()/Signal_Realign() 时做
  */
void            Signal_SetWave(uint8_t mode);

/**
  * @brief  设置波形的高低电平（直接给 12 位码值）
  * @param  vmin 低电平码值（0~4095）
  * @param  vmax 高电平码值（0~4095）
  * @note   ★ 这是"电压一毛一样"的关键 ★
  *         对方方波的低/高电平被 ADC 量出来是多少码值，输出就用多少码值，
  *         不做任何归一化，所以 DAC 输出的电压范围与对方完全一致。
  */
void            Signal_SetLevels(uint32_t vmin, uint32_t vmax);

/**
  * @brief  设置方波的占空比
  * @param  duty 0.0~1.0，表示一个周期里高电平所占比例
  */
void            Signal_SetDuty(float duty);

/**
  * @brief  相位微调（度）
  * @param  deg 正数表示输出相位"提前" deg 度
  * @note   用来补偿"检测到对方上升沿 → 真的改变输出"之间的固定延迟。
  *         默认 0，一般不需要动。
  */
void            Signal_SetPhaseDeg(float deg);

/**
  * @brief  ★ 进入/退出"锁相模式"
  * @param  on 1 = 表头恒等于相位 0（配合 Signal_Realign() 用）；0 = 原来的相位连续行为
  * @note   必须打开才能真正做到"同相" —— 关着的时候表里会烘焙着任意相位，
  *         Realign 把 DMA 挪回表头后相位是乱的。
  */
void            Signal_SetPhaseLock(uint8_t on);

/**
  * @brief  ★★ 相位硬复位：把输出波形立刻拉回相位 0（上升沿）★★
  * @note   由 AWD 确认对方过零后调用。做的事：
  *           TIM6 停表 → DMA 关 → CNDTR 回表长、CMAR 回表头 → DMA 开
  *           → TIM6 CNT 顶到 ARR → TIM6 开表
  *         耗时约 1~2us（全是寄存器操作，不重铺表），每个输入周期调一次也不拖累 CPU。
  *         ★ 表头必须是"相位 0"，见 SIG_WAVE_xxx 的约定。
  *
  * ★ 多倍频李萨如图形（无穷）怎么用这个函数：
  *     输出频率设成输入的 N 倍，然后**在对方的每个过零点都调一次**（上升沿 + 下降沿）。
  *     N=2 时 sin(2ωt) 在 sin(ωt) 的两个过零点上都恰好过零且上升，所以两次复位都正确，
  *     而且"半个输入周期 = 一个完整输出周期"是**整除关系**，倍频被硬件锁死 ——
  *     图形完全静止。只把频率设成 2 倍是锁不住的：频率差哪怕 0.1%，
  *     每周期都会残留相位差，图形看起来就是在缓慢旋转。
  */
void            Signal_Realign(void);

/** @brief 取当前波形类型 */
uint8_t         Signal_GetWave(void);

/**
  * @brief  4Hz~700Hz 扫频（非阻塞版）
  * @note   Signal_Sweep_Init() 在 USER CODE BEGIN 2 里调一次，
  *         Signal_Sweep_Step() 在主循环里反复调，到时间自动换档。
  *         这样扫频可以和串口回发等其它任务共存。实现见 signal_slow.c。
  */
void            Signal_Sweep_Init(void);
void            Signal_Sweep_Step(void);

/**
  * @brief  【临时诊断】带记录点的启动流程，把每步返回值写到 g_trace[]
  * @note   跑完 g_trace_ready=1，用 J-Link 读 g_trace[0..15] 定位断点在哪一步。
  */
void            Signal_Trace(void);

#ifdef __cplusplus
}
#endif

#endif /* __SIGNAL_H */
