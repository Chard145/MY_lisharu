/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "signal.h"          /* DAC+TIM6+DMA  */
#include "uart_echo.h"       /* USART1 不定长收发（DMA + 空闲中断） */
#include "adc_vofa.h"        /* ADC2(500kSPS, TIM7触发) -> VOFA+ (JustFloat) */
#include "track.h"           /* ★ 频率跟踪：ADC 采对方的波 -> 测频 -> DAC 输出同频正弦 */
#include "debug_rtt.h"       /* SEGGER RTT 调试输出（走 SWD，不占 UART/引脚） */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  MX_TIM7_Init();
  MX_DAC3_Init();
  MX_OPAMP1_Init();
  /* USER CODE BEGIN 2 */
  /* RTT 调试通道：越早初始化越好，这样后面所有 printf 都能被 J-Link 抓走。
   * 不需要任何接线改动（走原有 SWD：PA13/PA14），也不占 USART。 */
  Debug_RTT_Init();
  Debug_RTT_Println("boot: RTT ready, init peripherals...");

  /* Start the sine output. 通路：DAC3 ─(片内)─> OPAMP1 跟随器 ─> PA2
   *
   * ★ 只有一件事必须自己做：CubeMX 只生成 MX_OPAMP1_Init()，**不会生成启动**。
   *   HAL_OPAMP_Init() 的源码注释写得很清楚：
   *       "(#) Enable the OPAMP using HAL_OPAMP_Start() function."
   *   Init 只把参数写进 OPAMP_CSR，运放本体还是关着的 → PA2 上什么都没有。
   *
   * ★ 顺序：HAL_OPAMP_Start() 必须在 Signal_Init() 之后 ——
   *   DAC3 要先跑起来，运放跟随的才不是 0V。
   *
   * 注：G4 的 OPAMP 没有独立时钟门控（CubeMX 里该 IP 是 ClockEnableMode="none"，
   *     芯片头文件里也没有 __HAL_RCC_OPAMP_CLK_ENABLE 这个宏），所以不需要开时钟。 */
  /* ★★ 跟频系统：输出频率由 Track_Poll() 根据"对方的波"实时决定 ★★
   *   所以这里**不能**再调 Signal_Sweep_Init() 的扫频 ——
   *   扫频和跟踪器会抢着调 Signal_SetFreq()，输出会变成一团乱跳。
   *   要扫频做失真度对比时，把下面 Signal_Init() 换回 Signal_Sweep_Init() 即可。 */
  Signal_Init();                /* 启动 DAC3，先给一个 1kHz 的默认输出 */
  HAL_OPAMP_Start(&hopamp1);    /* ★ 运放启动：必须晚于 DAC3 */

  Uart_Echo_Init();             /* 启动 USART1 不定长接收（DMA + 空闲中断） */
  Adc_Vofa_Init();              /* 启动 ADC2 采集(500kSPS) + DMA，发 VOFA+ */
  Track_Init();                 /* ★ 跟踪器：必须在 Adc_Vofa_Init() 之后（复用它的 adc_buf） */
  Debug_RTT_Println("boot: all modules started (frequency tracking mode)");

  /* ---- RTT 调试输出用法（不需要时可删）---------------------------------
   * printf 已被重定向到 RTT，J-Link RTT Viewer 里直接能看到：
   *     printf("angle=%d  id=%.3f  iq=%.3f\r\n", (int)angle, id, iq);
   *     Debug_RTT_Printf("freq=%.2f Hz\r\n", Signal_GetFreq());
   * 注意：别在 20kHz 中断里刷屏，缓冲满了会静默丢弃
   *       （可用 g_rtt_dropped 查看丢了多少）。
   * -------------------------------------------------------------------- */
  printf("RTT printf OK: %d %.3f\r\n", 42, 3.14159f);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* ★ 注意：这里**故意不调 Signal_Sweep_Step()** ★
     *   它的 s_last_tick 初值是 0，就算没调 Init，第一次进函数
     *   (HAL_GetTick() - 0) >= 1500 立刻成立 → 马上 Signal_SetFreq(500Hz)，
     *   然后每 1.5 秒换一档，把跟踪器的输出冲掉。
     *   所以跟频模式下必须整个摘掉它。
     *   要回去做扫频失真度对比：把 Signal_Init() 换成 Signal_Sweep_Init()、
     *   这里加回 Signal_Sweep_Step()、并把 Track_Poll() 注释掉。 */
    Uart_Echo_Poll();           /* 串口轮询：每 2ms 取走 DMA 收到的数据并回发 */
    Track_Poll();               /* ★ 跟频核心：测对方的频率 → 驱动 DAC 输出同频正弦 */
    Adc_Vofa_Poll();            /* ADC 轮询：取一整窗波形降采样后发到 VOFA+ 示波器 */

    /* ★ 这里**不要**再往 RTT 打周期性的文本日志 ★
     *   ADC_VOFA_LINK=1 时，VOFA+ 的 JustFloat 二进制流走的也是 RTT **通道 0**，
     *   插一行文本进去 = 往字节流里塞垃圾，VOFA+ 的帧解析会错位。
     *   跟频状态已经作为通道1/通道2 进了 VOFA+ 数据流，看波形图就够了；
     *   真要文本排查，用 J-Link 直接读 g_trk_* 这些全局变量。 */
    /* 串口收发由 DMA + 空闲中断在后台完成，主循环不用管 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
