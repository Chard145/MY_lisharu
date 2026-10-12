/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dac.c
  * @brief   This file provides code for the configuration
  *          of the DAC instances.
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
#include "dac.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

DAC_HandleTypeDef hdac3;
DMA_HandleTypeDef hdma_dac3_ch1;

/* DAC3 init function */
void MX_DAC3_Init(void)
{

  /* USER CODE BEGIN DAC3_Init 0 */

  /* USER CODE END DAC3_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC3_Init 1 */

  /* USER CODE END DAC3_Init 1 */

  /** DAC Initialization
  */
  hdac3.Instance = DAC3;
  if (HAL_DAC_Init(&hdac3) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC3_Init 2 */

  /* USER CODE END DAC3_Init 2 */

}

void HAL_DAC_MspInit(DAC_HandleTypeDef* dacHandle)
{

  if(dacHandle->Instance==DAC3)
  {
  /* USER CODE BEGIN DAC3_MspInit 0 */

  /* USER CODE END DAC3_MspInit 0 */
    /* DAC3 clock enable */
    __HAL_RCC_DAC3_CLK_ENABLE();

    /* DAC3 DMA Init */
    /* DAC3_CH1 Init */
    hdma_dac3_ch1.Instance = DMA1_Channel1;
    hdma_dac3_ch1.Init.Request = DMA_REQUEST_DAC3_CHANNEL1;
    hdma_dac3_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_dac3_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dac3_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dac3_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_dac3_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_dac3_ch1.Init.Mode = DMA_CIRCULAR;
    hdma_dac3_ch1.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_dac3_ch1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(dacHandle,DMA_Handle1,hdma_dac3_ch1);

  /* USER CODE BEGIN DAC3_MspInit 1 */

  /* USER CODE END DAC3_MspInit 1 */
  }
}

void HAL_DAC_MspDeInit(DAC_HandleTypeDef* dacHandle)
{

  if(dacHandle->Instance==DAC3)
  {
  /* USER CODE BEGIN DAC3_MspDeInit 0 */

  /* USER CODE END DAC3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_DAC3_CLK_DISABLE();

    /* DAC3 DMA DeInit */
    HAL_DMA_DeInit(dacHandle->DMA_Handle1);
  /* USER CODE BEGIN DAC3_MspDeInit 1 */

  /* USER CODE END DAC3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
