/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_NodeTypeDef Node_GPDMA1_Channel2;
DMA_QListTypeDef List_GPDMA1_Channel2;
DMA_HandleTypeDef handle_GPDMA1_Channel2;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV10;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}
/* ADC2 init function */
void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV10;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

static uint32_t HAL_RCC_ADC_CLK_ENABLED=0;

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  DMA_NodeConfTypeDef NodeConfig= {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADCDAC;
    PeriphClkInitStruct.AdcDacClockSelection = RCC_ADCDACCLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* ADC1 clock enable */
    HAL_RCC_ADC_CLK_ENABLED++;
    if(HAL_RCC_ADC_CLK_ENABLED==1){
      __HAL_RCC_ADC_CLK_ENABLE();
    }

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PC1     ------> ADC1_INP11
    PC3     ------> ADC1_INP13
    PA0     ------> ADC1_INP0
    PA2     ------> ADC1_INP14
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspInit 0 */

  /* USER CODE END ADC2_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADCDAC;
    PeriphClkInitStruct.AdcDacClockSelection = RCC_ADCDACCLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* ADC2 clock enable */
    HAL_RCC_ADC_CLK_ENABLED++;
    if(HAL_RCC_ADC_CLK_ENABLED==1){
      __HAL_RCC_ADC_CLK_ENABLE();
    }

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**ADC2 GPIO Configuration
    PC5     ------> ADC2_INP8
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* ADC2 DMA Init */
    /* GPDMA1_REQUEST_ADC2 Init */
    NodeConfig.NodeType = DMA_GPDMA_LINEAR_NODE;
    NodeConfig.Init.Request = GPDMA1_REQUEST_ADC2;
    NodeConfig.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
    NodeConfig.Init.Direction = DMA_PERIPH_TO_MEMORY;
    NodeConfig.Init.SrcInc = DMA_SINC_FIXED;
    NodeConfig.Init.DestInc = DMA_DINC_FIXED;
    NodeConfig.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_WORD;
    NodeConfig.Init.DestDataWidth = DMA_DEST_DATAWIDTH_WORD;
    NodeConfig.Init.SrcBurstLength = 1;
    NodeConfig.Init.DestBurstLength = 1;
    NodeConfig.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT1|DMA_DEST_ALLOCATED_PORT0;
    NodeConfig.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    NodeConfig.Init.Mode = DMA_NORMAL;
    NodeConfig.TriggerConfig.TriggerPolarity = DMA_TRIG_POLARITY_MASKED;
    NodeConfig.DataHandlingConfig.DataExchange = DMA_EXCHANGE_NONE;
    NodeConfig.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;
    if (HAL_DMAEx_List_BuildNode(&NodeConfig, &Node_GPDMA1_Channel2) != HAL_OK)
    {
      Error_Handler();
    }

    if (HAL_DMAEx_List_InsertNode(&List_GPDMA1_Channel2, NULL, &Node_GPDMA1_Channel2) != HAL_OK)
    {
      Error_Handler();
    }

    if (HAL_DMAEx_List_SetCircularMode(&List_GPDMA1_Channel2) != HAL_OK)
    {
      Error_Handler();
    }

    handle_GPDMA1_Channel2.Instance = GPDMA1_Channel2;
    handle_GPDMA1_Channel2.InitLinkedList.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
    handle_GPDMA1_Channel2.InitLinkedList.LinkStepMode = DMA_LSM_FULL_EXECUTION;
    handle_GPDMA1_Channel2.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT0;
    handle_GPDMA1_Channel2.InitLinkedList.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    handle_GPDMA1_Channel2.InitLinkedList.LinkedListMode = DMA_LINKEDLIST_CIRCULAR;
    if (HAL_DMAEx_List_Init(&handle_GPDMA1_Channel2) != HAL_OK)
    {
      Error_Handler();
    }

    if (HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel2, &List_GPDMA1_Channel2) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle, DMA_Handle, handle_GPDMA1_Channel2);

    if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel2, DMA_CHANNEL_NPRIV) != HAL_OK)
    {
      Error_Handler();
    }

    /* ADC2 interrupt Init */
    HAL_NVIC_SetPriority(ADC2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC2_IRQn);
  /* USER CODE BEGIN ADC2_MspInit 1 */

  /* USER CODE END ADC2_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC_CLK_ENABLED--;
    if(HAL_RCC_ADC_CLK_ENABLED==0){
      __HAL_RCC_ADC_CLK_DISABLE();
    }

    /**ADC1 GPIO Configuration
    PC1     ------> ADC1_INP11
    PC3     ------> ADC1_INP13
    PA0     ------> ADC1_INP0
    PA2     ------> ADC1_INP14
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1|GPIO_PIN_3);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_2);

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspDeInit 0 */

  /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC_CLK_ENABLED--;
    if(HAL_RCC_ADC_CLK_ENABLED==0){
      __HAL_RCC_ADC_CLK_DISABLE();
    }

    /**ADC2 GPIO Configuration
    PC5     ------> ADC2_INP8
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_5);

    /* ADC2 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);

    /* ADC2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(ADC2_IRQn);
  /* USER CODE BEGIN ADC2_MspDeInit 1 */

  /* USER CODE END ADC2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
