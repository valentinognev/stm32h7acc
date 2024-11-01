/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
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
#include "spi.h"

/* USER CODE BEGIN 0 */
#include "main.h"

uint8_t spiTxFinished = 1;
uint8_t spiRxFinished = 1;
/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_0);
  LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);

  LL_DMA_ClearFlag_TC4(DMA1);
  LL_DMA_ClearFlag_TE4(DMA1);
  LL_DMA_ClearFlag_TC5(DMA1);
  LL_DMA_ClearFlag_TE5(DMA1);

  LL_SPI_EnableDMAReq_TX(SPI1);
  LL_SPI_EnableDMAReq_RX(SPI1);

  LL_DMA_EnableIT_TE(DMA1, LL_DMA_STREAM_0);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_0);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_STREAM_1);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_1);
  LL_SPI_Enable(SPI1);

  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_0, 1);
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, 1);

  /* USER CODE END SPI1_Init 2 */

}
/* SPI2 init function */
void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI1;
    PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PB3 (JTDO/TRACESWO)     ------> SPI1_SCK
    PB4 (NJTRST)     ------> SPI1_MISO
    PB5     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = ACCL1_SCK_Pin|ACCL1_MISO_Pin|ACCL1_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* SPI1 DMA Init */
    /* SPI1_RX Init */
    hdma_spi1_rx.Instance = DMA1_Stream0;
    hdma_spi1_rx.Init.Request = DMA_REQUEST_SPI1_RX;
    hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_spi1_rx.Init.Mode = DMA_NORMAL;
    hdma_spi1_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmarx,hdma_spi1_rx);

    /* SPI1_TX Init */
    hdma_spi1_tx.Instance = DMA1_Stream1;
    hdma_spi1_tx.Init.Request = DMA_REQUEST_SPI1_TX;
    hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_spi1_tx.Init.Mode = DMA_NORMAL;
    hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmatx,hdma_spi1_tx);

    /* SPI1 interrupt Init */
    HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
  else if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
    PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* SPI2 clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI2 GPIO Configuration
    PB12     ------> SPI2_NSS
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PB3 (JTDO/TRACESWO)     ------> SPI1_SCK
    PB4 (NJTRST)     ------> SPI1_MISO
    PB5     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOB, ACCL1_SCK_Pin|ACCL1_MISO_Pin|ACCL1_MOSI_Pin);

    /* SPI1 DMA DeInit */
    HAL_DMA_DeInit(spiHandle->hdmarx);
    HAL_DMA_DeInit(spiHandle->hdmatx);

    /* SPI1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(SPI1_IRQn);
  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
  else if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**SPI2 GPIO Configuration
    PB12     ------> SPI2_NSS
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);

  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void SPI_TransmitReceive_DMA(uint16_t* transferData, uint16_t* receiveData, uint16_t size)
{
  for (uint16_t i=0; i<size; i++)
  {
    HAL_GPIO_WritePin(  ACCL1_CS_GPIO_Port,   ACCL1_CS_Pin, GPIO_PIN_RESET); spiTxFinished = 0;spiRxFinished = 0;
    // uint8_t res = HAL_SPI_TransmitReceive_DMA(&hspi1, (uint32_t)(transferData+i), (uint32_t)(receiveData+i), 1) ;
    uint8_t res = HAL_SPI_TransmitReceive_IT(&hspi1, (uint32_t)(transferData+i), (uint32_t)(receiveData+i), 1) ;
    if(res!= HAL_OK) // after we will use bytesize if we want to optimize
        res = res;

    // LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_4);
    // LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_5);
    // LL_SPI_Disable(SPI1);
    // LL_DMA_ConfigAddresses(DMA2, LL_DMA_CHANNEL_4, LL_SPI_DMA_GetRegAddr(SPI1), (uint32_t)(receiveData+i), LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_CHANNEL_4));
    // LL_DMA_ConfigAddresses(DMA2, LL_DMA_CHANNEL_5, (uint32_t)(transferData+i), LL_SPI_DMA_GetRegAddr(SPI1), LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_CHANNEL_5));

    // LL_SPI_Enable(SPI1);   
    // LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_4);
    // LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_5);

  }
}

void SPI_Transfer_DMA(uint16_t* transferData, uint16_t size)
{
  for (uint16_t i=0; i<size; i++)
  {
    HAL_GPIO_WritePin(  ACCL1_CS_GPIO_Port,   ACCL1_CS_Pin, GPIO_PIN_RESET); spiTxFinished = 0;
    uint8_t res = HAL_SPI_Transmit_DMA(&hspi1, (uint32_t)(transferData+i), 1) ;
    if(res!= HAL_OK) // after we will use bytesize if we want to optimize
        res = res;

    // LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_5);
    // LL_DMA_ConfigAddresses(DMA2, LL_DMA_CHANNEL_5, (uint32_t)(transferData+i), LL_SPI_DMA_GetRegAddr(SPI1), LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_CHANNEL_5));

    // HAL_GPIO_WritePin(  ACCL1_CS_GPIO_Port,   ACCL1_CS_Pin, GPIO_PIN_RESET); spiTxFinished = 0;
    // LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_5);
  }
}

void SPI_Receive_DMA(uint16_t* receiveData, uint16_t size)
{
  for (uint16_t i=0; i<size; i++)
  {
    HAL_GPIO_WritePin(  ACCL1_CS_GPIO_Port,   ACCL1_CS_Pin, GPIO_PIN_RESET); spiRxFinished = 0;
    uint8_t res = HAL_SPI_Receive_DMA(&hspi1, (uint32_t)(receiveData+i), 1) ;
    if(res!= HAL_OK) // after we will use bytesize if we want to optimize
        res = res;
    // LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_4);
    // LL_DMA_ConfigAddresses(DMA2, LL_DMA_CHANNEL_4, LL_SPI_DMA_GetRegAddr(SPI1), (uint32_t)(receiveData+i), LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_CHANNEL_4));

    // HAL_GPIO_WritePin(  ACCL1_CS_GPIO_Port,   ACCL1_CS_Pin, GPIO_PIN_RESET); spiRxFinished = 0;
    // LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_4);
  }
}

void DMA1_ReceiveComplete(void)
{
    // TX Done .. Do Something ...
  // LL_DMA_ClearFlag_TC4(DMA2);
  if (spiTxFinished)
    HAL_GPIO_WritePin(  ACCL1_CS_GPIO_Port,   ACCL1_CS_Pin, GPIO_PIN_SET);
  spiRxFinished = 1;
}

void DMA1_TransmitComplete(void)
{
    // RX Done .. Do Something ...
  // LL_DMA_ClearFlag_TC5(DMA2);
  if (spiRxFinished)
    HAL_GPIO_WritePin(  ACCL1_CS_GPIO_Port,   ACCL1_CS_Pin, GPIO_PIN_SET);
  spiTxFinished = 1;
}

/* USER CODE END 1 */
