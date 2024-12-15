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

#define BUFFERSIZE                       (100)
uint8_t aTxBuffer[BUFFERSIZE] = {0};

/* Buffer used for reception */
uint8_t aRxBuffer[BUFFERSIZE] = {0};

extern uint32_t wTransferState;
uint8_t spiTxFinished = 1;
uint8_t spiRxFinished = 1;

bool HALSPI = true;
/* Buffer used for transmission */
uint8_t ubNbDataToTransmit = 0;
__IO uint8_t ubTransmitIndex = 0;

/* Buffer used for reception */
uint8_t ubNbDataToReceive = 0;
__IO uint8_t ubReceiveIndex = 0;

/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;

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
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi1.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
    PeriphClkInitStruct.Spi1ClockSelection = RCC_SPI1CLKSOURCE_PLL1Q;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* SPI1 interrupt Init */
    HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
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
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

    /* SPI1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(SPI1_IRQn);
  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void Activate_SPI_Interupts(void)
{
  /* Configure SPI1 transfer interrupts */
  
  /* Enable TXP   Interrupt */
  LL_SPI_EnableIT_TXP(SPI1);
  /* Enable RXP  Interrupt */
  LL_SPI_EnableIT_RXP(SPI1);
  /* Enable SPI1 Error Interrupt */
  LL_SPI_EnableIT_CRCERR(SPI1);
}

void Deactivate_SPI_Interupts(void)
{
  /* Configure SPI1 transfer interrupts */
  
  /* Enable TXP   Interrupt */
  LL_SPI_DisableIT_TXP(SPI1);
  /* Enable RXP  Interrupt */
  LL_SPI_DisableIT_RXP(SPI1);
  /* Enable SPI1 Error Interrupt */
  LL_SPI_DisableIT_CRCERR(SPI1);
}

void SPI_TransmitReceive_DMA(uint8_t* transferData, uint8_t* receiveData, uint16_t size, const bmi160_t *bmi160)
{
  uint8_t res = HAL_OK;
  // uint8_t aRxBuffer[117] = {0};

  // HAL_SPI_TransmitReceive_IT(&hspi1, aTxBuffer, aRxBuffer, 2);
  // HAL_SPI_TransmitReceive_DMA(&hspi1, aTxBuffer, aRxBuffer, 2);
  // LL_GPIO_ResetOutputPin(bmi160->cs_port, bmi160->cs_pin);
  //SPI_TransmitReceive(&transferData[0], &receiveData[0], 1, bmi160);
  res = HAL_SPI_TransmitReceive(&hspi1, &transferData[0], &receiveData[0], 1, 1000); 
  wTransferState = TRANSFER_WAIT;
  //res = HAL_SPI_TransmitReceive_IT(&hspi1, transferData, receiveData, 2);
  if (res!= HAL_OK) // after we will use bytesize if we want to optimize
    res = res;

  //LL_GPIO_SetOutputPin(bmi160->cs_port, bmi160->cs_pin);
  res = HAL_SPI_TransmitReceive(&hspi1, &transferData[1], &receiveData[1], 1, 1000);
  wTransferState = TRANSFER_WAIT;
  //uint8_t res = HAL_SPI_TransmitReceive_IT(&hspi1, transferData, receiveData, 2);
  if (res!= HAL_OK) // after we will use bytesize if we want to optimize
     res = res;
//  while(wTransferState != TRANSFER_COMPLETE); // after we will use bytesize if we want to optimize
 
  for (uint16_t i=2; i<size; i+=2)
  {
    // uint8_t res = HAL_SPI_TransmitReceive_DMA(&hspi1, (uint32_t)(transferData+i), (uint32_t)(receiveData+i), 1) ;
    wTransferState = TRANSFER_WAIT;
    res = HAL_SPI_TransmitReceive_IT(&hspi1, (transferData+i), (receiveData+i), 1) ;
    if (res!= HAL_OK) // after we will use bytesize if we want to optimize
      res = res;
    while(wTransferState != TRANSFER_COMPLETE); // after we will use bytesize if we want to optimize
    res = HAL_SPI_TransmitReceive_IT(&hspi1, (transferData+i+1), (receiveData+i+1), 1) ;
    if (res!= HAL_OK) // after we will use bytesize if we want to optimize
      res = res;
    while(wTransferState != TRANSFER_COMPLETE); // after we will use bytesize if we want to optimize
    //HAL_Delay(1);
    for (uint16_t j=1; j<10; j+=1);
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

void SPI_Transfer_DMA(uint8_t* transferData, uint16_t size, const bmi160_t *bmi160)
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

void SPI_Receive_DMA(uint8_t* receiveData, uint16_t size, const bmi160_t *bmi160)
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

/**
  * @brief  TxRx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of Interrupt TxRx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  wTransferState = TRANSFER_COMPLETE;
}

void WaitAndCheckEndOfTransfer(void)
{
  /* 1 - Wait end of transmission */
  while (ubTransmitIndex != ubNbDataToTransmit)
  {
  }
  /* Disable TXP Interrupt */
  LL_SPI_DisableIT_TXP(SPI1);

  /* 2 - Wait end of reception */
  while (ubNbDataToReceive > ubReceiveIndex)
  {
  }
  /* Disable RXP Interrupt */
  LL_SPI_DisableIT_RXP(SPI1);
}

void  SPI1_Rx_Callback(void)
{
  /* Read character in Data register.
  RXP flag is cleared by reading data in DR register */
  aRxBuffer[ubReceiveIndex++] = LL_SPI_ReceiveData8(SPI1);
}

/**
  * @brief  Function called from SPI1 IRQ Handler when TXP flag is set
  *         Function is in charge  to transmit byte on SPI lines.
  * @param  None
  * @retval None
  */
void  SPI1_Tx_Callback(void)
{
  /* Write character in Data register.
  TXP flag is cleared by reading data in DR register */
  LL_SPI_TransmitData8(SPI1, aTxBuffer[ubTransmitIndex++]);
}

/**
  * @brief  Function called in case of error detected in SPI IT Handler
  * @param  None
  * @retval None
  */
void SPI1_TransferError_Callback(void)
{
  /* Disable RXP  Interrupt             */
  LL_SPI_DisableIT_RXP(SPI1);

  /* Disable TXP   Interrupt             */
  LL_SPI_DisableIT_TXP(SPI1);
}

/* USER CODE END 1 */
