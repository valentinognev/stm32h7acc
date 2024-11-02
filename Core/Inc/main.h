/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_cortex.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_system.h"
#include "stm32h7xx_ll_utils.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_dma.h"

#include "stm32h7xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32h7xx_ll_spi.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
enum {
  TRANSFER_WAIT,
  TRANSFER_COMPLETE,
  TRANSFER_ERROR
};
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ACCL4_SCK_Pin GPIO_PIN_2
#define ACCL4_SCK_GPIO_Port GPIOE
#define Button1_Pin GPIO_PIN_3
#define Button1_GPIO_Port GPIOE
#define ACCL4_CS_Pin GPIO_PIN_4
#define ACCL4_CS_GPIO_Port GPIOE
#define ACCL4_MISO_Pin GPIO_PIN_5
#define ACCL4_MISO_GPIO_Port GPIOE
#define ACCL4_MOSI_Pin GPIO_PIN_6
#define ACCL4_MOSI_GPIO_Port GPIOE
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOA
#define ACCL1_CS_Pin GPIO_PIN_4
#define ACCL1_CS_GPIO_Port GPIOA
#define ACCL1_SCK_Pin GPIO_PIN_5
#define ACCL1_SCK_GPIO_Port GPIOA
#define ACCL1_MISO_Pin GPIO_PIN_6
#define ACCL1_MISO_GPIO_Port GPIOA
#define ACCL1_MOSI_Pin GPIO_PIN_7
#define ACCL1_MOSI_GPIO_Port GPIOA
#define ACCL2_CS_Pin GPIO_PIN_12
#define ACCL2_CS_GPIO_Port GPIOB
#define ACCL2_SCK_Pin GPIO_PIN_13
#define ACCL2_SCK_GPIO_Port GPIOB
#define ACCL2_MISO_Pin GPIO_PIN_14
#define ACCL2_MISO_GPIO_Port GPIOB
#define ACCL2_MOSI_Pin GPIO_PIN_15
#define ACCL2_MOSI_GPIO_Port GPIOB
#define ACCL3_SCK_Pin GPIO_PIN_10
#define ACCL3_SCK_GPIO_Port GPIOC
#define ACCL3_MISO_Pin GPIO_PIN_11
#define ACCL3_MISO_GPIO_Port GPIOC
#define ACCL3_MOSI_Pin GPIO_PIN_12
#define ACCL3_MOSI_GPIO_Port GPIOC
#define ACCL3_CS_Pin GPIO_PIN_0
#define ACCL3_CS_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
