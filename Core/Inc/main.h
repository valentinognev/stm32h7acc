/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h5xx_hal.h"

#include "stm32h5xx_ll_usart.h"
#include "stm32h5xx_ll_rcc.h"
#include "stm32h5xx_ll_system.h"
#include "stm32h5xx_ll_gpio.h"
#include "stm32h5xx_ll_exti.h"
#include "stm32h5xx_ll_bus.h"
#include "stm32h5xx_ll_cortex.h"
#include "stm32h5xx_ll_utils.h"
#include "stm32h5xx_ll_pwr.h"
#include "stm32h5xx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
enum {
  TRANSFER_WAIT,
  TRANSFER_COMPLETE,
  TRANSFER_ERROR
};

typedef struct {
	bool valid;
	float accelBias[3];
	float gyroBias[3];
	float magBias[3];
	float magScale[3];
} calData;
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
#define BUTTON_Pin LL_GPIO_PIN_13
#define BUTTON_GPIO_Port GPIOC
#define ACCL7_CS_Pin LL_GPIO_PIN_2
#define ACCL7_CS_GPIO_Port GPIOA
#define ACCL5_CS_Pin LL_GPIO_PIN_3
#define ACCL5_CS_GPIO_Port GPIOA
#define ACCL4_CS_Pin LL_GPIO_PIN_4
#define ACCL4_CS_GPIO_Port GPIOA
#define IMU_SCK_Pin LL_GPIO_PIN_5
#define IMU_SCK_GPIO_Port GPIOA
#define IMU_MISO_Pin LL_GPIO_PIN_6
#define IMU_MISO_GPIO_Port GPIOA
#define IMU_MOSI_Pin LL_GPIO_PIN_7
#define IMU_MOSI_GPIO_Port GPIOA
#define ACCL1_CS_Pin LL_GPIO_PIN_4
#define ACCL1_CS_GPIO_Port GPIOC
#define ACCL3_CS_Pin LL_GPIO_PIN_5
#define ACCL3_CS_GPIO_Port GPIOC
#define ACCL6_CS_Pin LL_GPIO_PIN_0
#define ACCL6_CS_GPIO_Port GPIOB
#define ACCL2_CS_Pin LL_GPIO_PIN_1
#define ACCL2_CS_GPIO_Port GPIOB
#define DIODE_Pin LL_GPIO_PIN_2
#define DIODE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
