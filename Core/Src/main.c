/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "icache.h"
#include "memorymap.h"
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmi160.h"
#include "Madgwick.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUMOFACCL 6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static uint16_t Buffercmp(uint8_t *pBuffer1, uint8_t *pBuffer2, uint16_t BufferLength);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
/* Buffer used for transmission */
uint8_t aTxBuffer[] = "****SPI - Two Boards communication based on Interrupt **** SPI Message ******** SPI Message ******** SPI Message ****";
#define BUFFERSIZE                       (COUNTOF(aTxBuffer) - 1)

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

/* Buffer used for reception */
uint8_t aRxBuffer[BUFFERSIZE] = {0};

/* transfer state */
__IO uint32_t wTransferState = TRANSFER_WAIT;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

calData calibration[NUMOFACCL];
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  MX_SDMMC1_SD_Init();
  MX_USB_PCD_Init();
  MX_ICACHE_Init();
  /* USER CODE BEGIN 2 */
  MX_FileX_Init();
  //   MX_FileX_Process();
  initDataFile();
  
  char acclDataOut[800];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 
  //bma250e_context bma250 = bma250e_init(BMA250E_DEFAULT_SPI_BUS,-1, 10);
  //bmg160_context bmg160 = bmg160_init(BMG160_DEFAULT_SPI_BUS,-1, 10);
  bmi160_context_t bmi160[NUMOFACCL];
  uint8_t test[NUMOFACCL]={0};
  bmi160[0].sensorPosition.geometryIndex = 0;  bmi160[1].sensorPosition.geometryIndex = 0;  bmi160[2].sensorPosition.geometryIndex = 0;
  bmi160[3].sensorPosition.geometryIndex = 0;  bmi160[4].sensorPosition.geometryIndex = 0;  bmi160[5].sensorPosition.geometryIndex = 0;
  //bmi160[6].sensorPosition.geometryIndex = 0;

  // test[0] = bmi160_init(&bmi160[0], ACCL1_CS_GPIO_Port, ACCL1_CS_Pin, true);
  test[0] = bmi160_init(&bmi160[1], ACCL2_CS_GPIO_Port, ACCL2_CS_Pin, true);
  test[1] = bmi160_init(&bmi160[2], ACCL3_CS_GPIO_Port, ACCL3_CS_Pin, true);
  test[2] = bmi160_init(&bmi160[3], ACCL4_CS_GPIO_Port, ACCL4_CS_Pin, true);
  test[3] = bmi160_init(&bmi160[4], ACCL5_CS_GPIO_Port, ACCL5_CS_Pin, true);
  test[4] = bmi160_init(&bmi160[5], ACCL6_CS_GPIO_Port, ACCL6_CS_Pin, true);
  test[5] = bmi160_init(&bmi160[6], ACCL7_CS_GPIO_Port, ACCL7_CS_Pin, true);
  
  float ax[NUMOFACCL], ay[NUMOFACCL], az[NUMOFACCL], temperature[NUMOFACCL];
  float gx[NUMOFACCL], gy[NUMOFACCL], gz[NUMOFACCL];
  float mx[NUMOFACCL], my[NUMOFACCL], mz[NUMOFACCL];
  uint16_t bytesWritten = 0;
  uint32_t counter = 0;
  while (1)
  {
    bytesWritten = 0;
    uint32_t timems = HAL_GetTick();
    bytesWritten += sprintf(&acclDataOut[bytesWritten], "% 8d% 8d",counter++, timems);
    for(int i = 0; i < NUMOFACCL; i++)
    {
      bmi160_update(&bmi160[i]);
      gx[i] = bmi160[i].gyroX;    gy[i] = bmi160[i].gyroY;    gz[i] = bmi160[i].gyroZ;
      ax[i] = bmi160[i].accelX;   ay[i] = bmi160[i].accelY;   az[i] = bmi160[i].accelZ;
      mx[i] = bmi160[i].magX;     my[i] = bmi160[i].magY;     mz[i] = bmi160[i].magZ;

      bytesWritten += sprintf(&acclDataOut[bytesWritten], "% 8.1f% 8.1f% 8.1f% 8.1f% 8.1f% 8.1f% 8.1f% 8.1f% 8.1f|", gx[i], gy[i], gz[i], ax[i], ay[i], az[i], mx[i], my[i], mz[i]);
    }
    bytesWritten += sprintf(&acclDataOut[bytesWritten], "\n");
    writeDataToFile(acclDataOut, bytesWritten);
    //Madgwick_updateIMU(gx, gy, gz, ax, ay, az);
    HAL_Delay(40);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_CSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 125;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
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

#ifdef  USE_FULL_ASSERT
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
