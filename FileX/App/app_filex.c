/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_filex.c
  * @author  MCD Application Team
  * @brief   FileX applicative file
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
#include "app_filex.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FILEBASE "IMU_DATA"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Define FileX global data structures.  */
FX_MEDIA sdio_disk;
FX_FILE  fx_file;
uint32_t posInFile = 0;
CHAR file_name[FX_MAX_LONG_NAME_LEN] = "";

UINT media_memory[512 / sizeof(UINT)];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */
/**
  * @brief  Application FileX Initialization.
  * @param  None
  * @retval int
  */
UINT MX_FileX_Init(void)
{
  UINT ret = FX_SUCCESS;
  /* USER CODE BEGIN MX_FileX_Init */

  /* USER CODE END MX_FileX_Init */

  /* Initialize FileX.  */
  fx_system_initialize();

  /* USER CODE BEGIN MX_FileX_Init 1*/

  /* Start application */
  /* Open the sdio_disk driver. */
  ret =  fx_media_open(&sdio_disk, "STM32_SDIO_DISK", fx_stm32_sd_driver, 0, (VOID *) media_memory, sizeof(media_memory));

  /* Check the media open ret. */
  if (ret != FX_SUCCESS)
  {
    Error_Handler();
  }

  /* USER CODE END MX_FileX_Init 1*/

  return ret;
}

/* USER CODE BEGIN 1 */
UINT openDataFile(void);
UINT closeDataFile(void);

UINT writeDataToFile(uint8_t* data, uint32_t size)
{
    UINT status;
    status = openDataFile();
    status = fx_file_seek(&fx_file, posInFile);
    status = fx_file_write(&fx_file, data, size);
    status = closeDataFile();
    posInFile += size;
    return status;
}

UINT openDataFile(void)
{
  /* Open the test file. */
  UINT status =  fx_file_open(&sdio_disk, &fx_file, file_name, FX_OPEN_FOR_WRITE);

  /* Check the file open status. */
  if (status != FX_SUCCESS)
  {
    /* Error opening file, call error handler. */
    Error_Handler();
  }

  return status;
}

UINT initDataFile(void)
{
  UINT status;
  uint32_t curIndex = 0;
  sprintf(file_name, "%s_%d.TXT", FILEBASE, curIndex);

  status =  fx_file_create(&sdio_disk, file_name);
  if (status != FX_SUCCESS && status != FX_ALREADY_CREATED)
  {
    Error_Handler();
  }
  while (status == FX_ALREADY_CREATED)
  {
    curIndex++;
    sprintf(file_name, "%s_%d.TXT", FILEBASE, curIndex);
    status =  fx_file_create(&sdio_disk, file_name);
    if (status != FX_SUCCESS && status != FX_ALREADY_CREATED)
    {
      Error_Handler();
    }
  }
  return status;
}

UINT closeDataFile(void)
{
  UINT status;
  /* Close the test file. */
  status =  fx_file_close(&fx_file);

  /* Check the file close status. */
  if (status != FX_SUCCESS)
  {
    /* Error closing the file, call error handler. */
    Error_Handler();
  }

  status = fx_media_flush(&sdio_disk);

  /* Check the media flush  status. */
  if (status != FX_SUCCESS)
  {
    /* Error closing the file, call error handler. */
    Error_Handler();
  }

  return status;
}


VOID MX_FileX_Process(void)
{
  UINT status;
  ULONG bytes_read;
  CHAR read_buffer[32];
  CHAR data1[] = "This is FileX working on STM32_1";
  CHAR data2[] = "This is FileX working on STM32_2";
  CHAR file_name[FX_MAX_LONG_NAME_LEN] = "";
  uint32_t curIndex = 0;
  sprintf(file_name, "%s_%d.TXT", FILEBASE, curIndex);

  status =  fx_file_create(&sdio_disk, file_name);
  if (status != FX_SUCCESS && status != FX_ALREADY_CREATED)
  {
    Error_Handler();
  }
  while (status == FX_ALREADY_CREATED)
  {
    curIndex++;
    sprintf(file_name, "%s_%d.TXT", FILEBASE, curIndex);
    status =  fx_file_create(&sdio_disk, file_name);
    if (status != FX_SUCCESS && status != FX_ALREADY_CREATED)
    {
      Error_Handler();
    }
  }

  /* Open the test file. */
  status =  fx_file_open(&sdio_disk, &fx_file, file_name, FX_OPEN_FOR_WRITE);

  /* Check the file open status. */
  if (status != FX_SUCCESS)
  {
    /* Error opening file, call error handler. */
    Error_Handler();
  }

  /* Seek to the beginning of the test file. */
  status =  fx_file_seek(&fx_file, 0);

  /* Check the file seek status. */
  if (status != FX_SUCCESS)
  {
    /* Error performing file seek, call error handler. */
    Error_Handler();
  }

  /* Write a string to the test file. */
  status =  fx_file_write(&fx_file, data1, sizeof(data1));

  /* Check the file write status. */
  if (status != FX_SUCCESS)
  {
    /* Error writing to a file, call error handler. */
    Error_Handler();
  }
  status =  fx_file_write(&fx_file, data2, sizeof(data2));

  /* Check the file write status. */
  if (status != FX_SUCCESS)
  {
    /* Error writing to a file, call error handler. */
    Error_Handler();
  }

  /* Close the test file. */
  status =  fx_file_close(&fx_file);

  /* Check the file close status. */
  if (status != FX_SUCCESS)
  {
    /* Error closing the file, call error handler. */
    Error_Handler();
  }

  status = fx_media_flush(&sdio_disk);

  /* Check the media flush  status. */
  if (status != FX_SUCCESS)
  {
    /* Error closing the file, call error handler. */
    Error_Handler();
  }

  /* Open the test file. */
  status =  fx_file_open(&sdio_disk, &fx_file, file_name, FX_OPEN_FOR_READ);

  /* Check the file open status. */
  if (status != FX_SUCCESS)
  {
    /* Error opening file, call error handler. */
    Error_Handler();
  }

  /* Seek to the beginning of the test file. */
  status =  fx_file_seek(&fx_file, 0);

  /* Check the file seek status. */
  if (status != FX_SUCCESS)
  {
    /* Error performing file seek, call error handler. */
    Error_Handler();
  }

  /* Read the first 28 bytes of the test file. */
  status =  fx_file_read(&fx_file, read_buffer, sizeof(data1), &bytes_read);

  /* Check the file read status.  */
  if ((status != FX_SUCCESS) || (bytes_read != sizeof(data1)))
  {
    /* Error reading file, call error handler. */
    Error_Handler();
  }

  /* Close the test file. */
  status =  fx_file_close(&fx_file);

  /* Check the file close status. */
  if (status != FX_SUCCESS)
  {
    /* Error closing the file, call error handler. */
    Error_Handler();
  }

  /* Close the media. */
  status =  fx_media_close(&sdio_disk);

  /* Check the media close status. */
  if (status != FX_SUCCESS)
  {
    /* Error closing the media, call error handler. */
    Error_Handler();
  }

}

/* USER CODE END 1 */
