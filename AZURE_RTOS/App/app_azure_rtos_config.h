/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_azure_rtos_config.h
  * @author  MCD Application Team
  * @brief   azure_rtos config header file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020-2021 STMicroelectronics.
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
#ifndef APP_AZURE_RTOS_CONFIG_H
#define APP_AZURE_RTOS_CONFIG_H
#ifdef __cplusplus
extern "C" {
#endif

/* USER CODE BEGIN 1 */
#if defined( __ICCARM__ )
  #define USBX_MEMORY \
      _Pragma("location=\".usbx_memory\"")
#else
  #define USBX_MEMORY \
      __attribute__((section(".usbx_memory")))
#endif

#if defined( __ICCARM__ )
  #define USBX_POOL_MEMORY \
      _Pragma("location=\".UsbxPoolSection\"")
#else
  #define USBX_POOL_MEMORY \
      __attribute__((section(".UsbxPoolSection")))
#endif

#if defined( __ICCARM__ )
  #define USBX_HPCD_MEMORY \
      _Pragma("location=\".UsbHpcdSection\"")
#else
  #define USBX_HPCD_MEMORY \
      __attribute__((section(".UsbHpcdSection")))
#endif
/* USER CODE END 1 */

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* Using static memory allocation via threadX Byte memory pools */

#define USE_STATIC_ALLOCATION                    1

#define TX_APP_MEM_POOL_SIZE                     40*1024

#define UX_HOST_APP_MEM_POOL_SIZE                32*1024

/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

#ifdef __cplusplus
}
#endif

#endif /* APP_AZURE_RTOS_CONFIG_H */
