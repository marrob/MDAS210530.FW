/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
 #define DEVICE_DEBUG_LEVEL    0


#if (DEVICE_DEBUG_LEVEL > 0)
#define  DeviceUsrLog(...)  {printf(__VA_ARGS__);\
                             printf("\r\n");}
#else
#define DeviceUsrLog(...)
#endif

#if (DEVICE_DEBUG_LEVEL > 1)

#define  DeviceErrLog(...)  {printf(VT100_ATTR_RED);\
                             printf("ERROR.DEVICE:") ;\
                             printf(__VA_ARGS__);\
                             printf(VT100_ATTR_RESET);\
                             printf("\r\n");}
#else
#define DeviceErrLog(...)
#endif

#if (DEVICE_DEBUG_LEVEL > 2)
#define  DeviceDbgLog(...)  {printf(VT100_ATTR_YELLOW);\
                             printf("DEBUG.DEVICE:") ;\
                             printf(__VA_ARGS__);\
                             printf(VT100_ATTR_RESET);\
                             printf("\r\n");}
#else
#define DeviceDbgLog(...)
#endif
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MV341_I_AIN0_Pin GPIO_PIN_0
#define MV341_I_AIN0_GPIO_Port GPIOA
#define MV205_I_AIN1_Pin GPIO_PIN_1
#define MV205_I_AIN1_GPIO_Port GPIOA
#define MV205_I_AIN2_Pin GPIO_PIN_2
#define MV205_I_AIN2_GPIO_Port GPIOA
#define U_MAIN_AIN3_Pin GPIO_PIN_3
#define U_MAIN_AIN3_GPIO_Port GPIOA
#define MV341_TEMP_AIN4_Pin GPIO_PIN_4
#define MV341_TEMP_AIN4_GPIO_Port GPIOA
#define MV205_1_TEMP_AIN5_Pin GPIO_PIN_5
#define MV205_1_TEMP_AIN5_GPIO_Port GPIOA
#define MV205_2_TEMP_AIN6_Pin GPIO_PIN_6
#define MV205_2_TEMP_AIN6_GPIO_Port GPIOA
#define MV205_1_EN_Pin GPIO_PIN_1
#define MV205_1_EN_GPIO_Port GPIOB
#define MV205_2_EN_Pin GPIO_PIN_2
#define MV205_2_EN_GPIO_Port GPIOB
#define INT_EXT_Pin GPIO_PIN_10
#define INT_EXT_GPIO_Port GPIOB
#define LOCK2_Pin GPIO_PIN_12
#define LOCK2_GPIO_Port GPIOB
#define LOCK1_Pin GPIO_PIN_13
#define LOCK1_GPIO_Port GPIOB
#define MV205EN_LED_Pin GPIO_PIN_4
#define MV205EN_LED_GPIO_Port GPIOB
#define LOCK2_LED_Pin GPIO_PIN_5
#define LOCK2_LED_GPIO_Port GPIOB
#define LOCK1_LED_Pin GPIO_PIN_6
#define LOCK1_LED_GPIO_Port GPIOB
#define LIVE_LED_Pin GPIO_PIN_7
#define LIVE_LED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define DEVICE_FW           0x210625
#define DEVICE_NAME         "MDAS210530"
#define DEVICE_PCB          "V00"
#define DEVICE_MNF          "COREAUDIO"
#define DEVICE_NAME_SIZE    32
#define DEVICE_FW_SIZE      sizeof(DEVICE_FW)
#define DEVICE_PCB_SIZE     sizeof(DEVICE_PCB)
#define DEVICE_MNF_SIZE     sizeof(DEVICE_MNF)

#define DEVICE_OK           0
#define DEVICE_FAIL         1


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
