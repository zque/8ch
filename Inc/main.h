/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOE
#define reset_4G_Pin GPIO_PIN_4
#define reset_4G_GPIO_Port GPIOE
#define K1_Pin GPIO_PIN_3
#define K1_GPIO_Port GPIOH
#define K2_Pin GPIO_PIN_4
#define K2_GPIO_Port GPIOH
#define K3_Pin GPIO_PIN_5
#define K3_GPIO_Port GPIOH
#define K4_Pin GPIO_PIN_6
#define K4_GPIO_Port GPIOH
#define K5_Pin GPIO_PIN_7
#define K5_GPIO_Port GPIOH
#define K6_Pin GPIO_PIN_8
#define K6_GPIO_Port GPIOH
#define K7_Pin GPIO_PIN_9
#define K7_GPIO_Port GPIOH
#define K8_Pin GPIO_PIN_10
#define K8_GPIO_Port GPIOH
#define BEE_Pin GPIO_PIN_11
#define BEE_GPIO_Port GPIOH
#define RE_Pin GPIO_PIN_12
#define RE_GPIO_Port GPIOH
#define LED8_Pin GPIO_PIN_12
#define LED8_GPIO_Port GPIOB
#define LED9_Pin GPIO_PIN_13
#define LED9_GPIO_Port GPIOB
#define LED10_Pin GPIO_PIN_14
#define LED10_GPIO_Port GPIOB
#define KM1_Pin GPIO_PIN_0
#define KM1_GPIO_Port GPIOI
#define KM2_Pin GPIO_PIN_1
#define KM2_GPIO_Port GPIOI
#define KM3_Pin GPIO_PIN_2
#define KM3_GPIO_Port GPIOI
#define KM4_Pin GPIO_PIN_3
#define KM4_GPIO_Port GPIOI
#define LED3_Pin GPIO_PIN_5
#define LED3_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_6
#define LED4_GPIO_Port GPIOB
#define LED5_Pin GPIO_PIN_7
#define LED5_GPIO_Port GPIOB
#define LED6_Pin GPIO_PIN_8
#define LED6_GPIO_Port GPIOB
#define LED7_Pin GPIO_PIN_9
#define LED7_GPIO_Port GPIOB
#define KM5_Pin GPIO_PIN_4
#define KM5_GPIO_Port GPIOI
#define KM6_Pin GPIO_PIN_5
#define KM6_GPIO_Port GPIOI
#define KM7_Pin GPIO_PIN_6
#define KM7_GPIO_Port GPIOI
#define KM8_Pin GPIO_PIN_7
#define KM8_GPIO_Port GPIOI
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
