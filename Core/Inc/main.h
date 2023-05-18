/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define D2_9_Pin GPIO_PIN_2
#define D2_9_GPIO_Port GPIOE
#define D2_10_Pin GPIO_PIN_3
#define D2_10_GPIO_Port GPIOE
#define D1_11_Pin GPIO_PIN_4
#define D1_11_GPIO_Port GPIOE
#define D2_12_Pin GPIO_PIN_5
#define D2_12_GPIO_Port GPIOE
#define D2_7_Pin GPIO_PIN_6
#define D2_7_GPIO_Port GPIOE
#define D2_6_Pin GPIO_PIN_13
#define D2_6_GPIO_Port GPIOC
#define D2_5_Pin GPIO_PIN_2
#define D2_5_GPIO_Port GPIOF
#define D5__Pin GPIO_PIN_6
#define D5__GPIO_Port GPIOA
#define ALARM30_Pin GPIO_PIN_5
#define ALARM30_GPIO_Port GPIOC
#define ALARM_Pin GPIO_PIN_13
#define ALARM_GPIO_Port GPIOF
#define SEL3_Pin GPIO_PIN_0
#define SEL3_GPIO_Port GPIOG
#define SEL2_Pin GPIO_PIN_1
#define SEL2_GPIO_Port GPIOG
#define SEL1_Pin GPIO_PIN_7
#define SEL1_GPIO_Port GPIOE
#define SEL0_Pin GPIO_PIN_8
#define SEL0_GPIO_Port GPIOE
#define D1_15_Pin GPIO_PIN_11
#define D1_15_GPIO_Port GPIOB
#define D1_14_Pin GPIO_PIN_2
#define D1_14_GPIO_Port GPIOG
#define D1_13_Pin GPIO_PIN_3
#define D1_13_GPIO_Port GPIOG
#define D1_16_Pin GPIO_PIN_4
#define D1_16_GPIO_Port GPIOG
#define D2_8_Pin GPIO_PIN_15
#define D2_8_GPIO_Port GPIOA
#define D3_5_Pin GPIO_PIN_9
#define D3_5_GPIO_Port GPIOG
#define D3_6_Pin GPIO_PIN_10
#define D3_6_GPIO_Port GPIOG
#define D3_7_Pin GPIO_PIN_11
#define D3_7_GPIO_Port GPIOG
#define D2_2_Pin GPIO_PIN_12
#define D2_2_GPIO_Port GPIOG
#define D2_1_Pin GPIO_PIN_13
#define D2_1_GPIO_Port GPIOG
#define D2_4_Pin GPIO_PIN_14
#define D2_4_GPIO_Port GPIOG
#define D3_8_Pin GPIO_PIN_15
#define D3_8_GPIO_Port GPIOG
#define D3_1_Pin GPIO_PIN_3
#define D3_1_GPIO_Port GPIOB
#define D3_2_Pin GPIO_PIN_4
#define D3_2_GPIO_Port GPIOB
#define D3_3_Pin GPIO_PIN_5
#define D3_3_GPIO_Port GPIOB
#define D3_4_Pin GPIO_PIN_6
#define D3_4_GPIO_Port GPIOB
#define D2_13_Pin GPIO_PIN_7
#define D2_13_GPIO_Port GPIOB
#define D2_14_Pin GPIO_PIN_8
#define D2_14_GPIO_Port GPIOB
#define D2_15_Pin GPIO_PIN_9
#define D2_15_GPIO_Port GPIOB
#define D2_3_Pin GPIO_PIN_0
#define D2_3_GPIO_Port GPIOE
#define D2_16_Pin GPIO_PIN_1
#define D2_16_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
