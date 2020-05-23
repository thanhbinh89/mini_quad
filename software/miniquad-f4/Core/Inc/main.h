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
#define BAT0_Pin GPIO_PIN_0
#define BAT0_GPIO_Port GPIOA
#define PPM_Pin GPIO_PIN_5
#define PPM_GPIO_Port GPIOA
#define M3_Pin GPIO_PIN_0
#define M3_GPIO_Port GPIOB
#define M4_Pin GPIO_PIN_1
#define M4_GPIO_Port GPIOB
#define BUT1_Pin GPIO_PIN_12
#define BUT1_GPIO_Port GPIOB
#define BUT2_Pin GPIO_PIN_13
#define BUT2_GPIO_Port GPIOB
#define LED_USR_Pin GPIO_PIN_14
#define LED_USR_GPIO_Port GPIOB
#define LED_DBG_Pin GPIO_PIN_15
#define LED_DBG_GPIO_Port GPIOB
#define TX1_Pin GPIO_PIN_9
#define TX1_GPIO_Port GPIOA
#define RX1_Pin GPIO_PIN_10
#define RX1_GPIO_Port GPIOA
#define CS3_Pin GPIO_PIN_15
#define CS3_GPIO_Port GPIOA
#define SCK3_Pin GPIO_PIN_10
#define SCK3_GPIO_Port GPIOC
#define MISO3_Pin GPIO_PIN_11
#define MISO3_GPIO_Port GPIOC
#define MOSI3_Pin GPIO_PIN_12
#define MOSI3_GPIO_Port GPIOC
#define M1_Pin GPIO_PIN_4
#define M1_GPIO_Port GPIOB
#define M2_Pin GPIO_PIN_5
#define M2_GPIO_Port GPIOB
#define INT1_Pin GPIO_PIN_6
#define INT1_GPIO_Port GPIOB
#define INT1_EXTI_IRQn EXTI9_5_IRQn
#define DRDY_Pin GPIO_PIN_7
#define DRDY_GPIO_Port GPIOB
#define DRDY_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
