/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "l3g4.h"
#include "pid.h"
#include "ppm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DEBUG_LED_EN	0

#define DEBUG_SS_RAW_EN	0
#define DEBUG_SS_FIL_EN	0

#define DEBUG_RAW_RECV_EN	0
#define DEBUG_MAP_RECV_EN	0

#define DEBUG_PID_EN	0
#define DEBUG_MOTOR_EN	0

#define SET_MOTOR_1(value)	(htim3.Instance->CCR1=(uint16_t)value)
#define SET_MOTOR_2(value)	(htim3.Instance->CCR2=(uint16_t)value)
#define SET_MOTOR_3(value)	(htim3.Instance->CCR3=(uint16_t)value)
#define SET_MOTOR_4(value)	(htim3.Instance->CCR4=(uint16_t)value)

#define MAX_MOTOR_VALUE		999
//====================[P]=======[I]=======[D]====//
#define ROLL_PID	4.4,	0.0,	0.03
#define PITCH_PID	4.4,	0.0,	0.03
#define YAW_PID		6.0,	0.0,	0.0

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
L3G4_t L3G4;
ppm_t ppm;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
	(void) file;
	if (HAL_OK
			!= HAL_UART_Transmit(&huart1, (uint8_t*) ptr, len, HAL_MAX_DELAY)) {
		return 0;
	}
	return len;
}

int32_t map(int32_t value, int32_t input_min, int32_t input_max,
		int32_t output_min, int32_t output_max);
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_ADC1_Init();
	MX_SPI3_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_USART1_UART_Init();
	MX_TIM7_Init();
	/* USER CODE BEGIN 2 */
	printf("[MINIQUAD STARTED]\n");
	HAL_GPIO_WritePin(LED_USR_GPIO_Port, LED_USR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_DBG_GPIO_Port, LED_DBG_Pin, GPIO_PIN_RESET);

	printf("Starting Timer base\n");
	HAL_TIM_Base_Start(&htim7);

	printf("Starting PPM decoder\n");
	PPM_Init(&ppm, 8);
	PPM_InstallDriver(&ppm, &htim2, TIM_CHANNEL_1);
	PPM_Start(&ppm);

	printf("Starting PWM\n");
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	SET_MOTOR_1(0);
	SET_MOTOR_2(0);
	SET_MOTOR_3(0);
	SET_MOTOR_4(0);

	printf("Setting sensor\n");
	L3G4_Init(&L3G4);
	L3G4_InstallDriver(&L3G4, &hspi3);
	L3G4_InstallGPIO(&L3G4, CS3_GPIO_Port, CS3_Pin, DRDY_Pin);
	while (L3G4_OK != L3G4_CheckConnect(&L3G4)) {
		HAL_GPIO_WritePin(LED_DBG_GPIO_Port, LED_DBG_Pin, GPIO_PIN_SET);
		HAL_Delay(50);
		HAL_GPIO_WritePin(LED_DBG_GPIO_Port, LED_DBG_Pin, GPIO_PIN_RESET);
		HAL_Delay(1000);
	}
	HAL_GPIO_WritePin(LED_DBG_GPIO_Port, LED_DBG_Pin, GPIO_PIN_RESET);
	printf("Configuring sensor\n");
	L3G4_Config(&L3G4);

	printf("Calibrating sensor\n");
//	L3G4_Calib(&L3G4, 500);
	printf("Calibrating finished: %d, %d, %d\n",
			(int16_t) L3G4_GetCalibX(&L3G4), (int16_t) L3G4_GetCalibY(&L3G4),
			(int16_t) L3G4_GetCalibZ(&L3G4));

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
#if DEBUG_LED_EN == 1
	uint32_t lastLedTime = 0;
#endif

#if DEBUG_SS_RAW_EN == 1
	uint32_t lastSSRawTime = 0;
#endif

#if DEBUG_SS_FIL_EN == 1
	uint32_t lastSSFilterTime = 0;
#endif

#if	DEBUG_RAW_RECV_EN == 1
	uint32_t lastRawRecvTime = 0;
#endif

#if	DEBUG_MAP_RECV_EN == 1
	uint32_t lastMapRecvTime = 0;
#endif

#if DEBUG_PID_EN == 1
	uint32_t lastPidTime = 0;
#endif

#if DEBUG_MOTOR_EN == 1
	uint32_t lastMotorTime = 0;
#endif

	float dT;
	uint32_t lastT = 0, T;
	int16_t motorValue1, motorValue2, motorValue3, motorValue4;
	pidHelper_t pidRoll, pidPitch, pidYaw;
	float outputRoll, outputPitch, outputYaw;
	float readRoll, readPitch, readYaw;
	float inRoll = 0, inPitch = 0, inYaw = 0;
	float setRoll = 0, setPitch = 0, setYaw = 0, setThr = 0, setOffset = 0;

	printf("Setting PID\n");
	PID_Setup(&pidRoll, ROLL_PID, MAX_MOTOR_VALUE);
	PID_Setup(&pidPitch, PITCH_PID, MAX_MOTOR_VALUE);
	PID_Setup(&pidYaw, YAW_PID, MAX_MOTOR_VALUE);

	while (1) {
		if (L3G4_OK == L3G4_CheckDataAvailable(&L3G4)) {

			L3G4_ReadAll(&L3G4);

			/* Calculate dT */
			T = __HAL_TIM_GET_COUNTER(&htim7);
			if (T > lastT)
				dT = (float) (T - lastT) / 1000000.0;
			else
				dT = (float) (0xFFFF - lastT + T + 1) / 1000000.0;
			lastT = T;

			/* Get last data storage */
			readRoll = L3G4_GetLastX(&L3G4);
			readPitch = L3G4_GetLastY(&L3G4);
			readYaw = L3G4_GetLastZ(&L3G4);
			inRoll = inRoll * 0.0 + (readRoll * 1);
			inPitch = inPitch * 0.0 + (readPitch * 1);
			inYaw = inYaw * 0.0 + (readYaw * 1);

#if DEBUG_SS_RAW_EN == 1
			if (HAL_GetTick() - lastSSRawTime >= 20) {
				lastSSRawTime = HAL_GetTick();
				printf("%d, %d, %d\n", (int16_t) inRoll, (int16_t) inPitch,
						(int16_t) inYaw);
			}
#endif

#if DEBUG_SS_FIL_EN == 1
			if (HAL_GetTick() - lastSSFilterTime >= 20) {
				lastSSFilterTime = HAL_GetTick();
				printf("%d, %d, %d\n", (int16_t) readRoll, (int16_t) readPitch,
						(int16_t) readYaw);
			}
#endif
			setOffset = PPM_GetValue(&ppm, 7);
			if (setOffset == 0) {
				setRoll = setPitch = setYaw = setThr = 0;
			} else {
				setRoll = PPM_GetValue(&ppm, 0) - setOffset;
				setRoll = map(setRoll, -500, 500, -30, 30);
				setPitch = PPM_GetValue(&ppm, 1) - setOffset;
				setPitch = map(setPitch, -500, 500, -30, 30);
				setYaw = PPM_GetValue(&ppm, 3) - setOffset;
				setYaw = map(setYaw, -500, 500, 30, -30);
				setThr = PPM_GetValue(&ppm, 2) - setOffset + 500;
			}

#if DEBUG_RAW_RECV_EN == 1
			if (HAL_GetTick() - lastRawRecvTime >= 20) {
				lastRawRecvTime = HAL_GetTick();
				printf("%d, %d, %d, %d, %d, %d, %d, %d\n", PPM_GetValue(&ppm, 0), PPM_GetValue(&ppm, 1),
						PPM_GetValue(&ppm, 2), PPM_GetValue(&ppm, 3), PPM_GetValue(&ppm, 4),
						PPM_GetValue(&ppm, 5), PPM_GetValue(&ppm, 6), PPM_GetValue(&ppm, 7));
			}
#endif

#if DEBUG_MAP_RECV_EN == 1
			if (HAL_GetTick() - lastMapRecvTime >= 20) {
				lastMapRecvTime = HAL_GetTick();
				printf("%d, %d, %d, %d\n", (int16_t) setRoll,
						(int16_t) setPitch, (int16_t) setYaw, (int16_t) setThr);
			}
#endif

			outputRoll = PID_Calculate(&pidRoll, inRoll, setRoll, dT);
			outputPitch = PID_Calculate(&pidPitch, inPitch, setPitch, dT);
			outputYaw = PID_Calculate(&pidYaw, inYaw, setYaw, dT);

#if DEBUG_PID_EN == 1
			if (HAL_GetTick() - lastPidTime >= 20) {
				lastPidTime = HAL_GetTick();
				printf("%ld, %ld, %ld\n", (int32_t) outputRoll,
						(int32_t) outputPitch, (int32_t) outputYaw);
			}
#endif

			motorValue1 = setThr + (+outputRoll / 2.0) + (-outputPitch / 2.0)
					+ (-outputYaw / 2.0);
			motorValue2 = setThr + (-outputRoll / 2.0) + (-outputPitch / 2.0)
					+ (+outputYaw / 2.0);
			motorValue3 = setThr + (-outputRoll / 2.0) + (+outputPitch / 2.0)
					+ (-outputYaw / 2.0);
			motorValue4 = setThr + (+outputRoll / 2.0) + (+outputPitch / 2.0)
					+ (+outputYaw / 2.0);

			/* Limit motor value */
			if (setThr < 200) {
				motorValue1 = motorValue2 = motorValue3 = motorValue4 = 0;
			}

			if (motorValue1 > 999) {
				HAL_GPIO_WritePin(LED_USR_GPIO_Port, LED_USR_Pin, GPIO_PIN_SET);
				motorValue1 = 999;
			} else if (motorValue1 < 200) {
				motorValue1 = 0;
			}

			if (motorValue2 > 999) {
				HAL_GPIO_WritePin(LED_USR_GPIO_Port, LED_USR_Pin, GPIO_PIN_SET);
				motorValue2 = 999;
			} else if (motorValue2 < 200) {
				motorValue2 = 0;
			}

			if (motorValue3 > 999) {
				HAL_GPIO_WritePin(LED_USR_GPIO_Port, LED_USR_Pin, GPIO_PIN_SET);
				motorValue3 = 999;
			} else if (motorValue3 < 200) {
				motorValue3 = 0;
			}

			if (motorValue4 > 999) {
				HAL_GPIO_WritePin(LED_USR_GPIO_Port, LED_USR_Pin, GPIO_PIN_SET);
				motorValue4 = 999;
			} else if (motorValue4 < 200) {
				motorValue4 = 0;
			}

			SET_MOTOR_1(motorValue1);
			SET_MOTOR_2(motorValue2);
			SET_MOTOR_3(motorValue3);
			SET_MOTOR_4(motorValue4);

#if DEBUG_MOTOR_EN == 1
			if (HAL_GetTick() - lastMotorTime >= 20) {
				lastMotorTime = HAL_GetTick();
				printf("%d, %d, %d, %d\n", (int16_t) motorValue1,
						(int16_t) motorValue2, (int16_t) motorValue3,
						(int16_t) motorValue4);
			}
#endif

#if DEBUG_LED_EN == 1
		if (HAL_GetTick() - lastLedTime >= 1000) {
			lastLedTime = HAL_GetTick();

			HAL_GPIO_TogglePin(LED_DBG_GPIO_Port, LED_DBG_Pin);
		}
#endif
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == DRDY_Pin) {
		L3G4_InterruptCallback(&L3G4, GPIO_Pin);
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		PPM_InterruptCallback(&ppm);
	}
}

int32_t map(int32_t value, int32_t input_min, int32_t input_max,
		int32_t output_min, int32_t output_max) {
	int32_t v1 = (value - input_min) * 1000 / (input_max - input_min);
	int32_t v2 = v1 * (output_max - output_min) / 1000;
	return v2 + output_min;
}
/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM14 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM14) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
		HAL_GPIO_TogglePin(LED_DBG_GPIO_Port, LED_DBG_Pin);
		HAL_Delay(200);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
