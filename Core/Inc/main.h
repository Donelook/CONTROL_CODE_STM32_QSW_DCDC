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
#include "stm32g4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RESET_INTERLOCK_Pin GPIO_PIN_5
#define RESET_INTERLOCK_GPIO_Port GPIOE
#define DEADTIME_TR_Pin GPIO_PIN_0
#define DEADTIME_TR_GPIO_Port GPIOC
#define INTERLOCK_Pin GPIO_PIN_2
#define INTERLOCK_GPIO_Port GPIOC
#define RESET_FPGA_Pin GPIO_PIN_2
#define RESET_FPGA_GPIO_Port GPIOF
#define IMAX_Pin GPIO_PIN_4
#define IMAX_GPIO_Port GPIOA
#define IMAX2_Pin GPIO_PIN_5
#define IMAX2_GPIO_Port GPIOA
#define IMIN_Pin GPIO_PIN_6
#define IMIN_GPIO_Port GPIOA
#define START_STOP_FPGA_Pin GPIO_PIN_5
#define START_STOP_FPGA_GPIO_Port GPIOC
#define CS_VREF_1_Pin GPIO_PIN_1
#define CS_VREF_1_GPIO_Port GPIOB
#define PCB_TEMP_Pin GPIO_PIN_7
#define PCB_TEMP_GPIO_Port GPIOE
#define CS_VREF_2_Pin GPIO_PIN_9
#define CS_VREF_2_GPIO_Port GPIOE
#define INPUT_V_ADC_Pin GPIO_PIN_13
#define INPUT_V_ADC_GPIO_Port GPIOE
#define OUTPUT_V_ADC_Pin GPIO_PIN_14
#define OUTPUT_V_ADC_GPIO_Port GPIOE
#define HEATSINK_TEMP_Pin GPIO_PIN_13
#define HEATSINK_TEMP_GPIO_Port GPIOB
#define FAN_PWM_Pin GPIO_PIN_12
#define FAN_PWM_GPIO_Port GPIOD
#define NOT_RST_4_Pin GPIO_PIN_6
#define NOT_RST_4_GPIO_Port GPIOC
#define DEADTIME_HC_Pin GPIO_PIN_7
#define DEADTIME_HC_GPIO_Port GPIOC
#define IMAX2_SUM_Pin GPIO_PIN_8
#define IMAX2_SUM_GPIO_Port GPIOA
#define READY_4_Pin GPIO_PIN_1
#define READY_4_GPIO_Port GPIOD
#define READY_3_Pin GPIO_PIN_2
#define READY_3_GPIO_Port GPIOD
#define READY_2_Pin GPIO_PIN_3
#define READY_2_GPIO_Port GPIOD
#define READY_1_Pin GPIO_PIN_4
#define READY_1_GPIO_Port GPIOD
#define NOT_FAULT_3_Pin GPIO_PIN_6
#define NOT_FAULT_3_GPIO_Port GPIOD
#define NOT_FAULT_2_Pin GPIO_PIN_7
#define NOT_FAULT_2_GPIO_Port GPIOD
#define NOT_FAULT_1_Pin GPIO_PIN_3
#define NOT_FAULT_1_GPIO_Port GPIOB
#define NOT_RST_3_Pin GPIO_PIN_5
#define NOT_RST_3_GPIO_Port GPIOB
#define NOT_RST_2_Pin GPIO_PIN_6
#define NOT_RST_2_GPIO_Port GPIOB
#define NOT_RST_1_Pin GPIO_PIN_7
#define NOT_RST_1_GPIO_Port GPIOB
#define CS_FAULT_2_Pin GPIO_PIN_8
#define CS_FAULT_2_GPIO_Port GPIOB
#define CS_OCD_2_Pin GPIO_PIN_9
#define CS_OCD_2_GPIO_Port GPIOB
#define CS_FAULT_1_Pin GPIO_PIN_0
#define CS_FAULT_1_GPIO_Port GPIOE
#define CS_OCD_1_Pin GPIO_PIN_1
#define CS_OCD_1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
