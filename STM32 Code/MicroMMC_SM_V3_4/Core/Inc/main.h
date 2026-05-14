/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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
#define Viso_Pin GPIO_PIN_14
#define Viso_GPIO_Port GPIOC
#define nFAULT_Pin GPIO_PIN_15
#define nFAULT_GPIO_Port GPIOC
#define VREF_ADC_Pin GPIO_PIN_0
#define VREF_ADC_GPIO_Port GPIOA
#define EN1_Pin GPIO_PIN_1
#define EN1_GPIO_Port GPIOA
#define Vout1_Pin GPIO_PIN_2
#define Vout1_GPIO_Port GPIOA
#define CM1_Pin GPIO_PIN_3
#define CM1_GPIO_Port GPIOA
#define PTP_Pin GPIO_PIN_4
#define PTP_GPIO_Port GPIOA
#define VREF_DAC_Pin GPIO_PIN_5
#define VREF_DAC_GPIO_Port GPIOA
#define CM2_Pin GPIO_PIN_6
#define CM2_GPIO_Port GPIOA
#define Vout2_Pin GPIO_PIN_7
#define Vout2_GPIO_Port GPIOA
#define Vsm_Pin GPIO_PIN_0
#define Vsm_GPIO_Port GPIOB
#define TIM1_TRIG_Pin GPIO_PIN_1
#define TIM1_TRIG_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_8
#define PWM1_GPIO_Port GPIOA
#define EN2_Pin GPIO_PIN_6
#define EN2_GPIO_Port GPIOC
#define PWM2_Pin GPIO_PIN_9
#define PWM2_GPIO_Port GPIOA
#define TIM2_TRIG_Pin GPIO_PIN_3
#define TIM2_TRIG_GPIO_Port GPIOB
#define LED_COM_Pin GPIO_PIN_4
#define LED_COM_GPIO_Port GPIOB
#define LED_STAT_Pin GPIO_PIN_5
#define LED_STAT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
