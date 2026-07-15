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

#include "stm32f7xx_ll_adc.h"
#include "stm32f7xx_ll_dma.h"
#include "stm32f7xx_ll_rcc.h"
#include "stm32f7xx_ll_bus.h"
#include "stm32f7xx_ll_system.h"
#include "stm32f7xx_ll_exti.h"
#include "stm32f7xx_ll_cortex.h"
#include "stm32f7xx_ll_utils.h"
#include "stm32f7xx_ll_pwr.h"
#include "stm32f7xx_ll_tim.h"
#include "stm32f7xx_ll_usart.h"
#include "stm32f7xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

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
#define POT_A_Pin LL_GPIO_PIN_5
#define POT_A_GPIO_Port GPIOF
#define POT_B_Pin LL_GPIO_PIN_0
#define POT_B_GPIO_Port GPIOC
#define Y_VAL_Pin LL_GPIO_PIN_3
#define Y_VAL_GPIO_Port GPIOC
#define DIRECTION_MOTOR_B__Pin LL_GPIO_PIN_5
#define DIRECTION_MOTOR_B__GPIO_Port GPIOA
#define DIRECTION_MOTOR_B_A6_Pin LL_GPIO_PIN_6
#define DIRECTION_MOTOR_B_A6_GPIO_Port GPIOA
#define X_VAL_Pin LL_GPIO_PIN_3
#define X_VAL_GPIO_Port GPIOA
#define DIRECTION_MOTOR_A__Pin LL_GPIO_PIN_9
#define DIRECTION_MOTOR_A__GPIO_Port GPIOE
#define DIRECTION_MOTOR_A_E11_Pin LL_GPIO_PIN_11
#define DIRECTION_MOTOR_A_E11_GPIO_Port GPIOE
#define PWM_A_Pin LL_GPIO_PIN_13
#define PWM_A_GPIO_Port GPIOE
#define PWM_B_Pin LL_GPIO_PIN_6
#define PWM_B_GPIO_Port GPIOC
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

/* USER CODE BEGIN Private defines */
/* Pin map (see Docs/20260708_1119_Plan_Migracion_Arduino_a_STM32.md §3.3):
 *   Y_VAL  PC3  ADC1 IN13  -> telemetría Sensor1 (eje Y)
 *   X_VAL  PA3  ADC2 IN3   -> telemetría Sensor2 (eje X); Arduino A0 en NUCLEO
 *   POT_A  PF5  ADC3 IN15  |  POT_B PC0 ADC3 IN10  (modo MANUAL)
 *   PA7 ya no se usa (antes IN7; reemplazado por PA3 por linealidad en banco). */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
