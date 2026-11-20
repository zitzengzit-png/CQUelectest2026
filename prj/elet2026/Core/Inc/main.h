/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#define KEY0_Pin GPIO_PIN_4
#define KEY0_GPIO_Port GPIOE
#define KEY0_EXTI_IRQn EXTI4_IRQn
#define F7_Pin GPIO_PIN_0
#define F7_GPIO_Port GPIOF
#define F6_Pin GPIO_PIN_2
#define F6_GPIO_Port GPIOF
#define F5_Pin GPIO_PIN_4
#define F5_GPIO_Port GPIOF
#define BEEP_LED_Pin GPIO_PIN_7
#define BEEP_LED_GPIO_Port GPIOF
#define START_Pin GPIO_PIN_8
#define START_GPIO_Port GPIOF
#define START_EXTI_IRQn EXTI9_5_IRQn
#define STBY_Pin GPIO_PIN_1
#define STBY_GPIO_Port GPIOC
#define KEY_UP_Pin GPIO_PIN_0
#define KEY_UP_GPIO_Port GPIOA
#define KEY_UP_EXTI_IRQn EXTI0_IRQn
#define RPWM_Pin GPIO_PIN_1
#define RPWM_GPIO_Port GPIOA
#define LPWM_Pin GPIO_PIN_2
#define LPWM_GPIO_Port GPIOA
#define RF_Pin GPIO_PIN_4
#define RF_GPIO_Port GPIOA
#define RB_Pin GPIO_PIN_5
#define RB_GPIO_Port GPIOA
#define RECHO1_Pin GPIO_PIN_6
#define RECHO1_GPIO_Port GPIOA
#define RECHO2_Pin GPIO_PIN_7
#define RECHO2_GPIO_Port GPIOA
#define F1_Pin GPIO_PIN_8
#define F1_GPIO_Port GPIOE
#define F8_Pin GPIO_PIN_15
#define F8_GPIO_Port GPIOB
#define F4_Pin GPIO_PIN_9
#define F4_GPIO_Port GPIOD
#define LB_Pin GPIO_PIN_10
#define LB_GPIO_Port GPIOD
#define LF_Pin GPIO_PIN_11
#define LF_GPIO_Port GPIOD
#define LECHO1_Pin GPIO_PIN_12
#define LECHO1_GPIO_Port GPIOD
#define LECHO2_Pin GPIO_PIN_13
#define LECHO2_GPIO_Port GPIOD
#define F3_Pin GPIO_PIN_14
#define F3_GPIO_Port GPIOD
#define F2_Pin GPIO_PIN_15
#define F2_GPIO_Port GPIOD
#define B2_Pin GPIO_PIN_6
#define B2_GPIO_Port GPIOG
#define B1_Pin GPIO_PIN_8
#define B1_GPIO_Port GPIOG
#define B8_Pin GPIO_PIN_6
#define B8_GPIO_Port GPIOC
#define B3_Pin GPIO_PIN_8
#define B3_GPIO_Port GPIOC
#define B5_Pin GPIO_PIN_11
#define B5_GPIO_Port GPIOC
#define B4_Pin GPIO_PIN_12
#define B4_GPIO_Port GPIOC
#define B7_Pin GPIO_PIN_6
#define B7_GPIO_Port GPIOD
#define B6_Pin GPIO_PIN_15
#define B6_GPIO_Port GPIOG
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
