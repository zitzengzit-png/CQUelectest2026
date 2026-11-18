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
#define LB_Pin GPIO_PIN_10
#define LB_GPIO_Port GPIOD
#define LF_Pin GPIO_PIN_11
#define LF_GPIO_Port GPIOD
#define LECHO1_Pin GPIO_PIN_12
#define LECHO1_GPIO_Port GPIOD
#define LECHO2_Pin GPIO_PIN_13
#define LECHO2_GPIO_Port GPIOD
#define LCD_SCL_Pin GPIO_PIN_6
#define LCD_SCL_GPIO_Port GPIOB
#define LCD_SDA_Pin GPIO_PIN_7
#define LCD_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
