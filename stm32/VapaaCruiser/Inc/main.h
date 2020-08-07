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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define US_LF_Trigger_Pin GPIO_PIN_0
#define US_LF_Trigger_GPIO_Port GPIOC
#define US_RF_Trigger_Pin GPIO_PIN_1
#define US_RF_Trigger_GPIO_Port GPIOC
#define US_LB_Trigger_Pin GPIO_PIN_2
#define US_LB_Trigger_GPIO_Port GPIOC
#define US_RB_Trigger_Pin GPIO_PIN_3
#define US_RB_Trigger_GPIO_Port GPIOC
#define US_F_Trigger_Pin GPIO_PIN_1
#define US_F_Trigger_GPIO_Port GPIOA
#define US_B_Trigger_Pin GPIO_PIN_2
#define US_B_Trigger_GPIO_Port GPIOA
#define US_F_Receive_Pin GPIO_PIN_6
#define US_F_Receive_GPIO_Port GPIOA
#define US_B_Receive_Pin GPIO_PIN_7
#define US_B_Receive_GPIO_Port GPIOA
#define US_LB_Receive_Pin GPIO_PIN_0
#define US_LB_Receive_GPIO_Port GPIOB
#define US_RB_Receive_Pin GPIO_PIN_1
#define US_RB_Receive_GPIO_Port GPIOB
#define US_LF_Receive_Pin GPIO_PIN_10
#define US_LF_Receive_GPIO_Port GPIOB
#define US_RF_Receive_Pin GPIO_PIN_11
#define US_RF_Receive_GPIO_Port GPIOB
#define Motor_Forward_Pin GPIO_PIN_6
#define Motor_Forward_GPIO_Port GPIOC
#define Motor_Turn_Pin GPIO_PIN_7
#define Motor_Turn_GPIO_Port GPIOC
#define Master_TX_Pin GPIO_PIN_9
#define Master_TX_GPIO_Port GPIOA
#define Master_RX_Pin GPIO_PIN_10
#define Master_RX_GPIO_Port GPIOA
#define GPS_TX_Pin GPIO_PIN_10
#define GPS_TX_GPIO_Port GPIOC
#define GPS_RX_Pin GPIO_PIN_11
#define GPS_RX_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/