/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define XTAL_1_Pin GPIO_PIN_0
#define XTAL_1_GPIO_Port GPIOF
#define XTAL_2_Pin GPIO_PIN_1
#define XTAL_2_GPIO_Port GPIOF
#define PWM_C_Pin GPIO_PIN_0
#define PWM_C_GPIO_Port GPIOA
#define PWM_B_Pin GPIO_PIN_2
#define PWM_B_GPIO_Port GPIOA
#define PWM_A_Pin GPIO_PIN_3
#define PWM_A_GPIO_Port GPIOA
#define IF_A_Pin GPIO_PIN_5
#define IF_A_GPIO_Port GPIOA
#define IF_B_Pin GPIO_PIN_6
#define IF_B_GPIO_Port GPIOA
#define T_Sen_Pin GPIO_PIN_7
#define T_Sen_GPIO_Port GPIOA
#define V_Sen_Pin GPIO_PIN_0
#define V_Sen_GPIO_Port GPIOB
#define SO_A_Pin GPIO_PIN_1
#define SO_A_GPIO_Port GPIOB
#define SO_B_Pin GPIO_PIN_2
#define SO_B_GPIO_Port GPIOB
#define LED_Y_Pin GPIO_PIN_12
#define LED_Y_GPIO_Port GPIOB
#define LED_G_Pin GPIO_PIN_14
#define LED_G_GPIO_Port GPIOB
#define SPI_NSS_Pin GPIO_PIN_15
#define SPI_NSS_GPIO_Port GPIOA
#define SPI_SCK_Pin GPIO_PIN_10
#define SPI_SCK_GPIO_Port GPIOC
#define SPI_MOSI_Pin GPIO_PIN_5
#define SPI_MOSI_GPIO_Port GPIOB
#define CAN_RX_Pin GPIO_PIN_8
#define CAN_RX_GPIO_Port GPIOB
#define CAN_TX_Pin GPIO_PIN_9
#define CAN_TX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
