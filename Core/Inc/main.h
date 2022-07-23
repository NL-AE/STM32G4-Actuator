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

// FOC related
#define PI 		3.14159274101f
#define PI2		6.28318530718f
#define SQRT3 	1.73205080757f
#define SQRT3_2	0.86602540378f
#define SQRT1_3	0.57735026919f

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

// ADC
void  ADC_Get_Raw    (int16_t*i_a_Raw, int16_t*i_b_Raw, int16_t*PVDD_Raw, int16_t*Temp_Raw);	// Reads all ADCs
void  ADC_Filter_Curr(int16_t i_a_Raw, int16_t i_b_Raw, int16_t*i_a_Fil, int16_t*i_b_Fil);		// Put ADC readings into filter
void  ADC_Norm_Curr  (int16_t i_a_Fil, int16_t i_b_Fil, float*i_a, float*i_b);					// Normalise ADC values to currents
void  ADC_Filter_Misc(uint16_t PVDD_Raw, uint16_t Temp_Raw, uint16_t*PVDD_Fil, uint16_t*Temp_Fil);	// Put ADC readings into filter
void  ADC_Norm_Misc  (uint16_t PVDD_Fil, uint16_t Temp_Fil, float*PVDD, float*Temp);				// Normalise ADC values to properties
// Encoder
void  ENC_Read_Ang(float*Angle);				// ask for encoder angle over SPI
void  ENC_Read_Vel(float*Velocity);				// ask for encoder velocity over SPI
void  ENC_Write(uint8_t com1, uint8_t com2, uint8_t data1, uint8_t data2, uint8_t mask1, uint8_t mask2);
void  ENC_Interrupt(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void  ENC_Filter (int16_t IIF_Raw, uint32_t dIIF_Raw, int16_t*IIF_Fil, uint32_t*dIIF_Fil);	// Filter
void  ENC_Norm   (int16_t IIF_Fil, uint32_t dIIF_Fil, float*theta, float*dtheta);			// Normalise encoder values
// FOC stuff
void  Set_PWM3(float DC_1, float DC_2, float DC_3);		// set duty cycle values for channels A,B,C
float _sin(float theta);								// sin(theta)
float _cos(float theta);								// cos(theta)
// Interrupts
void  FOC_RCR(void);			// RCR interrupt
void  FOC_Interrupt(void);		// FOC interrupt
void  CAN_Interrupt(void);		// CAN RX interrupt

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
#define IF_A_EXTI_IRQn EXTI9_5_IRQn
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
#define SPI_CS_Pin GPIO_PIN_15
#define SPI_CS_GPIO_Port GPIOA
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
