/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <stdint.h>
#include <math.h>
#include "sin_lookup.h"
#include "stm32g4xx_it.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Code
#define REV 1		// code revision
#define PRINT 1		// want prints?

// Phase Mappings
#define Phase_A_Ch TIM_CHANNEL_4
#define Phase_B_Ch TIM_CHANNEL_3
#define Phase_C_Ch TIM_CHANNEL_1

// CAN ID (actuator)
#define CAN_TX_ID 2		// Transmits as this ID
#define CAN_RX_ID 1		// Receives messages from this ID

// Res Div.:	V_o = V_in * R_Bot / (R_Top + B_Bot)
#define R_Top 255.0f		// R divider - top resistor			/ KOhms
#define R_Bot 10.0f			// R divider - bottom resistor		/ KOhms

// LM60: V_o = (6.25mV * T/C) + 424mV
#define T_Slope  0.00625f	// T sensor  - slope	/V
#define T_Offset 0.424f		// T sensor  - offset	/v

// Op Amp
#define R_Shunt   0.001f    // Shunt resistor /ohms
#define OP_Gain   40.0f		// Opamp gain
#define OP_Offset 40.0f		// Opamp offset

// Macros
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Encoder
typedef struct
{
	float SPI_theta;			// IIF from SPI 0-360deg used to zero

	int16_t  IIF_Counter;		// The counter variable that is interrupt driven so dont use it in calculations
	int64_t  IIF_Revolutions;	// Number of full revolutions taken
	uint16_t IIF_Raw;			// Current angle 0-4095
} ENC_Struct;
// ADC
typedef struct
{
	uint16_t ADC1_DMA_Buff[2];		// DMA buffer for ADC 1
	uint16_t ADC2_DMA_Buff[2];		// DMA buffer for ADC 2

	float V_Bus;	// V bus
	float Temp_C;	// Temp in C

	uint16_t i_a_Raw, i_b_Raw, V_Raw, T_Raw;	// Raw ADC readings
	uint16_t i_a_Fil, i_b_Fil, V_Fil, T_Fil;	// Filtered ADC readings
} ADC_Struct;
// FOC
typedef struct
{
	// Proprties
	int Pole_Pairs;				// number of pole pairs
	float dt;					// delta T of FOC response		/seconds

	// Angles
	float m_theta, m_dtheta;	// mechanical theta and dtheta	/deg		/degs-1
	float e_theta, e_dtheta;	// electrical theta and dtheta	/deg		/degs-1

	// FOC currents
	float i_a, i_b, i_c;		// Phase currents				/amps
	float i_alph, i_beta;		// Alpha/Beta currents			/amps
	float i_d, i_q;				// Direct/Quadrature currents	/amps

	// Current target
	float DC_I;					// 0 to 1 of the duty cycle controlling current

	// FOC sector control
	float alpha;				// Angle from start of sector to current position
	int sector;					// which sector currently in

	// Duty cycles
	float DC_1, DC_2, DC_0;		// Duty cycle of vector start, end and unforced

	uint16_t PWM_Reg_Max;		// PWM register max
	float PWM_A, PWM_C, PWM_B;	// Duty cycle
} FOC_Struct;
// Filter
typedef struct
{
	// Butterworth from: https://www.meme.net.au/butterworth.html
	// At 6.667KHz smapling
	// y(i) = k1*x(i) + k1*x(i-1) + k2*y(i-1)

	// Filters i_a, i_b, PVDD, Temp, IIF count, IIF vel

	float i_k[2];		// Filter coefficients for current filters
	float Misc_k[2];	// Filter coefficients for misc filter

	int16_t i_a_Pre, i_a_Pre_Fil, i_b_Pre, i_b_Pre_Fil;			// Previous values for current
	int16_t PVDD_Pre, PVDD_Pre_Fil, Temp_Pre, Temp_Pre_Fil;		// Previous values for PVDD and temp
} FIL_Struct;
// CAN
typedef struct
{
	uint32_t timeout;		// timeout/ms

	FDCAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[6];

	FDCAN_TxHeaderTypeDef tx_header;
	uint8_t tx_data[6];

	FDCAN_FilterTypeDef filter;
} CAN_Struct;
// Controller
typedef struct
{
	float Torque;
	float Velocity;
} CON_Struct;

ENC_Struct enc;
ADC_Struct adc;
FOC_Struct foc;
FIL_Struct fil;
CAN_Struct can;
CON_Struct con;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// For SWD debug port 0 printf()
int _write(int file, char *ptr, int len)
{
	int i=0;
	for(i=0; i<len; i++)
		ITM_SendChar((*ptr++));
	return len;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_FDCAN1_Init();
  MX_TIM2_Init();
  MX_SPI3_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  printf("\nActuator Firmware Version: %i\n",REV);

  /* Start ADCs */
  printf("Start ADC... ");
  HAL_ADCEx_Calibration_Start(&hadc1,LL_ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2,LL_ADC_SINGLE_ENDED);

  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);
//  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc.ADC1_DMA_Buff, 2);
//  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc.ADC2_DMA_Buff, 2);

  printf("Good\n");

  /* Start Timers */
  printf("Start TIM... ");
  HAL_TIM_Base_Start(&htim2);
  while(__HAL_TIM_GET_COUNTER(&htim2)<858+20){}	// wait for cycles to sync up clocks
  	  	  	  	  	  	  	  	  	  	  	  	// 858 cycles for middle
  	  	  	  	  	  	  	  	  	  	  	    // each cycle delays by around 7.5ns
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Start(&htim2, Phase_A_Ch);
  HAL_TIM_PWM_Start(&htim2, Phase_B_Ch);
  HAL_TIM_PWM_Start(&htim2, Phase_C_Ch);
  Set_PWM3(0,0,0);							// Set PWM channels to same value
  printf("Good\n");

  /* Start Encoder */
  printf("Start ENC... ");
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 1);

  // command is
  // 15 	- 0 write/1 read
  // 14-11	- lock				0000 default	1010 for 0x05h-0x11h
  // 10		- 0 access to current values/1 access all in buffer
  // 9-4	- 6 bit address
  // 3-0	- 4 bit number of data words	if 0000, no safety word

  // 		command1, 	command2,   data1,	    data2,      mask1,      mask2
  printf("\n");
//  ENC_Write(0b11010000, 0b01100001, 0b01000000, 0b00000001, 0b11000000, 0b00010111);		// write MOD_1	06 register		A/B
  ENC_Write(0b11010000, 0b01100001, 0b01000000, 0b00000010, 0b11000000, 0b00010111);		// write MOD_1	06 register		step/dir
  ENC_Write(0b11010000, 0b10000001, 0b00001000, 0b00000001, 0b01111111, 0b11111111);		// write MOD_2	08 register
  ENC_Write(0b11010000, 0b10010001, 0b00000000, 0b00000000, 0b11111111, 0b11111111);		// write MOD_3  09 register
  ENC_Write(0b11010000, 0b11010001, 0b00000000, 0b00001001, 0b11111111, 0b11111111);		// write IFAB	0D register	(13)
  ENC_Write(0b11010000, 0b11100001, 0b00000000, 0b10000000, 0b00000001, 0b11111011);		// write MOD_4	0E register (14)

  ENC_Read_Ang(&enc.SPI_theta);
  enc.IIF_Counter = (int)(enc.SPI_theta /360.0f * 4095.0f);	// Zero encoder

  /* Start CAN */
  printf("Start CAN... ");
  // can code
  printf("Good\n");

  /* Setup FOC structure*/
  foc.Pole_Pairs = 21.0f;
  foc.dt = (float)(2.0f/(170.0f*1000000.0f/(htim2.Init.Period+1)/(htim2.Init.RepetitionCounter+1)));
  foc.PWM_Reg_Max = htim2.Init.Period;

  /* Setup Filter structure */
  fil.i_k[0]    = 0.421f;	fil.i_k[1]    = 0.158f;
  fil.Misc_k[0] = 0.421f;	fil.Misc_k[1] = 0.158f;

  printf("\nSetup complete!\n\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, 1);

	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_Start(&hadc2);

	  HAL_ADC_PollForConversion(&hadc1, 1);

	  adc.V_Raw = HAL_ADC_GetValue(&hadc1);
	  adc.T_Raw = HAL_ADC_GetValue(&hadc2);

	  ADC_Norm_Misc(adc.V_Raw, adc.T_Raw, &adc.V_Bus, &adc.Temp_C);

	  printf("ADC1: %i\tADC2: %i\n", adc.V_Raw, adc.T_Raw);

	  HAL_Delay(200);
	  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, 0);
	  HAL_Delay(800);



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// Read ADCs
void  ADC_Get_Raw    (int16_t*i_a_Raw, int16_t*i_b_Raw, int16_t*PVDD_Raw, int16_t*Temp_Raw)
{
//	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc.ADC1_DMA_Buff, 2);
//	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc.ADC2_DMA_Buff, 2);
//
////	while(adc.ADC1_CC==0 || adc.ADC2_CC==0){}	// wait for conversion to complete
//	adc.ADC1_CC=0;									// reset flags
//	adc.ADC2_CC=0;
//
//	*i_a_Raw	= adc.ADC1_DMA_Buff[0];
//	*i_b_Raw	= adc.ADC2_DMA_Buff[0];
//	*PVDD_Raw	= adc.ADC1_DMA_Buff[1];
//	*Temp_Raw	= adc.ADC2_DMA_Buff[1];
}
void  ADC_Filter_Curr(int16_t i_a_Raw, int16_t i_b_Raw, int16_t*i_a_Fil, int16_t*i_b_Fil)
{
	// Filter
	*i_a_Fil = fil.i_k[0]*i_a_Raw + fil.i_k[0]*fil.i_a_Pre + fil.i_k[1]*fil.i_a_Pre_Fil;
	*i_b_Fil = fil.i_k[0]*i_b_Raw + fil.i_k[0]*fil.i_b_Pre + fil.i_k[1]*fil.i_b_Pre_Fil;

	// Now store current values as previous values
	fil.i_a_Pre = i_a_Raw;
	fil.i_b_Pre = i_b_Raw;

	fil.i_a_Pre_Fil = *i_a_Fil;
	fil.i_b_Pre_Fil = *i_b_Fil;
}
void  ADC_Norm_Curr  (int16_t i_a_Fil, int16_t i_b_Fil, float*i_a, float*i_b)
{
	*i_a = (((float)(i_a_Fil-OP_Offset))*3.3f/4095.0f)/OP_Gain/R_Shunt;
	*i_b = (((float)(i_b_Fil-OP_Offset))*3.3f/4095.0f)/OP_Gain/R_Shunt;
}
void  ADC_Filter_Misc(uint16_t PVDD_Raw, uint16_t Temp_Raw, uint16_t*PVDD_Fil, uint16_t*Temp_Fil)
{
	// Filter
	*PVDD_Fil = fil.Misc_k[0]*PVDD_Raw + fil.Misc_k[0]*fil.PVDD_Pre + fil.Misc_k[1]*fil.PVDD_Pre_Fil;
	*Temp_Fil = fil.Misc_k[0]*Temp_Raw + fil.Misc_k[0]*fil.Temp_Pre + fil.Misc_k[1]*fil.Temp_Pre_Fil;

	// Now store current values as previous values
	fil.PVDD_Pre = PVDD_Raw;
	fil.Temp_Pre = Temp_Raw;

	fil.PVDD_Pre_Fil = *PVDD_Fil;
	fil.Temp_Pre_Fil = *Temp_Fil;
}
void  ADC_Norm_Misc  (uint16_t PVDD_Fil, uint16_t Temp_Fil, float*PVDD, float*Temp)
{
	*PVDD = (float)PVDD_Fil*3.3f/4095.0f / R_Bot * (R_Bot + R_Top);
	*Temp = (((float)Temp_Fil*3.3f/4095.0f)-T_Offset)/T_Slope;
}
// Encoder
void  ENC_Read_Ang(float*Angle)
{
	uint8_t ENC_ASK_POS   [2] = {0b10000000,0b00100001};	// Command for asking position
	uint8_t SPI_BUFF[2] = {0,0};

	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 0);
	HAL_SPI_Transmit(&hspi3, (uint8_t*)&ENC_ASK_POS, 2, 10);	// Ask for data
	HAL_SPI_Receive (&hspi3, (uint8_t*)SPI_BUFF    , 2, 10);	// Receive 4 bytes of data
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 1);

	int16_t SPI_ANG = (SPI_BUFF[0] << 8 | SPI_BUFF[1]);			// make 16 bit
	int16_t ANG_VAL = (0b0011111111111111 & SPI_ANG);					// keep last 14 bits
	ANG_VAL -= (((SPI_ANG)&(1UL<<(14)))>>(14))*(-16384);
	*Angle = 360.0f/32768.0f * ANG_VAL;
}
void  ENC_Read_Vel(float*Velocity)
{
	uint8_t ENC_ASK_VEL   [2] = {0b10000000,0b00110000};	// Command for asking velocity
	uint8_t SPI_BUFF[2] = {0,0};

	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 0);
	HAL_SPI_Transmit(&hspi3, (uint8_t*)&ENC_ASK_VEL, 2, 10);	// Ask for data
	HAL_SPI_Receive (&hspi3, (uint8_t*)SPI_BUFF    , 2, 10);	// Receive 2 bytes of data
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 1);

	int16_t SPI_VEL = (SPI_BUFF[0] << 8 | SPI_BUFF[1]);			// make 16 bit
	int16_t VEL_VAL = (0b0011111111111111 & SPI_VEL);			// keep last 14 bits
	*Velocity = 360.0f/32768.0f * VEL_VAL / 2.0f / 0.0000427f;
}
void  ENC_Write(uint8_t com1, uint8_t com2, uint8_t data1, uint8_t data2, uint8_t mask1, uint8_t mask2)
{
	uint8_t ADDR = (com2>>4)&0b00001111;
	printf("\tRegister 0x%02X... ",ADDR);

	// read
	uint8_t ENC_R_COM [2] = {com1|0b1000000,								// make into read command
							 com2};
	uint8_t SPI_BUFF[2] = {0,0};

//	printf("\n");
//	printf("\t\tTX: %c%c%c%c %c%c%c%c   %c%c%c%c %c%c%c%c\n", BYTE_TO_BINARY(ENC_R_COM[0]), BYTE_TO_BINARY(ENC_R_COM[1]));
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 0);
	HAL_SPI_Transmit(&hspi3, (uint8_t*)&ENC_R_COM, 2, 10);		// Read current register
	HAL_SPI_Receive (&hspi3, (uint8_t*)SPI_BUFF, 2, 10);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 1);
//	printf("\t\tRead  : %c%c%c%c %c%c%c%c   %c%c%c%c %c%c%c%c\n", BYTE_TO_BINARY(SPI_BUFF[0]), BYTE_TO_BINARY(SPI_BUFF[1]));

	// write
	uint8_t ENC_W_EOM [4] = {com1&0b01111111,								// make into write command
						     com2,											// same
							 (SPI_BUFF[0] & (~mask1)) | (data1 & mask1),	// keep read when mask=0, keep data when mask=1
							 (SPI_BUFF[1] & (~mask2)) | (data2 & mask2)};	// keep read when mask=0, keep data when mask=1

//	printf("\t\tTX    : %c%c%c%c %c%c%c%c   %c%c%c%c %c%c%c%c   %c%c%c%c %c%c%c%c   %c%c%c%c %c%c%c%c\n", BYTE_TO_BINARY(ENC_W_EOM[0]), BYTE_TO_BINARY(ENC_W_EOM[1]), BYTE_TO_BINARY(ENC_W_EOM[2]), BYTE_TO_BINARY(ENC_W_EOM[3]));
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 0);
	HAL_SPI_Transmit(&hspi3, (uint8_t*)&ENC_W_EOM, 4, 10);		// Write to register
	HAL_SPI_Receive (&hspi3, (uint8_t*)SPI_BUFF, 2, 10);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 1);
//	printf("\t\tS Word: %c%c%c%c %c%c%c%c   %c%c%c%c %c%c%c%c\n", BYTE_TO_BINARY(SPI_BUFF[0]), BYTE_TO_BINARY(SPI_BUFF[1]));

//	// check safety word
//	uint16_t SAFE_WORD = (SPI_BUFF[0] << 8 | SPI_BUFF[1]);		// form 16bit safety word
//
//	uint8_t ERR_1 = (SAFE_WORD >> 15)&1U;		// only care about first bit
//	uint8_t ERR_2 = (SAFE_WORD >> 14)&1U;		// shift left one then make it first
//	uint8_t ERR_3 = (SAFE_WORD >> 13)&1U;		// shift left 2 ...
//	uint8_t ERR_4 = (SAFE_WORD >> 12)&1U;		// ...
//
//	if(ERR_1==0 || ERR_2==0 || ERR_3==0 || ERR_4==0)
//	{
//		if(ERR_1==0){printf("\t\t\tErr 1: Reset/watchdog overflow!\n");}
//		if(ERR_2==0){printf("\t\t\tErr 2: System error!\n");}
//		if(ERR_3==0){printf("\t\t\tErr 3: Interface access error!\n");}
//		if(ERR_4==0){printf("\t\t\tErr 4: Invalid angle value!\n");}
//		Error_Handler();
//	}

	// check if written
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 0);
	HAL_SPI_Transmit(&hspi3, (uint8_t*)&ENC_R_COM, 2, 10);		// Read current register
	HAL_SPI_Receive (&hspi3, (uint8_t*)SPI_BUFF, 2, 10);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 1);
//	printf("\t\tNew   : %c%c%c%c %c%c%c%c   %c%c%c%c %c%c%c%c\n", BYTE_TO_BINARY(SPI_BUFF[0]), BYTE_TO_BINARY(SPI_BUFF[1]));

	if(((data1&mask1) != (SPI_BUFF[0]&mask1)) || (data2&mask2) != (SPI_BUFF[1]&mask2))
	{
		printf("Error Writing\n");
		Error_Handler();
	}

	printf("Good\n");
}
void  ENC_Interrupt(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))
		enc.IIF_Counter++;		// If high, increment
	else
		enc.IIF_Counter--;		// If low , decrement

	if(enc.IIF_Counter>=4096)	// If overflow
		enc.IIF_Counter = 0;		// Set to 0

	if(enc.IIF_Counter<0)		// If underflow
		enc.IIF_Counter = 4095;		// Set to 4095
}
// FOC stuff
void  Set_PWM3(float DC_1, float DC_2, float DC_3)
{
	__HAL_TIM_SET_COMPARE(&htim2,Phase_A_Ch,foc.PWM_Reg_Max*DC_1);	// Set PWM channels
	__HAL_TIM_SET_COMPARE(&htim2,Phase_B_Ch,foc.PWM_Reg_Max*DC_2);
	__HAL_TIM_SET_COMPARE(&htim2,Phase_C_Ch,foc.PWM_Reg_Max*DC_3);
}
float _sin(float theta)
{
	return sin_lookup[(int)floor(theta)];
}
float _cos(float theta)
{
	return sin_lookup[(int)floor(fmodf(theta+270.0f,360.0f))];
}
// Timer Interrupts
void  FOC_Interrupt(void)
{
	/* LED on */
	HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, 1);

//	/* FOC sample */
//	enc.IIF_Raw = enc.IIF_Counter;											// Copy current encoder angle
//	ADC_Get_Raw(&adc.i_a_Raw,&adc.i_b_Raw, &adc.PVDD_Raw, &adc.Temp_Raw);	// Read raw ADC
//
//	/* Filter and normalise readings */
//	ADC_Filter_Curr(adc.i_a_Raw,adc.i_b_Raw,&adc.i_a_Fil,&adc.i_b_Fil);		// Filter raw ADC currents
//	ADC_Norm_Curr  (adc.i_a_Fil,adc.i_b_Fil,&foc.i_a,&foc.i_b);				// Normalise currents
//	foc.m_theta = (float)enc.IIF_Raw / 4095.0f * 360.0f;					// Normalise angle to 0-360deg
//
//	/* FOC maths */
//	// Get electrical angles correct
//	foc.e_theta = fmodf(foc.m_theta*foc.Pole_Pairs,360.0f);	// get electrical angle and constrain in 360 deg
//
//	// Clarke -> alpha/beta
//	foc.i_alph = foc.i_a;
//	foc.i_beta = SQRT1_3 * (2.0f*foc.i_b - foc.i_a);
//
//	// Park -> direct/quadrature
//	float sin_Ang = _sin(foc.e_theta);
//	float cos_Ang = _cos(foc.e_theta);
//	foc.i_d = cos_Ang*foc.i_alph + sin_Ang*foc.i_beta;
//	foc.i_q = cos_Ang*foc.i_beta - sin_Ang*foc.i_alph;
//
//	/* Regulate currents */
//	foc.DC_I = 0.5f;				// Current duty cycle
//
//	/* Set PWM Compare values */
//	foc.alpha = fmodf(foc.e_theta,60.0f);	// calculate alpha
//
//	foc.DC_1 = foc.DC_I*_sin(60.0f - foc.alpha);
//	foc.DC_2 = foc.DC_I*_sin(foc.alpha);
//	foc.DC_0 = 1.0f - foc.DC_1 - foc.DC_2;
//
//	foc.sector = (int)floor(foc.e_theta/60.0f);
//
//	switch (foc.sector)
//	{
//		case 0:
//			foc.PWM_A = 0.5*foc.DC_0;
//			foc.PWM_B = 0.5*foc.DC_0 + foc.DC_1;
//			foc.PWM_C = 0.5*foc.DC_0 + foc.DC_1 + foc.DC_2;
//			break;
//		case 1:
//			foc.PWM_A = 0.5*foc.DC_0 + foc.DC_2;
//			foc.PWM_B = 0.5*foc.DC_0;
//			foc.PWM_C = 0.5*foc.DC_0 + foc.DC_1 + foc.DC_2;
//			break;
//		case 2:
//			foc.PWM_A = 0.5*foc.DC_0 + foc.DC_1 + foc.DC_2;
//			foc.PWM_B = 0.5*foc.DC_0;
//			foc.PWM_C = 0.5*foc.DC_0 + foc.DC_1;
//			break;
//		case 3:
//			foc.PWM_A = 0.5*foc.DC_0 + foc.DC_1 + foc.DC_2;
//			foc.PWM_B = 0.5*foc.DC_0 + foc.DC_2;
//			foc.PWM_C = 0.5*foc.DC_0;
//			break;
//		case 4:
//			foc.PWM_A = 0.5*foc.DC_0 + foc.DC_1;
//			foc.PWM_B = 0.5*foc.DC_0 + foc.DC_1 + foc.DC_2;
//			foc.PWM_C = 0.5*foc.DC_0;
//			break;
//		case 5:
//			foc.PWM_A = 0.5*foc.DC_0;
//			foc.PWM_B = 0.5*foc.DC_0 + foc.DC_1 + foc.DC_2;
//			foc.PWM_C = 0.5*foc.DC_0 + foc.DC_2;
//			break;
//	}

	/* Set PWM */
//	Set_PWM3(1.0f-foc.PWM_A, 1.0f-foc.PWM_B, 1.0f-foc.PWM_C);
//	Set_PWM3(1.0f-0.066f, 1.0f-0.500f, 1.0f-0.933f);

	/* LED off */
	HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, 0);
}
void  CAN_Interrupt(void)
{
//	// Get CAN message
//		// 4b  Unused
//		// 12b Position
//		// 16b Velocity
//		// 16b Torque
//	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can.rx_header, can.rx_data);
//
//	// Create CAN response message
//		// 8b  temp
//		// 12b position
//		// 14b velocity
//		// 14b torque
//	uint32_t TxMailbox;
//	uint16_t temp_vel = 0b0011111111111111;
//	uint16_t temp_tor = 0b0011111111111111;
//
//	can.tx_data[0] = (uint8_t) (((float)adc.Temp_Fil*adc.VDDA/4095.0f)-adc.Temp_V_Offset)/adc.Temp_Slope;
//	can.tx_data[1] = (uint8_t) (enc.IIF_Raw>>4);
//	can.tx_data[2] = (uint8_t) ((enc.IIF_Raw<<4) | ((temp_vel>>10) & 0b00001111));
//	can.tx_data[3] = (uint8_t) (temp_vel>>2);
//	can.tx_data[4] = (uint8_t) ((temp_vel<<6) | ((temp_tor>>8) & 0b00001111));
//	can.tx_data[5] = (uint8_t) (temp_tor);
//
//	// Send CAN message
//	HAL_CAN_AddTxMessage(&hcan1, &can.tx_header, can.tx_data, &TxMailbox);
//
//	// if special commands, do function else unpack rx message
//	if((can.rx_data[0]==0xFF) & (can.rx_data[1]==0xFF) & (can.rx_data[2]==0xFF) & (can.rx_data[3]==0xFF) & (can.rx_data[4]==0xFF))
//	{
//		switch (can.rx_data[5])
//		{
//		case 0:
//			break;
//		case 1:
//			break;
//		}
//	}
//	else
//	{
//		// unpack and update target values
//	}
//
//	can.timeout = 0;	// reset timeout timer
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  printf("Hardware Error\n");
	  HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, 1);
	  HAL_Delay(200);
	  HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, 0);
	  HAL_Delay(800);
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

