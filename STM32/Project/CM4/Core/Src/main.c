/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <string.h>

#include <Animaciones.h>
#include "ssd1306.c"
#include "ssd1306_fonts.c"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

//ENCODER
#define AS5600_ADDRESS 0x36 << 1
#define STATUS 0x0B
#define ANGLE_H 0x0E
#define ANGLE_L 0x0F
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c4;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
//ENCODER
uint8_t angH;
uint8_t angL;
uint16_t angle;
uint8_t check = 255;
uint16_t angleDegrees;

//UART

uint8_t bufferMAT[2]; //PARA MATLAB
char buffer[50];
uint16_t len = 2;

//ADC
uint8_t medicion[1];

//CONTROL
uint16_t referencia;
int16_t errorActual;
int16_t errorAnterior;
float Gn;
float Ts = 0.025;

int16_t delta;  //PARA CALCULAR CONTROL

// PID
float Kp = 3.1549;
float Ki = 12.7301;
float Kd = 0.037447;

//PI
/*
float Kp = 1.0825;
float Ki = 2.3068;*/

uint8_t CicloUtil;

//ANTIRREBOTE
uint32_t last_int_time = 0;
uint32_t current_time;
uint8_t debounce = 50;

//CAN
FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_TxHeaderTypeDef TxHeader2;
FDCAN_FilterTypeDef sFilterConfig;

uint8_t TxData[8] = {1,2,3,4,5,6,7,8};
uint8_t TxData2[8] = {8, 7, 6, 5, 4, 3, 2, 1};
uint8_t RxData[8];

//FFT
uint8_t tiempoDeError;

struct shared_data{
	uint8_t alerta;

};

volatile struct shared_data * const xfr_ptr = (struct shared_data *)0x38001000;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_I2C4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM7_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /* Activate HSEM notification for Cortex-M4*/
  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
  /*
  Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
  perform system initialization (system clock config, external memory configuration.. )
  */
  HAL_PWREx_ClearPendingEvent();
  HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
  /* Clear HSEM flag */
  __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C4_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  MX_FDCAN1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim7);
  AS5600_init();
  AS5600_read_angle();
  referencia = (uint16_t)(angle* 360.0f/4096.0f);
  errorAnterior = referencia;


  ssd1306_Init();
  updateOLED();
  HAL_NVIC_SetPriority(HSEM2_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(HSEM2_IRQn);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


  }
  /* USER CODE END 3 */
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = ENABLE;
  hfdcan1.Init.ProtocolException = ENABLE;
  hfdcan1.Init.NominalPrescaler = 5;
  hfdcan1.Init.NominalSyncJumpWidth = 8;
  hfdcan1.Init.NominalTimeSeg1 = 0x1F;
  hfdcan1.Init.NominalTimeSeg2 = 8;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 1;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 1;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x321;
    sFilterConfig.FilterID2 = 0x7FF;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK){
        /* Filter configuration Error */
  	  Error_Handler();
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK){
        /* Filter configuration Error */
  	  Error_Handler();
    }


    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
        /* Filter configuration Error */
  	  Error_Handler();

    }


    TxHeader.Identifier = 0xFF;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    TxHeader2.Identifier = 0xAA;
    TxHeader2.IdType = FDCAN_STANDARD_ID;
    TxHeader2.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader2.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader2.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader2.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader2.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader2.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader2.MessageMarker = 0;
  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00909FCE;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x00909FCE;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 74;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 7499;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 250;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DOWN_Pin */
  GPIO_InitStruct.Pin = DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DOWN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : UP_Pin */
  GPIO_InitStruct.Pin = UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//INTERRUPCIÓN DE TIMERS
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim){
	if(htim == &htim7){
		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
		AS5600_read_angle();
		angleDegrees = (uint16_t)(angle* 360.0f/4096.0f);

		/*referencia = (medicion[0]*72)/255;

		if(referencia < 5){
			referencia = 5;
		}*/


		if((angleDegrees >  300) && (angleDegrees < 360)){
			angleDegrees = 0;
		}

		/*
		if((angleDegrees >  72) && (angleDegrees < 90)){
			angleDegrees = 72;
		}*/


		errorActual = referencia - angleDegrees;
		errorActual = (errorActual*20)/72;

		Gn = (Kp*errorActual)+((Kd/Ts)*(errorActual-errorAnterior))+((Ki*Ts)*(errorActual+errorAnterior));

		//Gn = (Kp*errorActual)+((Ki*Ts)*(errorActual+errorAnterior));

		if(errorActual < 0){
			  uint8_t valPWM = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
			  CicloUtil = (uint8_t)(((Gn/(-72))*20)+valPWM);
		        if(CicloUtil > 20){
		        	CicloUtil = 20;
		        }
			  TIM3->CCR2 = 0;
			  TIM3->CCR1 = CicloUtil;
		}else{
			uint8_t valPWM2 = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2);
			CicloUtil = (uint8_t)(((Gn/(72))*20)+valPWM2);
	        if(CicloUtil > 20){
	        	CicloUtil = 20;
	        }
			TIM3->CCR2 = CicloUtil;
			TIM3->CCR1 = 0;
		}

		//CicloUtil = (uint8_t)(((Gn/186.0)*100)+valPWM);
		//len = sprintf(buffer, "%.1f\n\r", referencia);
		//len = sprintf(buffer, "holaaa");
		//buffer[0] = referencia;
		//buffer[1] = angleDegrees;
		//len = sprintf(buffer, "%u \t %u\n\r", referencia, angleDegrees);
		//HAL_UART_Transmit(&huart3, (uint8_t *) buffer, len, 100);

		//bufferMAT[0] = (uint8_t)referencia;
		//bufferMAT[1] = (uint8_t)angleDegrees;

		//len = sprintf(buffer, "%u \t %u \n\r", referencia, angleDegrees);
		errorAnterior = errorActual;

		//HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

		//HAL_UART_Transmit(&huart3, (uint8_t *) bufferMAT, len, 100);
	}
}


//FUNCIONES DE ENCODER
void AS5600_init(void){
	uint8_t statusReg = 0;
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Read(&hi2c4, AS5600_ADDRESS, STATUS, 1, &statusReg, 1, 100);
	HAL_Delay(50);

	//Faltaría lo de checar el status
}

void AS5600_read_angle(void){
	uint8_t data[2] = {0};
	HAL_I2C_Mem_Read(&hi2c4, AS5600_ADDRESS, ANGLE_H, 1, data, 2, 100);
	angle = ((uint16_t)data[0]<<8) | data[1];

}


//INTERRUPCIONES EXTERNAS
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	current_time = HAL_GetTick();
	if(GPIO_Pin == UP_Pin){
		if((current_time - last_int_time)>debounce){
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			if(referencia >= 90){
				referencia = 95;
			}else{
				referencia += 4;
			}
		}

	}else if(GPIO_Pin == DOWN_Pin){
		if((current_time - last_int_time)>debounce){
			HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
			if(referencia <= 35){
				referencia = 30;
			}else{
				referencia -= 4;
			}
		}

	}

	last_int_time = current_time;
	updateOLED();
}

//INTERRUPCIÓN DE CAN
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){

	if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK){
	    Error_Handler();
	}
	else{
		// HASTA ARRIBA
		 if (RxHeader.Identifier == 0x43) {
			  //HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
			  referencia = 90;
			  updateOLED();


		  }
		 // ENMEDIO
		  if (RxHeader.Identifier == 0x11) {
			  referencia = 60;
			  updateOLED();
			  //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

		  }
		  // HASTA ABAJO
		  if (RxHeader.Identifier == 0x65) {
			  referencia = 30;
			  updateOLED();
			  //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

		  }
		  // REQUEST -> enviar posicion
		  if (RxHeader.Identifier == 0x50) {
			  TxData2[0] = (uint8_t)angleDegrees;
			  //HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);

			  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader2, TxData2) != HAL_OK)
				  {
					  /* Transmission request Error */
					  //HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
					  Error_Handler();
				  }
		  }
	}
}

void updateOLED(){
	if((referencia >= 30)&&(referencia < 50)){
		ssd1306_Fill(Black);
		ssd1306_DrawBitmap(0, 0, pos2, 128, 64, White);
		ssd1306_UpdateScreen();
	}else if((referencia >= 50)&&(referencia < 70)){
		ssd1306_Fill(Black);
		ssd1306_DrawBitmap(0, 0, pos1, 128, 64, White);
		ssd1306_UpdateScreen();
	}else{
		ssd1306_Fill(Black);
		ssd1306_DrawBitmap(0, 0, pos0, 128, 64, White);
		ssd1306_UpdateScreen();
	}
}

void HAL_HSEM_FreeCallback(uint32_t SemMask){
	// EVALUACIÓN DE ALERTA (MANDAR CAN)
	if(xfr_ptr->alerta == 1){
	  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
		  {
			  Error_Handler();
		  }
	}
	tiempoDeError++;
	if((tiempoDeError > 5)&&xfr_ptr->alerta == 1){
	   tiempoDeError=0;
	   if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
		  {
			  Error_Handler();
		  }
	   xfr_ptr->alerta = 0;

	}
	HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));


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
