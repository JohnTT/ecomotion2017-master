/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2017 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "master.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
AllCell_Bat_Current BMS_Bat_State;
AllCell_Bat_Current BMS_Bat_Info;
AllCell_Bat_Current BMS_Bat_Current;
AllCell_Bat_Current BMS_Bat_Voltage;
AllCell_Bat_Current BMS_Bat_Temperature;
AllCell_Bat_Current BMS_Bat_Status;
AllCell_Bat_Current BMS_Bat_PwAvailable;
AllCell_Bat_Current BMS_Bat_RTC;


static float DIAMETER = 0.50; //50 cm diameter
static float PI = 3.1415926535; //the number pi
static int CLOCKSPEED = 20000; //timer's clock
float rpmChan1; //revolutions per minute for one of the wheels of the vehicle
float speedChan1; //holds the car's speed from tim1 channel 1
//float rpmChan2; //revolutions per minute for the other wheel of the vehicle
//float speedChan2; //holds the car's speed from tim1 channel 2
int ADCScalingFactor; //should be constant
double NORMALFACTOR; //should be constant
unsigned int counter; //holds tim1 clock
unsigned int analog; //holds the analog value for hadc1
unsigned int tim1Ch1Capture = 70000; //holds the last value from tim1 channel 1
unsigned int tim1Ch1Compare; //holds the compared value from tim1 channel 1
unsigned int tim1Ch1Overflow; //holds the overflow bit for channel 1 when the tim1 clock resets to 0
char str[10]; //holds string values
int blinky = 0; //does stuff
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN2_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_CAN1_Init();
	MX_CAN2_Init();
	MX_USART2_UART_Init();
	MX_TIM1_Init();
	MX_I2C1_Init();

	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	HAL_Delay(1000);
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		//printf("MAIN LOOP\n\r");
		HAL_GPIO_WritePin(LEDx_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LEDx_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		HAL_Delay(100);

	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 144;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


/* I2C1 init function */
static void MX_I2C1_Init(void)
{

	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 38400;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LED0_Pin | LED1_Pin | LED2_Pin | LED3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LEDx_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan) {
	printf("Message Sent Successfully:");
	printf("\n\r");
}
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan) {
	printf("CAN Message Received from CAN Interface CAN");
	printf(itoa((hcan->Instance != CAN1) + 1, str, 10));
	printf("\n\r");

	if (hcan->Instance == CAN1) {
		HAL_GPIO_WritePin(LEDx_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
	}
	if (hcan->Instance == CAN2) {
		HAL_GPIO_WritePin(LEDx_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	}
	// CAN_FIFO1 = BMS CAN Bus @ 125Kbps
//	if (hcan->pRxMsg->FIFONumber == CAN_FIFO1) {
//		printf("CAN 1 Message Received by:");
//		printf(itoa(hcan->pRxMsg->ExtId, str, 16));
//		printf("\n\r");
//		switch (hcan->pRxMsg->ExtId) {
//
//		case AllCell_Bat_State_ID:
//			memcpy(&BMS_Bat_State, hcan->pRxMsg->Data, sizeof(BMS_Bat_State));
//			break;
//
//		case AllCell_Bat_Info_ID:
//			memcpy(&BMS_Bat_Info, hcan->pRxMsg->Data, sizeof(BMS_Bat_Info));
//			break;
//
//		case AllCell_Bat_Current_ID:
//			memcpy(&BMS_Bat_Current, hcan->pRxMsg->Data, sizeof(BMS_Bat_Current));
//			break;
//
//		case AllCell_Bat_Voltage_ID:
//			memcpy(&BMS_Bat_Voltage, hcan->pRxMsg->Data, sizeof(BMS_Bat_Voltage));
//
//			break;
//
//		case AllCell_Bat_Temperature_ID:
//			memcpy(&BMS_Bat_Temperature, hcan->pRxMsg->Data, sizeof(BMS_Bat_Temperature));
//			break;
//
//		case AllCell_Bat_Status_ID:
//			memcpy(&BMS_Bat_Status, hcan->pRxMsg->Data, sizeof(BMS_Bat_Status));
//			break;
//
//		case AllCell_Bat_PwAvailable_ID:
//			memcpy(&BMS_Bat_PwAvailable, hcan->pRxMsg->Data, sizeof(BMS_Bat_PwAvailable));
//			break;
//
//		case AllCell_Bat_RTC_ID:
//			memcpy(&BMS_Bat_RTC, hcan->pRxMsg->Data, sizeof(BMS_Bat_RTC));
//			break;
//
//		default:
//			break;
//
//		}
//
//	}
//
//	// CAN_FIFO0 = Main Vehicle CAN Bus @ 500Kbps
//	else if (hcan->pRxMsg->FIFONumber == CAN_FIFO0){
//		printf("CAN 0 Message Received by:");
//		printf(itoa(hcan->pRxMsg->StdId, str, 10));
//		printf("\n\r");
//
//
//		if (hcan->pRxMsg->StdId == 0x124) {
//			//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, hcan->pRxMsg->Data[0]);
//			//printf("Good stuff");
//		}
//	}
	if (HAL_CAN_Receive_IT(hcan, hcan->pRxMsg->FIFONumber) != HAL_OK) {
		Error_Handler();
	}
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	counter = __HAL_TIM_GetCounter(&htim1); //read TIM1 counter value
	if (htim->Instance == TIM1){
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
			if (tim1Ch1Capture != 70000){//initial capture, should be better initialized
				if (tim1Ch1Overflow)
					tim1Ch1Compare = 65535 - tim1Ch1Capture + counter; //flip around
				else
					tim1Ch1Compare = counter - tim1Ch1Capture; //going up
			}
			tim1Ch1Overflow = 0; //reset the overflow bit, since we capture-compared
			tim1Ch1Capture = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1); //read TIM1 channel 1 capture value for the next Compare
		}
		printf("Captured Tim1 Value:");
		printf(itoa(tim1Ch1Compare, str, 10));
		printf("\n\r");
		speedCalc(CLOCKSPEED, DIAMETER, tim1Ch1Compare, &rpmChan1, &speedChan1);
		//__HAL_TIM_SetCounter(htim, 0); //resets the counter after input capture interrupt occurs
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){//for counter update event (wrap back to 0)
	//put overflow bit stuff here.
	//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	printf("we elapsed");
	if(tim1Ch1Overflow && tim1Ch1Capture != 70000){
		Error_Handler();//error handler stuff, nothing for now
	}
	else {
		tim1Ch1Overflow = 1;
	}

}
int getTim1Prescaler(){
	return HAL_RCC_GetPCLK2Freq() / 20000; //since it is multiplied by 2
}
char *itoa (int value, char *result, int base)
{
	// check that the base if valid
	if (base < 2 || base > 36) {
		*result = '\0';
		return result;
	}

	char* ptr = result, *ptr1 = result, tmp_char;
	int tmp_value;

	do {
		tmp_value = value;
		value /= base;
		*ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
	} while ( value );

	// Apply negative sign
	if (tmp_value < 0) *ptr++ = '-';
	*ptr-- = '\0';
	while (ptr1 < ptr) {
		tmp_char = *ptr;
		*ptr--= *ptr1;
		*ptr1++ = tmp_char;
	}
	return result;
}
void speedCalc(int clockSpeed, float wheelDiameter, int compareVal, float* rpmVal, float* speedVal){
	*rpmVal = (clockSpeed*1.0) / (compareVal* 2.0) * 60; //revs per min
	*speedVal = (*rpmVal * 2 * PI * wheelDiameter) / 60; //speedChan1, in m/s
	*speedVal = (*speedVal * 3.6); //speedChan1, in km/h
	printf("Revs per min: ");
	printf(itoa(*rpmVal, str, 10));
	printf("\n\r");
	printf("Real Speed: ");
	printf(itoa(*speedVal, str, 10));
	printf("\n\r");
}
static void setCANbitRate(uint16_t bitRate, uint16_t periphClock, CAN_HandleTypeDef* theHcan) {
	uint8_t prescaleFactor = 0;
	switch (periphClock) {
	case 32:
		theHcan->Init.BS1 = CAN_BS1_13TQ;
		theHcan->Init.BS2 = CAN_BS2_2TQ;
		prescaleFactor = 2;
		break;
	case 36:
		theHcan->Init.BS1 = CAN_BS1_14TQ;
		theHcan->Init.BS2 = CAN_BS2_3TQ;
		prescaleFactor = 2;
		break;
	case 45:
		theHcan->Init.BS1 = CAN_BS1_12TQ;
		theHcan->Init.BS2 = CAN_BS2_2TQ;
		prescaleFactor = 3;
		break;
	case 48:
		theHcan->Init.BS1 = CAN_BS1_12TQ;
		theHcan->Init.BS2 = CAN_BS2_3TQ;
		prescaleFactor = 3;
		break;
	}
	theHcan->Init.SJW = CAN_SJW_1TQ;
	switch (bitRate) {
	case 1000:
		theHcan->Init.Prescaler = prescaleFactor * 1;
		break;
	case 500:
		theHcan->Init.Prescaler = prescaleFactor * 2;
		break;
	case 250:
		theHcan->Init.Prescaler = prescaleFactor * 4;
		break;
	case 125:
		theHcan->Init.Prescaler = prescaleFactor * 8;
		break;
	case 100:
		theHcan->Init.Prescaler = prescaleFactor * 10;
		break;
	case 83:
		theHcan->Init.Prescaler = prescaleFactor * 12;
		break;
	case 50:
		theHcan->Init.Prescaler = prescaleFactor * 20;
		break;
	case 20:
		theHcan->Init.Prescaler = prescaleFactor * 50;
		break;
	case 10:
		theHcan->Init.Prescaler = prescaleFactor * 100;
		break;
	}
}
void __io_putchar(uint8_t ch) {
	HAL_UART_Transmit(&huart2, &ch, 1, 1);
}
static void MX_CAN1_Init(void)
{
	__HAL_RCC_CAN1_CLK_ENABLE();
	hcan1.Instance = CAN1;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	setCANbitRate(250, 36, &hcan1);
	hcan1.Init.TTCM = DISABLE;
	hcan1.Init.ABOM = DISABLE;
	hcan1.Init.AWUM = DISABLE;
	hcan1.Init.NART = DISABLE;
	hcan1.Init.RFLM = DISABLE;
	hcan1.Init.TXFP = DISABLE;
	static CanTxMsgTypeDef TxMessage;
	static CanRxMsgTypeDef RxMessage;
	hcan1.pTxMsg = &TxMessage;
	hcan1.pRxMsg = &RxMessage;
	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		Error_Handler();
	}
	CAN_FilterConfTypeDef canFilterConfig;
	canFilterConfig.FilterNumber = 0;
	canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canFilterConfig.FilterIdHigh = 0x0000;
	canFilterConfig.FilterIdLow = 0x0000;
	canFilterConfig.FilterMaskIdHigh = 0x0000;
	canFilterConfig.FilterMaskIdLow = 0x0000;
	canFilterConfig.FilterFIFOAssignment = CAN_FIFO0;
	canFilterConfig.FilterActivation = ENABLE;
	canFilterConfig.BankNumber = 0;
	if(HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0) != HAL_OK) {
		Error_Handler();
	}
}
static void MX_CAN2_Init(void)
{
	__HAL_RCC_CAN2_CLK_ENABLE();
	hcan2.Instance = CAN2;
	hcan2.Init.Mode = CAN_MODE_NORMAL;
	setCANbitRate(250, 36, &hcan2);
	hcan2.Init.TTCM = DISABLE;
	hcan2.Init.ABOM = DISABLE;
	hcan2.Init.AWUM = DISABLE;
	hcan2.Init.NART = DISABLE;
	hcan2.Init.RFLM = DISABLE;
	hcan2.Init.TXFP = DISABLE;
	static CanTxMsgTypeDef TxMessage;
	static CanRxMsgTypeDef RxMessage;
	hcan2.pTxMsg = &TxMessage;
	hcan2.pRxMsg = &RxMessage;
	if (HAL_CAN_Init(&hcan2) != HAL_OK)
	{
		Error_Handler();
	}
	CAN_FilterConfTypeDef canFilterConfig;
	canFilterConfig.FilterNumber = 14;
	canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canFilterConfig.FilterIdHigh = 0x0000;
	canFilterConfig.FilterIdLow = 0x0000;
	canFilterConfig.FilterMaskIdHigh = 0x0000;
	canFilterConfig.FilterMaskIdLow = 0x0000;
	canFilterConfig.FilterFIFOAssignment = CAN_FIFO0;
	canFilterConfig.FilterActivation = ENABLE;
	canFilterConfig.BankNumber = 0;
	if(HAL_CAN_ConfigFilter(&hcan2, &canFilterConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0) != HAL_OK) {
		Error_Handler();
	}
}
static void MX_TIM1_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_IC_InitTypeDef sConfigIC;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 65535;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 0;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 15;
	if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler */
	/* User can add his own implementation to report the HAL error return state */
	HAL_StatusTypeDef status;
	do
	{
		printf("Error Handler\n\r");
		//		hcan1.pTxMsg->IDE = CAN_ID_STD;
		//		hcan1.pTxMsg->RTR = CAN_RTR_DATA;
		//		hcan1.pTxMsg->StdId = ecoMotion_Error_Master;
		//		hcan1.pTxMsg->DLC = 0;
		//		status = HAL_CAN_Transmit_IT(&hcan1);
		HAL_Delay(100);
	} while(status != HAL_OK);
	/* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
