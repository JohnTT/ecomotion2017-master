/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * Copyright (c) 2017 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "fatfs.h"

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

SD_HandleTypeDef hsd;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
AllCell_Bat_State BMS_Bat_State;
AllCell_Bat_Info BMS_Bat_Info;
AllCell_Bat_Current BMS_Bat_Current;
AllCell_Bat_Voltage BMS_Bat_Voltage;
AllCell_Bat_Temperature BMS_Bat_Temperature;
AllCell_Bat_Status BMS_Bat_Status;
AllCell_Bat_PwAvailable BMS_Bat_PwAvailable;
AllCell_Bat_RTC BMS_Bat_RTC;

AllCell_Bat_DataDoubles bmsDataExact;


static float DIAMETER = 0.50; //50 cm diameter
static float PI = 3.1415926535; //the number pi
static int CLOCKSPEED = 10000; //timer's clock
static int NUM_MAGNET = 2;
static int PERIOD_TIMER = 60000;
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
static void MX_SDIO_SD_Init(void);

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

	/* USER CODE END 2 */
#ifdef _DEBUG_ON
	MX_GPIO_Init();
//	MX_CAN1_Init();
	MX_USART2_UART_Init();
//	MX_CAN2_Init();
	MX_TIM1_Init();
	//MX_I2C1_Init();
	//	MX_SDIO_SD_Init();
	//	MX_FATFS_Init();//NEVER INIT THIS!!! IT NO WORK!!!!
#else
	MX_GPIO_Init();
	MX_CAN1_Init();
	MX_USART2_UART_Init();
	MX_CAN2_Init();
	MX_TIM1_Init();
	MX_I2C1_Init();
#endif

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1); //look into peripheral control functions to find out more about configuration
	HAL_TIM_Base_Start_IT(&htim1); //start the base for update interrupts
//	HAL_Delay(1000);
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		printf("MAIN LOOP MASTER\n\r");
		printf("Counter = %d\n\r", counter);
#ifdef _REBROADCAST_ALLCELL
		// Important Information
		HAL_StatusTypeDef status;
		masterCAN1_BMS masterCAN1;
		hcan1.pTxMsg->IDE = CAN_ID_STD;
		hcan1.pTxMsg->RTR = CAN_RTR_DATA;
		hcan1.pTxMsg->StdId = ecoMotion_MasterBMS;
		hcan1.pTxMsg->DLC = 8;
		masterCAN1.current = BMS_Bat_Info.Current;
		masterCAN1.voltage = BMS_Bat_Info.Voltage;
		masterCAN1.temperature = BMS_Bat_Info.Temp;
		masterCAN1.bat_percentage = BMS_Bat_Status.SOC;

		memcpy(hcan1.pTxMsg->Data, &masterCAN1, sizeof(masterCAN1));
		status = HAL_CAN_Transmit_IT(&hcan1);
		if (status != HAL_OK) {
			Error_Handler();
		}
#endif

		//printUART2();
		HAL_Delay(1000);

	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

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
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 72;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 3;
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

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDIO|RCC_PERIPHCLK_CLK48;
	PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
	PeriphClkInitStruct.SdioClockSelection = RCC_SDIOCLKSOURCE_CLK48;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

/* SDIO init function */
static void MX_SDIO_SD_Init(void)
{

	hsd.Instance = SDIO;
	hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
	hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
	hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
	hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
	hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
	hsd.Init.ClockDiv = 0;

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
/* USER CODE BEGIN 4 */
void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan) {
#ifdef _CAN_PRINTF
	printf("Message Sent Successfully on CAN%u\n\r", (hcan->Instance != CAN1)+1);
#endif
}
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan) {

#ifdef _CAN_PRINTF
	printf("CAN Message Received from CAN Interface CAN %u\n\r", (hcan->Instance != CAN1) + 1);
#endif

	// CAN1 @ 250Kbps -> BMS
	if (hcan->Instance == CAN1) {
		HAL_GPIO_WritePin(LEDx_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
	}

	if (hcan->Instance == CAN2) {
		HAL_GPIO_WritePin(LEDx_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		parseBMSCAN(hcan->pRxMsg);
	}

	for (int i = 0; i < 1000; i++) {}
	HAL_GPIO_WritePin(LEDx_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LEDx_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

	if (HAL_CAN_Receive_IT(hcan, hcan->pRxMsg->FIFONumber) != HAL_OK) {
		Error_Handler();
	}
}

void parseBMSCAN(CanRxMsgTypeDef *BMSRxMsg) {
	static const float _Current_Factor = 0.05;
	static const float _Voltage_Factor = 0.05;
	static const float _Impedance_Factor = 0.01;
	static const float _CellVolt_Factor = 0.01;
	static const float _Day_Factor = 0.25;
	static const float _Second_Factor = 0.25;
	static const float _SOC_Factor = 0.5;
	static const float _Capacity_Factor = 0.01;


	static const uint16_t _Current_Offset = 1600;
	static const uint16_t _Temp_Offset = 40;
	static const uint32_t _PwAvailable_Offset = 2000000000;
	static const float _Year_Offset = 1985;


	switch (BMSRxMsg->ExtId) {

	case AllCell_Bat_State_ID:
		memcpy(&BMS_Bat_State, BMSRxMsg->Data, sizeof(BMS_Bat_State));
		break;

	case AllCell_Bat_Info_ID:
		memcpy(&BMS_Bat_Info, BMSRxMsg->Data, sizeof(BMS_Bat_Info));
		bmsDataExact.voltageInfoMsg = BMS_Bat_Info.Voltage * _Voltage_Factor;
		bmsDataExact.currentInfoMsg = BMS_Bat_Info.Current * _Current_Factor - _Current_Offset;
		bmsDataExact.impedance  = BMS_Bat_Info.Impedance * _Impedance_Factor;

		BMS_Bat_Info.Impedance *= _Impedance_Factor;
		break;

	case AllCell_Bat_Current_ID:
		memcpy(&BMS_Bat_Current, BMSRxMsg->Data, sizeof(BMS_Bat_Current));
		bmsDataExact.currentCurMsg = BMS_Bat_Current.Current * _Current_Factor - _Current_Offset;
		bmsDataExact.chargeLim = BMS_Bat_Current.Charge_Limit * _Current_Factor;
		bmsDataExact.dischargeLim = BMS_Bat_Current.Discharge_Limit * _Current_Factor;

		BMS_Bat_Current.Current = BMS_Bat_Current.Current * _Current_Factor - _Current_Offset;
		BMS_Bat_Current.Charge_Limit = BMS_Bat_Current.Charge_Limit * _Current_Factor;
		BMS_Bat_Current.Discharge_Limit = BMS_Bat_Current.Discharge_Limit * _Current_Factor;
		break;

	case AllCell_Bat_Voltage_ID:
		memcpy(&BMS_Bat_Voltage, BMSRxMsg->Data, sizeof(BMS_Bat_Voltage));
		bmsDataExact.voltageVoltMsg = BMS_Bat_Voltage.Voltage * _Voltage_Factor;
		bmsDataExact.mincellVoltage = BMS_Bat_Voltage.Min_Cell_Voltage * _Voltage_Factor;
		bmsDataExact.maxcellVoltage = BMS_Bat_Voltage.Max_Cell_Voltage * _Voltage_Factor;

		BMS_Bat_Voltage.Voltage *= _Voltage_Factor;
		BMS_Bat_Voltage.Min_Cell_Voltage *= _CellVolt_Factor;
		BMS_Bat_Voltage.Max_Cell_Voltage *= _CellVolt_Factor;
		break;

	case AllCell_Bat_Temperature_ID:
		memcpy(&BMS_Bat_Temperature, BMSRxMsg->Data, sizeof(BMS_Bat_Temperature));
		BMS_Bat_Temperature.Temp_BMS -= _Temp_Offset;
		BMS_Bat_Temperature.Avg_Cell_Temp -= _Temp_Offset;
		BMS_Bat_Temperature.Min_Cell_Temp -= _Temp_Offset;
		BMS_Bat_Temperature.Max_Cell_Temp -= _Temp_Offset;
		break;

	case AllCell_Bat_Status_ID:
		memcpy(&BMS_Bat_Status, BMSRxMsg->Data, sizeof(BMS_Bat_Status));
		bmsDataExact.percentCharged = BMS_Bat_Status.SOC * _SOC_Factor;
		bmsDataExact.currentCapacity = BMS_Bat_Status.Capacity * _Capacity_Factor;

		BMS_Bat_Status.Capacity *= _Capacity_Factor;
		break;

	case AllCell_Bat_PwAvailable_ID:
		memcpy(&BMS_Bat_PwAvailable, BMSRxMsg->Data, sizeof(BMS_Bat_PwAvailable));
		BMS_Bat_PwAvailable.PwAvailable_Charge -= _PwAvailable_Offset;
		BMS_Bat_PwAvailable.PwAvailable_Discharge -= _PwAvailable_Offset;
		break;

	case AllCell_Bat_RTC_ID:
		memcpy(&BMS_Bat_RTC, BMSRxMsg->Data, sizeof(BMS_Bat_RTC));

#ifdef _REBROADCAST_ALLCELL
		// Important Information
		HAL_StatusTypeDef status;
		hcan1.pTxMsg->IDE = CAN_ID_STD;
		hcan1.pTxMsg->RTR = CAN_RTR_DATA;
		hcan1.pTxMsg->StdId = ecoMotion_MasterRTC;
		hcan1.pTxMsg->DLC = 8;

		memcpy(hcan1.pTxMsg->Data, &BMS_Bat_RTC, sizeof(BMS_Bat_RTC));
		status = HAL_CAN_Transmit_IT(&hcan1);
		if (status != HAL_OK) {
			Error_Handler();
		}
#endif

		BMS_Bat_RTC.Day *= _Day_Factor;
		BMS_Bat_RTC.Second *= _Second_Factor;
		break;

	default:
		break;

	}
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	counter = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1); //read TIM1 counter value
	if (htim->Instance == TIM1){
		if (tim1Ch1Capture != 70000){//initial capture, should be better initialized
			if (tim1Ch1Overflow)
				tim1Ch1Compare = PERIOD_TIMER - tim1Ch1Capture + counter; //flip around
			else
				tim1Ch1Compare = counter - tim1Ch1Capture; //going up
		}
		tim1Ch1Overflow = 0; //reset the overflow bit, since we capture-compared
		tim1Ch1Capture = counter;  //read TIM1 channel 1 capture value for the next Compare
		if (tim1Ch1Compare <= 1000){ //MUST FIX THIS
		}
		else {
			printf("Captured Tim1 Value:");
			printf(itoa(tim1Ch1Compare, str, 10));
			printf("\n\r");
			speedCalc(CLOCKSPEED, DIAMETER, tim1Ch1Compare, &rpmChan1, &speedChan1);
		}
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
	return HAL_RCC_GetPCLK2Freq() / 5000; //since it is multiplied by 2
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
	*rpmVal = (clockSpeed*1.0) / (compareVal* NUM_MAGNET * 1.0) * 60; //revs per min
	*speedVal = (*rpmVal * PI * wheelDiameter) / 60; //speedChan1, in m/s
	*speedVal = (*speedVal * 3.6); //speedChan1, in km/h

#ifdef _DEBUG_ON
	printf("Revs per min: ");
	printf(itoa(*rpmVal, str, 10));
	printf("\n\r");
	printf("Real Speed: ");
	printf(itoa(*speedVal, str, 10));
	printf("\n\r");
#endif
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

#ifdef _DEBUG_ON
void __io_putchar(uint8_t ch) {
	HAL_UART_Transmit(&huart2, &ch, 1, 1);
}
#endif

void printUART2() {
	// Bat State - Need to be started
	printf("STATE MESSAGE ---------------\n\r");
	printf("Number of Power Cycles: %u\n\r", BMS_Bat_State.Reset);

	// Bat Info Message
	printf("INFO MESSAGE ---------------\n\r");
	printf("Current: %u Amps\n\r", BMS_Bat_Info.Current);
	printf("Voltage: %u Volts\n\r", BMS_Bat_Info.Voltage);
	printf("Temperature: %u deg Celsius\n\r", BMS_Bat_Info.Temp);
	printf("Impedance: %u mOhm\n\r", BMS_Bat_Info.Impedance);

	// Bat Current Message
	printf("CURRENT MESSAGE ---------------\n\r");
	printf("Current: %u Amps\n\r", BMS_Bat_Current.Current);
	printf("Charge Limit: %u Amps\n\r", BMS_Bat_Current.Charge_Limit);
	printf("Disharge Limit: %u Amps\n\r", BMS_Bat_Current.Discharge_Limit);

	// Voltage Message
	printf("VOLTAGE MESSAGE ---------------\n\r");
	printf("Voltage: %u Volts\n\r", BMS_Bat_Voltage.Voltage);
	printf("Min Cell Voltage: %u Volts\n\r", BMS_Bat_Voltage.Min_Cell_Voltage);
	printf("Min Cell Number: %u\n\r", BMS_Bat_Voltage.Nb_Min_Voltage);
	printf("Max Cell Voltage: %u Volts\n\r", BMS_Bat_Voltage.Max_Cell_Voltage);
	printf("Max Cell Number: %u\n\r", BMS_Bat_Voltage.Nb_Max_Voltage);

	// Temp Message
	printf("TEMP MESSAGE ---------------\n\r");
	printf("BMS Temperature: %u deg Celsius\n\r", BMS_Bat_Temperature.Temp_BMS);
	printf("Min Cell Temp: %u deg Celsius\n\r", BMS_Bat_Temperature.Min_Cell_Temp);
	printf("Min Cell Number: %u\n\r", BMS_Bat_Temperature.Nb_Min_Temp);
	printf("Max Cell Temp: %u deg Celsius\n\r", BMS_Bat_Temperature.Max_Cell_Temp);
	printf("Max Cell Number: %u\n\r", BMS_Bat_Temperature.Nb_Max_Temp);

	// Status Message
	printf("STATUS MESSAGE ---------------\n\r");
	printf("State of Charge: %u %%\n\r", BMS_Bat_Status.SOC);
	printf("Current Capacity: %u Ahr\n\r", BMS_Bat_Status.Capacity);

	// Power Available Message
	printf("POWER AVAILABLE MESSAGE ---------------\n\r");
	printf("PwAvailableCharge: %lu Watts\n\r", BMS_Bat_PwAvailable.PwAvailable_Charge);
	printf("PwAvailableDisharge: %lu Watts\n\r", BMS_Bat_PwAvailable.PwAvailable_Discharge);


	// Real Time Clock Message
	printf("RTC MESSAGE ---------------\n\r");
	printf("Year: %u\n\r", BMS_Bat_RTC.Year+1985);
	printf("Month: %u\n\r", BMS_Bat_RTC.Month);
	printf("Day: %u\n\r", BMS_Bat_RTC.Day);
	printf("Hour: %u\n\r", BMS_Bat_RTC.Hour);
	printf("Minute: %u\n\r", BMS_Bat_RTC.Minute);
	printf("Second: %u\n\r", BMS_Bat_RTC.Second);

	// Exact BMS Data as Doubles
	printf("BMS doubles ---------------\n\r");
	printf("Current <InfoMsg>: %f [Amps]\n\r", bmsDataExact.currentInfoMsg);
	printf("Voltage <InfoMsg>: %f [Volts]\n\r", bmsDataExact.voltageInfoMsg);
	printf("Impedance <InfoMsg>: %f [mOhms]\n\r", bmsDataExact.impedance);
	printf("Current <CurrentMsg>: %f [Amps]\n\r", bmsDataExact.currentCurMsg);
	printf("Charge Limit <CurrentMsg>: %f [Amps]\n\r", bmsDataExact.chargeLim);
	printf("Discharge <CurrentMsg>: %f [Amps]\n\r", bmsDataExact.dischargeLim);
	printf("Voltage <VoltageMsg>: %f [Volts]\n\r", bmsDataExact.voltageVoltMsg);
	printf("Min Cell Voltage <VoltageMsg>: %f [Volts]\n\r", bmsDataExact.mincellVoltage);
	printf("Max Cell Voltage <VoltageMsg>: %f [Volts]\n\r", bmsDataExact.maxcellVoltage);
	printf("Percent Charged <StateMsg>: %f [%%]\n\r", bmsDataExact.percentCharged);
	printf("Current Capacity <StateMsg>: %f [Ahr]\n\r", bmsDataExact.currentCapacity);

	printf("\n\r");

}

static void MX_CAN1_Init(void)
{
	__HAL_RCC_CAN1_CLK_ENABLE();
	hcan1.Instance = CAN1;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	setCANbitRate(500, 36, &hcan1);
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
	canFilterConfig.BankNumber = 14;
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

	canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canFilterConfig.FilterIdHigh = 0x0000;
	canFilterConfig.FilterIdLow = 0x0000;
	canFilterConfig.FilterMaskIdHigh = 0x0000;
	canFilterConfig.FilterMaskIdLow = 0x0000;
	canFilterConfig.FilterFIFOAssignment = CAN_FIFO1;
	canFilterConfig.FilterActivation = ENABLE;
	canFilterConfig.BankNumber = 14;
	canFilterConfig.FilterNumber = 14;
	if(HAL_CAN_ConfigFilter(&hcan2, &canFilterConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_CAN_Receive_IT(&hcan2, CAN_FIFO1) != HAL_OK) {
		Error_Handler();
	}
}
static void MX_TIM1_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_IC_InitTypeDef sConfigIC;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = getTim1Prescaler();
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = PERIOD_TIMER;
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
		hcan1.pTxMsg->IDE = CAN_ID_EXT;
		hcan1.pTxMsg->RTR = CAN_RTR_DATA;
		hcan1.pTxMsg->ExtId = ecoMotion_Error_Master;
		hcan1.pTxMsg->DLC = 0;

		printf("Error Handler - Master - CAN1 ID: %x\n\r", hcan1.pTxMsg->ExtId);

#ifdef _ERRORHANDLER_CAN1TRANSMIT
		status = HAL_CAN_Transmit_IT(&hcan1);
#endif

		HAL_GPIO_WritePin(LEDx_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
		for (int i = 0; i < 1000; i++) {}
		HAL_GPIO_WritePin(LEDx_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);


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
