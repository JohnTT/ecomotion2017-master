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

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN2_Init(void);

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
  //MX_CAN1_Init();
  MX_USART2_UART_Init();
  //MX_CAN2_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
	  printf("MAIN LOOP");
	  printf("\n\r");

	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  HAL_Delay(100);

  /* USER CODE BEGIN 3 */

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

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

/* CAN1 init function */
//static void MX_CAN1_Init(void)
//{
//
//  hcan1.Instance = CAN1;
//  hcan1.Init.Prescaler = 3;
//  hcan1.Init.Mode = CAN_MODE_NORMAL;
//  hcan1.Init.SJW = CAN_SJW_1TQ;
//  hcan1.Init.BS1 = CAN_BS1_12TQ;
//  hcan1.Init.BS2 = CAN_BS2_2TQ;
//  hcan1.Init.TTCM = DISABLE;
//  hcan1.Init.ABOM = DISABLE;
//  hcan1.Init.AWUM = DISABLE;
//  hcan1.Init.NART = DISABLE;
//  hcan1.Init.RFLM = DISABLE;
//  hcan1.Init.TXFP = DISABLE;
//  if (HAL_CAN_Init(&hcan1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//}
//
///* CAN2 init function */
//static void MX_CAN2_Init(void)
//{
//
//  hcan2.Instance = CAN2;
//  hcan2.Init.Prescaler = 16;
//  hcan2.Init.Mode = CAN_MODE_NORMAL;
//  hcan2.Init.SJW = CAN_SJW_1TQ;
//  hcan2.Init.BS1 = CAN_BS1_1TQ;
//  hcan2.Init.BS2 = CAN_BS2_1TQ;
//  hcan2.Init.TTCM = DISABLE;
//  hcan2.Init.ABOM = DISABLE;
//  hcan2.Init.AWUM = DISABLE;
//  hcan2.Init.NART = DISABLE;
//  hcan2.Init.RFLM = DISABLE;
//  hcan2.Init.TXFP = DISABLE;
//  if (HAL_CAN_Init(&hcan2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//}

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

}

/* USER CODE BEGIN 4 */
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

// Needed for printf
void __io_putchar(uint8_t ch) {
	HAL_UART_Transmit(&huart2, &ch, 1, 1);
}
static void MX_CAN1_Init(void)
{
	__HAL_RCC_CAN1_CLK_ENABLE();
	hcan1.Instance = CAN1;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	setCANbitRate(1000, 32, &hcan1);
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
	canFilterConfig.FilterFIFOAssignment = 0;
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
	__HAL_RCC_CAN1_CLK_ENABLE();
	hcan2.Instance = CAN2;
	hcan2.Init.Mode = CAN_MODE_NORMAL;
	setCANbitRate(1000, 32, &hcan2);
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
	canFilterConfig.FilterNumber = 0;
	canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canFilterConfig.FilterIdHigh = 0x0000;
	canFilterConfig.FilterIdLow = 0x0000;
	canFilterConfig.FilterMaskIdHigh = 0x0000;
	canFilterConfig.FilterMaskIdLow = 0x0000;
	canFilterConfig.FilterFIFOAssignment = 0;
	canFilterConfig.FilterActivation = ENABLE;
	canFilterConfig.BankNumber = 14;
	if(HAL_CAN_ConfigFilter(&hcan2, &canFilterConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0) != HAL_OK) {
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
  while(1) 
  {
  }
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