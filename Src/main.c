/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include "bt_commands.h"
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint32_t string_size = 0;
uint32_t string_length = 0;
uint8_t cycleBuf_count = 0;
//char ar_commands[15][15];
char strings[32];
//extern char *c;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
BtModuleTypeDef bt;

BtTxMsgTypeDef TxMes;
BtRxMsgTypeDef RxMes;
uint8_t len = 6;
RingBufTypeDef ringBuf_btCommands;
typedef struct 
{
	uint8_t canRx;
	uint8_t btRx;
}FlagTypeDef;
FlagTypeDef flag;
uint8_t muteOnDbg = 0;
uint8_t muteOffDbg = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin== GPIO_PIN_0)
	{
		
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static uint32_t rx_counter = 0;
		//HAL_UART_Receive_IT(&huart2, RxMes.short_resp, len);
		HAL_UART_Receive_IT(&huart2, &bt.var, 1);
	if ( ((bt.var == 'B')||(bt.var == 'S')||(bt.var == 'O')) && bt.prVar!='C' )
	{
		for(uint8_t i = 0; i < 10; i++)
		{ bt.rx_buf[i] = 0; }
	}
	if (bt.var != 0x0A)
	{
		bt.rx_buf[rx_counter++] = bt.var;
	}
	else if (bt.var == 0x0A)
	{
		rx_counter = 0;
		flag.btRx = 1;
	}
	//flag.btRx = 1;
	bt.prVar = bt.var;
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{

}

void RING_Put(BtCommandTypeDef command, RingBufTypeDef* buf)
{
		buf->buffer[buf->idxIn++]= command;  
    if (buf->idxIn >= CYCLE_BUF_SIZE_x) buf->idxIn = 0;
}
BtCommandTypeDef RING_Pop(RingBufTypeDef* buf)
{
	BtCommandTypeDef retval = buf->buffer[buf->idxOut++];
	if (buf->idxOut >= CYCLE_BUF_SIZE_x) buf->idxOut = 0;
	return retval;
}
uint8_t RING_GetCount(RingBufTypeDef* buf)
{
    uint8_t retval = 0;
    if (buf->idxIn < buf->idxOut) retval = CYCLE_BUF_SIZE_x + buf->idxIn - buf->idxOut;
    else retval = buf->idxIn - buf->idxOut;
    return retval;
}
void RING_Clear(RingBufTypeDef* buf)
{
    buf->idxIn = 0;
    buf->idxOut = 0;
}/*
void RING_Init(RingBufTypeDef* buf)
{
    buf->size = size;
    buf->buffer = test_array[][];//(uint8_t*) malloc(size);
    RING_Clear(buf);
}*/
void bt_mute(uint8_t on, uint8_t *Vol)
{
	if (on)
	{
		HAL_UART_Transmit(&huart2, (uint8_t*)BT_VOLUME_QUERY, strlen(BT_VOLUME_QUERY), 100);
		bt.ticks = HAL_GetTick();
		while (!flag.btRx) 
		{
			if (HAL_GetTick() - bt.ticks > 100)
			{
				bt.err = 1;
				break;
			}
		}
		
			flag.btRx = 0;
			Vol[0] = bt.rx_buf[5];
			Vol[1] = bt.rx_buf[6];
		
	}
	else
	{
		uint8_t vol_com[9] = {'C', 'O', 'M', '+', 'V', Vol[0], Vol[1], '\r', '\n'};
		HAL_UART_Transmit(&huart2, vol_com, strlen((const char*)vol_com), 100);
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//ar_commands[15][15];
	//strcpy(&ar_commands[toOn][0], "COM+PWOS\r\n");
	//strcpy(&ar_commands[toOff][0], "COM+PWDS\r\n");
	//strcpy(TxMes.On, "COM+PWOS\r\n");
	//strcpy(TxMes.Off, "COM+PWDS\r\n");
	bt.pBtTxMsg = &TxMes;
	bt.pBtRxMsg = &RxMes;
	//strings[0] = c[0];
	
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	//HAL_UART_Receive_IT(&huart2, RxMes.short_resp, len);
	HAL_UART_Receive_IT(&huart2, &bt.var, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	string_size = sizeof(BT_ON);
	string_length = strlen(&bt_commands[toOn][0]);
	//bt.command = toOn;
	RING_Put(toNop, &ringBuf_btCommands);

	RING_Put(toNop, &ringBuf_btCommands);
	
	RING_Put(toNop, &ringBuf_btCommands);
	
	RING_Put(toNop, &ringBuf_btCommands);

	bt.ticks = HAL_GetTick();
  while (1)
  {
		/*if (bt.command == toOn)
		{
			
			//HAL_UART_Transmit(&huart2, (uint8_t*)BT_ON, strlen(BT_ON), 100);
			HAL_UART_Transmit(&huart2, (uint8_t*)&ar_commands[toOn][0], strlen(&ar_commands[toOn][0]), 100);
			bt.command = toNop;
		}
		else if (bt.command == toOff)
		{
		
			HAL_UART_Transmit(&huart2, (uint8_t*)BT_OFF, strlen(BT_OFF), 100);
			bt.command = toNop;
		}*/
		
		//if (bt.command != toNop)
		//{
		if (muteOnDbg)
		{
			muteOnDbg = 0;
			bt_mute(1, bt.vol);
		}
		if (muteOffDbg)
		{
			muteOffDbg = 0;
			bt_mute(0, bt.vol);
		}
		
		if (HAL_GetTick() - bt.ticks > 100)
		{
			bt.ticks = HAL_GetTick();
			cycleBuf_count = RING_GetCount(&ringBuf_btCommands);
			if (cycleBuf_count) 
			{
				bt.command = RING_Pop(&ringBuf_btCommands);
				if (bt.command != toNop)
					HAL_UART_Transmit(&huart2, (uint8_t*)&bt_commands[bt.command][0], strlen(&bt_commands[bt.command][0]), 100);
				bt.command = toNop;
			}
		}
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA4 PA5 PA6 
                           PA7 PA8 PA9 PA10 
                           PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB11 PB12 PB13 PB14 
                           PB15 PB3 PB4 PB5 
                           PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
