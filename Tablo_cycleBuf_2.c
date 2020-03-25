/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "ssd1306_tests.h"
//#include "ssd1306.h"
//#include "stdlib.h"
#include "nRF24.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


#define NRF_ON 	HAL_GPIO_WritePin(NRF_PWR_CTRL_GPIO_Port, NRF_PWR_CTRL_Pin, GPIO_PIN_RESET)
#define NRF_OFF HAL_GPIO_WritePin(NRF_PWR_CTRL_GPIO_Port, NRF_PWR_CTRL_Pin, GPIO_PIN_SET)


typedef enum {
    Sleep = 0x00, // 
    Drowse = 0x01,  //
		Wake = 0x02,
		Unknown = 0x03
} POWER_STATE;
typedef enum {
    To_Sleep = 0x00, // 
    To_Drowse = 0x01,  //
		To_Wake = 0x02,
		No_Command = 0x03
} POWER_STATE_COMMAND;

POWER_STATE pwr_state = Unknown;
POWER_STATE p_pwr_state = Unknown;
POWER_STATE_COMMAND command = No_Command;

typedef struct {
	unsigned char tim4;
	unsigned char one_sec;
	uint8_t display_renew;
} flags_t;
flags_t flag;

typedef struct {
	uint16_t seconds;
	uint8_t adc;
} counters_t;
counters_t counter;

//DEBUG
//volatile uint16_t tim_counter = 0;
//uint16_t res_tim_counter = 0;
uint8_t nrf_pwr = 1;
//uint8_t display_renew = 0;
uint8_t nrf_ready = 0;
volatile uint16_t tx_counter = 0;
uint16_t res_tx_counter = 0;
volatile uint16_t rx_counter_1 = 0;
volatile uint32_t tmp;
uint32_t adc_tmp = 0;
uint32_t adc_value;
//uint8_t adc_counter = 0;
uint8_t volts = 0;
uint8_t sub_volts = 0;
HAL_StatusTypeDef hal_status;

RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
//volatile uint16_t counter = 0;
uint32_t subs = 0;
uint8_t secs = 0;
uint8_t min = 0;
uint32_t subs_normal = 0;
char str_mins[3]={0};
char str_secs[3]={0};
char str_subs[5]={0};
char str_volts[2]={0};
char str_subv[3]={0};

extern nrf24l01_t nrf;

//unsigned char spi_rx_data[PACKET_SIZE];
unsigned char id = 0;
unsigned char state = 0;
unsigned char start = 0;
unsigned char tq_data[PACKET_SIZE] = {'T','Q','x','x'};//
unsigned char id_ok[PACKET_SIZE] = {'T',0,'x','x'};//SetxAB A - adress, B - adress
unsigned char led_counter = 0;


unsigned char rx_mes[4] = {1,5,35,77};
unsigned char bcd_array[7];
unsigned char port_array[7];
volatile uint8_t segment_counter = 0;

//volatile uint16_t sec_counter = 0;
uint8_t test_array[4] = {0};
uint8_t seg_port = 0;

RING_buffer_t ring_Rx;
//uint8_t test_array [4][2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_RTCEx_RTCEventCallback (RTC_HandleTypeDef *hrtc)
{
 if(RTC_IT_SEC)
 {	
		counter.seconds++;
		//flag.one_sec = 1;
 }
}
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim4)	//
{
			
	GPIOB->BRR = (uint32_t)0x03F8;	//port B reset PB3-PB9 anods
	
	GPIOB->BRR = (uint32_t)0xC000;	//port B reset PB14-PB15 part of cathodes
	GPIOA->BRR = (uint32_t)0x9F00;	//port A reset PA8-PA12, PA15 part of cathodes
	
	seg_port = port_array[segment_counter];
	GPIOB->BSRR = (uint32_t)seg_port << 3;	//PB3 - PB9 set
	
	switch (segment_counter)
	{
		case 0: GPIOA->BSRR = (uint32_t)0x1000; break; //PA12
		case 1: GPIOA->BSRR = (uint32_t)0x0800; break; //PA11
		case 2: GPIOA->BSRR = (uint32_t)0x0400; break; //PA10
		case 3: GPIOA->BSRR = (uint32_t)0x0200; break; //PA9
		case 4: GPIOA->BSRR = (uint32_t)0x0100; break; //PA8
		
		case 5: GPIOB->BSRR = (uint32_t)0x8000; break; //PB15
		case 6: GPIOB->BSRR = (uint32_t)0x4000; break; //PB14
	}

	
	if (segment_counter < 6)
	{
		segment_counter++;
	}
	else
	{
		segment_counter = 0;
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc2)
{

  tmp = HAL_ADC_GetValue(hadc2);

}

void dec_to_bcd(unsigned char *dec_array)
{
	unsigned char j = 0;
	for (unsigned char i = 0; i < sizeof(dec_array); i++)
	{	
		if (i == 0)
		{
			bcd_array[j] = dec_array[i];
		}
		else
		{
			bcd_array[j] = dec_array[i]/10;
			j++;
			bcd_array[j] = dec_array[i] - bcd_array[j-1]*10;	
		}
		j++;
	}	
}
unsigned char bcd_to_port(unsigned char bcd)
{
	unsigned char port = 0; 
	switch (bcd)
	{
		case 0: port = 0x7E; break;
		case 1: port = 0x30; break;
		case 2: port = 0x6D; break;
		case 3: port = 0x79; break;
		case 4: port = 0x33; break;
		case 5: port = 0x5B; break;
		case 6: port = 0x5F; break;
		case 7: port = 0x70; break;
		case 8: port = 0x7F; break;
		case 9: port = 0x7B; break;
		
		case 'U': port = 0x3E; break;
		case 'u': port = 0x1C; break;
		case 'S': port = 0x00; break;
		case 't': port = 0x00; break;
		case 'A': port = 0x00; break;
		case 'r': port = 0x00; break;
		case 'O': port = 0x00; break;
		case 'P': port = 0x00; break;
		case 'Y': port = 0x00; break;
		case 'H': port = 0x00; break;
		case '-': port = 0x01; break;
	}
	return port;
}

void RING_Put(uint8_t* array, RING_buffer_t* buf)
{
		for (uint8_t y = 0; y < sizeof(array); y++)
		{
			buf->buffer[buf->idxIn][y] = array[y];
		}
		buf->idxIn++;  
    if (buf->idxIn >= SIZE_x) buf->idxIn = 0;
}
void RING_Pop(RING_buffer_t *buf, uint8_t* array)
{
		for (uint8_t y = 0; y < sizeof(array); y++)
		{
			array[y] = buf->buffer[buf->idxOut][y];
		}
		buf->idxOut++; 
    if (buf->idxOut >= SIZE_x) buf->idxOut = 0;
}
uint16_t RING_GetCount(RING_buffer_t *buf)
{
    uint16_t retval = 0;
    if (buf->idxIn < buf->idxOut) retval = SIZE_x + buf->idxIn - buf->idxOut;
    else retval = buf->idxIn - buf->idxOut;
    return retval;
}/*
void RING_Clear(RING_buffer_t* buf)
{
    buf->idxIn = 0;
    buf->idxOut = 0;
}
void RING_Init(RING_buffer_t *buf)
{
    buf->size = size;
    buf->buffer = test_array[][];//(uint8_t*) malloc(size);
    RING_Clear(buf);
}*/
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
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_ADC2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	HAL_RTCEx_SetSecond_IT(&hrtc);
	NVIC_EnableIRQ(RTC_IRQn);
	
	NVIC_EnableIRQ(TIM4_IRQn);
	HAL_TIM_Base_Start_IT(&htim4);
	
	HAL_PWR_DisableSleepOnExit();
	/*__HAL_RCC_DBGMCU_CLK_ENABLE();*/ //RCC->APB2ENR |= RCC_APB2ENR_DBGMCUEN;
	//HAL_DBGMCU_EnableDBGSleepMode();	//DBGMCU->CR |= DBGMCU_CR_DBG_SLEEP;
	HAL_DBGMCU_EnableDBGStopMode();
	DWT_Delay_Init ();
	

	
	NRF_ON;
	nrf_ready = nrf_init(NRF_CH_TQ);
	if (nrf_ready)
	{
		
	}

	prx(PACKET_SIZE);
	

	test_array[0] = 0; test_array[1] = 0; test_array[2] = 0; test_array[3] = 0;
	RING_Put(test_array, &ring_Rx);
	// /*
	test_array[0] = 1; test_array[1] = 59; test_array[2] = 55; test_array[3] = 9;
	RING_Put(test_array, &ring_Rx);
	test_array[0] = 1; test_array[1] = 47; test_array[2] = 83; test_array[3] = 12;
	RING_Put(test_array, &ring_Rx);
	test_array[0] = 2; test_array[1] = 7; test_array[2] = 37; test_array[3] = 36;
	RING_Put(test_array, &ring_Rx);
	test_array[0] = 1; test_array[1] = 5; test_array[2] = 32; test_array[3] = 77;
	RING_Put(test_array, &ring_Rx);	//*/
	counter.seconds = 5;
	//uint8_t flag = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
			if (counter.seconds == 5)
			{
				flag.one_sec = 1;
				counter.seconds = 0;
				if (RING_GetCount(&ring_Rx))
				{
					RING_Pop(&ring_Rx, test_array);
					flag.display_renew = 1;
				}
			}
			if (flag.display_renew)
			{
				flag.display_renew = 0;
				dec_to_bcd(test_array);
				for (uint8_t i = 0; i < sizeof(port_array);i++)
				{
					port_array[i] = bcd_to_port(bcd_array[i]);
				}
			}

			
			
			//HAL_ADC_Start_IT(&hadc2);
			if (counter.adc <= 10)
			{
				adc_tmp = adc_tmp + tmp;
				counter.adc++;
			}
			else
			{
				adc_value = adc_tmp/68;//80//77
				adc_tmp = 0;
				counter.adc = 0;
				
				volts = adc_value/100;
				sub_volts = adc_value%100;
				//if (adc_value <= 290)
					//command = To_Sleep;
			}
			/*
			Battery voltage analyzing
			*/
			
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
			
			if (flag.one_sec)
			{
				Send_Packet(tq_data);//broadcast request
				prx(PACKET_SIZE);
				flag.one_sec = 0;
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			}
	
	

			
	/*--------------------nRF24L01 IRQ Handler-------------------------*/
	
			if ((HAL_GPIO_ReadPin(nrf_irq) == RESET)&&(nrf_pwr))
			{
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				//nrf.observe_tx = SPI_Read_Reg(OBSERVE_TX);
				nrf.plos_cnt = nrf.observe_tx >> PLOS_CNT;
				//nrf.setup_retr = SPI_Read_Reg(SETUP_RETR);
				
				nrf.status = SPI_Read_Reg(STATUS);
				nrf.rx_dr = (nrf.status >> RX_DR) & 0x01;
				nrf.tx_ds = (nrf.status >> TX_DS) & 0x01;
				nrf.max_rt = (nrf.status >> MAX_RT) & 0x01;
				//nrf.rx_empty = (SPI_Read_Reg(FIFO_STATUS)) & 0x01;
				/*if ((nrf.tx_ds)||(nrf.max_rt))
				{
					nrf.tx_ds = 0;
					nrf.status = SPI_Read_Reg(STATUS);
					SPI_Write_Reg(STATUS, nrf.status);
					prx(PACKET_SIZE);
					 if (nrf.max_rt)
					 {
							nrf.max_rt = 0;
							
					 }
					 if (nrf.plos_cnt == 15)
					 {
							nrf.plos_cnt = 0;
							nrf.rf_ch = SPI_Read_Reg(RF_CH);
							SPI_Write_Reg(RF_CH, nrf.rf_ch);
							
					 }
				Send_Packet(ts_data);
				tx_counter++;
				 }*/
				 if(nrf.rx_dr)
				 {   
						nrf.rx_dr = 0;
						nrf.rx_empty = (SPI_Read_Reg(FIFO_STATUS)) & 0x01;
						while (!nrf.rx_empty)
						{
							 for(uint8_t i_rx = 0; i_rx < PACKET_SIZE; i_rx++)
							 {
									nrf.rx_data[i_rx] = 0;
							 }
							 SPI_Read_Packet(R_RX_PAYLOAD, nrf.rx_data, PACKET_SIZE); 
							 nrf.rx_empty = (SPI_Read_Reg(FIFO_STATUS)) & 0x01;
						}
						SPI_Write_Reg(STATUS, nrf.status);
						SPI_Write_Reg(FLUSH_RX, NOP);
						state = 1;
						rx_counter_1++;
				 }
			}
			/*-------------END_OF_nRF24L01 IRQ Handler-------------------------*/
			if (state == 1)       //Проверка преамбулы пакета
			{
				
				if (nrf.rx_data[0] <= 9)
				{
					if (nrf.rx_data[1] <= 59)
					{
						//counter = 0;
						id_ok[1] = nrf.rx_data[3];
						HAL_Delay(5);//dont work without this delay, dont know why
						Send_Packet(id_ok);
						prx(PACKET_SIZE);
						HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
						state = 2;
					}
				 }
				if (nrf.rx_data[0] == 'T')
				{
						state = 10;
						
						//rx_counter++;
				}
			 }/*END_OF Проверка преамбулы пакета*/
			if (state == 2)       //
			{
				RING_Put(nrf.rx_data, &ring_Rx);
				state = 0;
				//if (!RING_GetCount(&ring_Rx)) //
					//counter.seconds = 5; //
			}
			//tx_counter++;

			 
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */
	
  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1200; //720;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|DIG_5_Pin|DIG_4_Pin|DIG_3_Pin 
                          |DIG_2_Pin|DIG_1_Pin|SEG_H_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|NRF_PWR_CTRL_Pin|DIG_7_Pin|DIG_6_Pin 
                          |SEG_G_Pin|SEG_F_Pin|SEG_E_Pin|SEG_D_Pin 
                          |SEG_C_Pin|SEG_B_Pin|SEG_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IN_1_Pin */
  GPIO_InitStruct.Pin = IN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IN_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IN_2_Pin */
  GPIO_InitStruct.Pin = IN_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IN_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF_IRQ_Pin */
  GPIO_InitStruct.Pin = NRF_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(NRF_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 PB11 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF_PWR_CTRL_Pin */
  GPIO_InitStruct.Pin = NRF_PWR_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NRF_PWR_CTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIG_7_Pin DIG_6_Pin SEG_G_Pin SEG_F_Pin 
                           SEG_E_Pin SEG_D_Pin SEG_C_Pin SEG_B_Pin 
                           SEG_A_Pin */
  GPIO_InitStruct.Pin = DIG_7_Pin|DIG_6_Pin|SEG_G_Pin|SEG_F_Pin 
                          |SEG_E_Pin|SEG_D_Pin|SEG_C_Pin|SEG_B_Pin 
                          |SEG_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DIG_5_Pin DIG_4_Pin DIG_3_Pin DIG_2_Pin 
                           DIG_1_Pin SEG_H_Pin */
  GPIO_InitStruct.Pin = DIG_5_Pin|DIG_4_Pin|DIG_3_Pin|DIG_2_Pin 
                          |DIG_1_Pin|SEG_H_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
  while(1)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
