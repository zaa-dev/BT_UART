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
#include "ssd1306_tests.h"
#include "ssd1306.h"
#include "stdlib.h"
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

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


#define SHAKE_TRES  40	//amount of shaking IRQ's to wake up from DROWSE
#define SHAKE_TIME  5		//TIME to count shaking IRQ's
#define WAKE_T			600	//timer from wake_up to sleep, if there isn't shake IRQ's
#define COUNT_T			300 //upper limit for racing time
volatile uint8_t shake_int = 0;
volatile uint16_t wake_to_sleep_t = WAKE_T;


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
	unsigned char display_renew;
	unsigned char adc;
} flags_t;
flags_t flag;

//DEBUG
volatile uint16_t tim_counter = 0;
uint16_t res_tim_counter = 0;
uint16_t message_num = 0;
uint16_t message_secs = 0;
uint16_t message_subs = 0;
uint8_t read = 0;
volatile uint16_t rx_counter = 0;
uint16_t res_rx_counter = 0;
volatile uint32_t tmp;
uint32_t adc_tmp = 0;
uint32_t adc_value;
uint8_t adc_counter = 0;
uint8_t volts = 0;
uint8_t sub_volts = 0;
HAL_StatusTypeDef hal_status;

uint8_t display_renew = 0;

RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
volatile uint16_t counter = 0;
uint32_t subs = 0;
uint8_t secs = 0;
uint8_t min = 0;
uint32_t subs_normal = 0;
char str_mins[3]={0};
char str_secs[3]={0};
char str_subs[4]={0};
char str_volts[2]={0};
char str_subv[3]={0};

extern nrf24l01_t nrf;

unsigned char spi_rx_data[PACKET_SIZE];
unsigned char state = 0;
unsigned char start = 0;
unsigned char finish = 0;
unsigned char ts_data[PACKET_SIZE] = {'T','S','x','x'};//SetxAB A - adress, B - adress
unsigned char tr_data[PACKET_SIZE] = {'T','R','x','x'};//SetxAB A - adress, B - adress
unsigned char led_counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_RTCEx_RTCEventCallback (RTC_HandleTypeDef *hrtc)
{
 if(RTC_IT_SEC)
 {	
	 if ( (start) && (counter < COUNT_T) )
	 {counter++;}
	 else if ((start) && (counter >= COUNT_T) )
	 {start = 0; wake_to_sleep_t = WAKE_T;}
	 
	 if (++led_counter > 255)
		 led_counter = 0;
	 
	 if( (!start) && (pwr_state == Wake) )
	 {
			if (wake_to_sleep_t)
				wake_to_sleep_t--;
			else
				command = To_Sleep;
	 }
	 res_rx_counter = rx_counter;
	 rx_counter = 0;
	 
	 res_tim_counter = tim_counter;
	 tim_counter = 0;
 }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin== GPIO_PIN_0)
	{
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		if (shake_int < SHAKE_TRES)
			shake_int++;
		if (pwr_state == Sleep)
		{pwr_state = Drowse; p_pwr_state = Sleep; shake_int = 0;}
		else if (pwr_state == Wake)
		{wake_to_sleep_t = WAKE_T;}
	}
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc2)

{

  tmp = HAL_ADC_GetValue(hadc2);
	flag.adc = 1;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim4)
{
	tim_counter++;
	flag.display_renew = 1;
	HAL_ADC_Start_IT(&hadc2);
}
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
  //MX_I2C1_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
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
	
	ssd1306_Init(); 
	ssd1306_Fill(Black);
	ssd1306_UpdateScreen();
	HAL_Delay(500);
	ssd1306_SetCursor(0, 2);
  ssd1306_WriteString("ver: 0.1", Font_11x18, White);
  //ssd1306_UpdateScreen();
	//HAL_Delay(1500);
	
	//ssd1306_Fill(Black);
	ssd1306_SetCursor(0, 36);
  ssd1306_WriteString("id: 1", Font_11x18, White);
  ssd1306_UpdateScreen();
	HAL_Delay(1500);
	
	ssd1306_Fill(Black);
	ssd1306_SetCursor(26, 36);
  ssd1306_WriteString("START!", Font_16x26, White);
  ssd1306_UpdateScreen();
	HAL_Delay(1500);
	
	nrf_init();
	
	wake_to_sleep_t = WAKE_T;
	pwr_state = Wake;
//	prx(PACKET_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		/*Shake analyze*/

		while (pwr_state == Drowse)
		{
			if (p_pwr_state != pwr_state)
			{
				p_pwr_state = pwr_state;
				led_counter = 0;
				SystemClock_Config(); MX_RTC_Init();
				HAL_RTCEx_SetSecond_IT(&hrtc); NVIC_EnableIRQ(RTC_IRQn);
			}
			if ( led_counter < SHAKE_TIME )
			{
				HAL_Delay(100);
			}
			else if ( ( shake_int >= SHAKE_TRES ) )
			{
				command = To_Wake;
				shake_int = 0;
			}
			else if ( ( led_counter >= SHAKE_TIME ) && (shake_int < SHAKE_TRES) )
			{
				command = To_Sleep;
				shake_int = 0;
			}
				
			if (command == To_Sleep)
			{
				command = No_Command;
				pwr_state = Sleep;
				POWERDOWN();	//nRF
				
				ssd1306_SetCursor(0, 36);	
				ssd1306_WriteString("Inactive", Font_16x26, White);
				ssd1306_UpdateScreen();
				HAL_Delay(1500);
				ssd1306_WriteCommand(0xAE);
				hal_status = HAL_RTC_DeInit(&hrtc); 
				//HAL_PWR_EnterSLEEPMode(0, PWR_SLEEPENTRY_WFI); //__WFI();
				/**/
				//@arg PWR_MAINREGULATOR_ON: Stop mode with regulator ON
				//@arg PWR_LOWPOWERREGULATOR_ON: Stop mode with low power regulator ON
				HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
			}
			else if (command == To_Wake)
			{
				//SystemClock_Config(); MX_RTC_Init();
				ssd1306_Init();
				nrf_init();
				ssd1306_SetCursor(26, 36); ssd1306_WriteString("Active", Font_16x26, White);ssd1306_UpdateScreen();
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET); led_counter = 0;
				pwr_state = Wake;
			}
		}
	
		/*End of *Shake analyze*/
		/*WAKE HANDLING*/
		while (pwr_state == Wake)
		{
			if (p_pwr_state != pwr_state)
			{
				p_pwr_state = pwr_state;
				wake_to_sleep_t = WAKE_T;
			}
			if (command == To_Sleep)
			{
				command = No_Command;
				pwr_state = Sleep;
				POWERDOWN();	//nRF
				
				ssd1306_SetCursor(0, 36);	
				ssd1306_WriteString("Inactive", Font_16x26, White);
				ssd1306_UpdateScreen();
				HAL_Delay(1500);
				ssd1306_WriteCommand(0xAE);
				hal_status = HAL_RTC_DeInit(&hrtc); 
				//HAL_PWR_EnterSLEEPMode(0, PWR_SLEEPENTRY_WFI); //__WFI();
				/**/
				//@arg PWR_MAINREGULATOR_ON: Stop mode with regulator ON
				//@arg PWR_LOWPOWERREGULATOR_ON: Stop mode with low power regulator ON
				HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
			}
			
			//HAL_ADC_Start_IT(&hadc2);
			if (flag.adc)
			{
				flag.adc = 0;
			
				if (adc_counter <= 10)
				{
					adc_tmp = adc_tmp + tmp;
					adc_counter++;
				}
				else
				{
					adc_value = adc_tmp/80;//80//77
					adc_tmp = 0;
					adc_counter = 0;
					
					volts = (uint8_t)(adc_value/100);
					sub_volts = (uint8_t)(adc_value%100);
					//if (adc_value <= 290)
						//command = To_Sleep;
				}
				/*
				Battery voltage analyzing
				*/
			}
			
			min = counter/60;
			secs = counter - (min*60);
			uint16_t tmp = 0;
			tmp = RTC->DIVL;
			subs = 32768 - (((uint32_t)RTC->DIVH << 16 ) | tmp) ;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
			if (start)
				subs_normal = (subs * 1000)/32768;
		
			HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
			//HAL_Delay(40);
			if (flag.display_renew)
			{
				
				flag.display_renew = 0;
				
				string_clear(str_mins);
				string_clear(str_secs);
				string_clear(str_subs);	
				string_clear(str_volts);
				string_clear(str_subv);
				ssd1306_Fill(Black);

				sprintf(str_volts, "%01d,", volts);
				sprintf(str_subv, "%02i V", sub_volts);
				
				ssd1306_SetCursor(0,2);
				ssd1306_WriteString("Bat: ",Font_11x18, White);
				ssd1306_WriteString(str_volts, Font_11x18, White);
				ssd1306_WriteString(str_subv, Font_11x18, White);
			
				
				//sprintf(str_mins, "%01d:", min);
				//sprintf(str_secs, "%02d:", secs);
				sprintf(str_subs, "Rx:%04d", res_rx_counter);//sprintf(str_subs, "%03d", subs_normal);
				
				ssd1306_SetCursor(0,36);
				//ssd1306_WriteString(str_mins, Font_16x26, White);
				//ssd1306_WriteString(str_secs, Font_16x26, White);
				ssd1306_WriteString(str_subs, Font_16x26, White);
				ssd1306_UpdateScreen();
			}
			if (led_counter > 3)
			{HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_SET);}

			
	/*--------------------nRF24L01 IRQ Handler-------------------------*/
			if (HAL_GPIO_ReadPin(nrf_irq) == RESET)
			{
				//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				nrf.observe_tx = SPI_Read_Reg(OBSERVE_TX);
				nrf.plos_cnt = nrf.observe_tx >> PLOS_CNT;
				nrf.setup_retr = SPI_Read_Reg(SETUP_RETR);
				nrf.status = SPI_Read_Reg(STATUS);
				nrf.rx_dr = (nrf.status >> RX_DR) & 0x01;
				nrf.tx_ds = (nrf.status >> TX_DS) & 0x01;
				nrf.max_rt = (nrf.status >> MAX_RT) & 0x01;
				nrf.rx_empty = (SPI_Read_Reg(FIFO_STATUS)) & 0x01;
				if ((nrf.tx_ds)||(nrf.max_rt))
				{
					nrf.tx_ds = 0;
					nrf.status = SPI_Read_Reg(STATUS);
					SPI_Write_Reg(STATUS, nrf.status);
					prx(PACKET_SIZE);
					 if (nrf.max_rt)
					 {
							nrf.max_rt = 0;
							/**/
					 }
					 if (nrf.plos_cnt == 15)
					 {
							nrf.plos_cnt = 0;
							nrf.rf_ch = SPI_Read_Reg(RF_CH);
							SPI_Write_Reg(RF_CH, nrf.rf_ch);
							/**/
					 }
				 }
				 if(nrf.rx_dr)
				 {   
						nrf.rx_dr = 0;
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
				 }
			}
			/*-------------END_OF_nRF24L01 IRQ Handler-------------------------*/
			if (state == 1)       //Проверка преамбулы пакета
			{
				state = 0;
				if (nrf.rx_data[0] == 'T')
				{
					if ((nrf.rx_data[1] == 'R')&&(!start))
					{
						//counter = 0;
						//start = 1;
						HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
						wake_to_sleep_t = WAKE_T;
						message_num = (nrf.rx_data[2]<<8)+nrf.rx_data[3];
						message_secs = message_num/1000;
						message_subs = message_num - message_secs*1000;
						counter = message_secs;
						start = 1;
						finish = 0;
					}
					else
					if (nrf.rx_data[1] == 'S')
					{
						start = 0;
						wake_to_sleep_t = WAKE_T;
						rx_counter++;
						
						if (!finish)
						{
							finish = 1;
							subs_normal = subs_normal + message_subs;
							if (subs_normal >= 1000)
							{
								counter = counter + 1;
								subs_normal = subs_normal - 1000;
							}
						}
					}
				 }
			 }/*END_OF Проверка преамбулы пакета*/
		
		 }/*END_OF *WAKE HANDLING*/ 
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

  /**Initializes the CPU, AHB and APB busses clocks 
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
  /**Initializes the CPU, AHB and APB busses clocks 
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
  /**Common config 
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
  /**Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  /**Initialize RTC Only 
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

  /**Initialize RTC and set the Time and Date 
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim4.Init.Prescaler = 7200;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB12 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
