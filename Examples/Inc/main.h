/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
#define ADXL_ID			0x00	//Device ID
#define THRESH_TAP	0x1D	//Tap threshold
#define OFSX				0x1E	//X-axis offset
#define OFSY				0x1F	//Y-axis offset
#define OFSZ				0x20	//Z-axis offset
#define DUR					0x21	//Tap duration
#define LATENT			0x22	//Tap latency
#define WINDOW			0x23	//Tap window
#define THRESH_ACT	0x24	//Activity threshold
#define THRESH_INACT			0x25	//Inactivity threshold
#define TIME_INACT	0x26	//Inactivity time
#define ACT_INACT_CTL			0x27	//Axis enable control for activity and inactivity detection
			#define ACT_ACDC		0x80	//
			#define ACT_X_EN		0x40	//
			#define ACT_Y_EN		0x20	//
			#define ACT_Z_EN		0x10	//
			#define INACT_ACDC	0x08	//
			#define INACT_X_EN	0x04	//
			#define INACT_Y_EN	0x02	//
			#define INACT_Z_EN	0x01	//
#define THRESH_FF		0x28	//Free-fall threshold
#define TIME_FF			0x29	//Free-fall time
#define TAP_AXES		0x2A	//Axis control for single tap/double tap
#define ACT_TAP_STATUS			0x2B	//Source of single tap/double tap
#define BW_RATE			0x2C	//Data rate and power mode control
			#define LOW_POWER		0x10	//
				/*--------Rate----------*/
				/*The default value is 0x0A, which
			translates to a 100 Hz output data rate.*/
					#define RATE_3200HZ		0x0F	//
					#define RATE_1600HZ		0x0E	//
					#define RATE_800HZ		0x0D	//
					#define RATE_400HZ		0x0C	//
					#define RATE_200HZ		0x0B	//
					#define RATE_100HZ		0x0A	//
					#define RATE_50HZ			0x09	//
					#define RATE_25HZ			0x08	//
					#define RATE_12i5HZ		0x07	//
					#define RATE_6i25HZ		0x06	//
					#define RATE_3i13HZ		0x05	//
					#define RATE_1i56HZ		0x04	//
					#define RATE_0i78HZ		0x03	//
					#define RATE_0i39HZ		0x02	//
					#define RATE_0i2HZ		0x01	//
					#define RATE_0i1HZ		0x00	//
				

#define POWER_CTL		0x2D	//Power-saving features control
			#define LINK				0x20	//
			#define AUTO_SLEEP	0x10	//
			#define MEASURE			0x08	//
			#define SLEEP				0x04	//
			/*--------WakeUp----------*/
				#define Freq_8Hz	0x00	//
				#define Freq_4Hz	0x01	//
				#define Freq_2Hz	0x02	//
				#define Freq_1Hz	0x03	//

#define INT_ENABLE	0x2E	//Interrupt enable control
#define INT_MAP			0x2F	//Interrupt mapping control
#define INT_SOURCE	0x30	//Source of interrupts
			#define DATA_READY	0x80	//
			#define SINGLE_TAP	0x40	//
			#define DOUBLE_TAP	0x20	//
			#define ACTIVITY		0x10	//
			#define INACTIVITY	0x08	//
			#define FREE_FALL		0x04	//
			#define WATERMARK		0x02	//
			#define OVERRUN			0x01	//
			
#define DATA_FORMAT	0x31	//Data format control
			#define SELF_TEST		0x80	//
			#define SPI					0x40	//
			#define INT_INVERT	0x20	//
			#define FULL_RES		0x08	//
			#define JUSTIFY			0x04	//
			/*--------RANGE-----------*/
				#define RANGE_2G	0x00	//
				#define RANGE_4G	0x01	//
				#define RANGE_8G	0x02	//
				#define RANGE_16G	0x03	//

#define DATAX0			0x32	//
#define DATAX1			0x33	//
#define DATAY0			0x34	//
#define DATAY1			0x35	//
#define DATAZ0			0x36	//
#define DATAZ1			0x37	//
#define FIFO_CTL		0x38	//FIFO control
#define ADXL_FIFO_STATUS	0x39	//FIFO status
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
