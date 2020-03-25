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
#define SIZE_x 8
#define SIZE_y 4
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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define IN_1_Pin GPIO_PIN_2
#define IN_1_GPIO_Port GPIOA
#define IN_2_Pin GPIO_PIN_3
#define IN_2_GPIO_Port GPIOA
#define NRF_IRQ_Pin GPIO_PIN_0
#define NRF_IRQ_GPIO_Port GPIOB
#define NRF_PWR_CTRL_Pin GPIO_PIN_13
#define NRF_PWR_CTRL_GPIO_Port GPIOB
#define DIG_7_Pin GPIO_PIN_14
#define DIG_7_GPIO_Port GPIOB
#define DIG_6_Pin GPIO_PIN_15
#define DIG_6_GPIO_Port GPIOB
#define DIG_5_Pin GPIO_PIN_8
#define DIG_5_GPIO_Port GPIOA
#define DIG_4_Pin GPIO_PIN_9
#define DIG_4_GPIO_Port GPIOA
#define DIG_3_Pin GPIO_PIN_10
#define DIG_3_GPIO_Port GPIOA
#define DIG_2_Pin GPIO_PIN_11
#define DIG_2_GPIO_Port GPIOA
#define DIG_1_Pin GPIO_PIN_12
#define DIG_1_GPIO_Port GPIOA
#define SEG_H_Pin GPIO_PIN_15
#define SEG_H_GPIO_Port GPIOA
#define SEG_G_Pin GPIO_PIN_3
#define SEG_G_GPIO_Port GPIOB
#define SEG_F_Pin GPIO_PIN_4
#define SEG_F_GPIO_Port GPIOB
#define SEG_E_Pin GPIO_PIN_5
#define SEG_E_GPIO_Port GPIOB
#define SEG_D_Pin GPIO_PIN_6
#define SEG_D_GPIO_Port GPIOB
#define SEG_C_Pin GPIO_PIN_7
#define SEG_C_GPIO_Port GPIOB
#define SEG_B_Pin GPIO_PIN_8
#define SEG_B_GPIO_Port GPIOB
#define SEG_A_Pin GPIO_PIN_9
#define SEG_A_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
typedef struct
{
    uint8_t buffer[SIZE_x][SIZE_y];
    uint16_t idxIn;
    uint16_t idxOut;
    //uint16_t size;
} RING_buffer_t;

void RING_Put(uint8_t* array, RING_buffer_t* buf);
void RING_Pop(RING_buffer_t *buf, uint8_t* array);
uint16_t RING_GetCount(RING_buffer_t *buf);
int32_t RING_ShowSymbol(uint16_t symbolNumber ,RING_buffer_t *buf);
void RING_Clear(RING_buffer_t* buf);
void RING_Init(RING_buffer_t *buf);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
