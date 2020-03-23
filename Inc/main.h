/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
typedef enum
{
	toNop,
	toOn,
	toOff
} BtCommandTypeDef;
typedef enum
{
	btNoCon,
	btOCon,
	btOff
} BtStatusTypeDef;

typedef struct
{
  uint8_t short_resp[8];     
	uint8_t long_resp[32]; 
	

}BtRxMsgTypeDef;
typedef struct
{
  char On[8];     
	char Off[8];   

}BtTxMsgTypeDef;
typedef struct
{
	BtCommandTypeDef command;
	uint8_t var;
	BtRxMsgTypeDef*            pBtRxMsg;     /*!< Pointer to receive structure  */
	BtTxMsgTypeDef*            pBtTxMsg;     /*!< Pointer to transmit structure  */
} BtModuleTypeDef;


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
#define BT_ON 			"COM+PWOS\r\n"
#define BT_OFF 			"COM+PWDS\r\n"
#define BT_RES 			"COM+REBOOT\r\n"
#define BT_VOL_P		"COM+VP\r\n"
#define BT_VOL_M		"COM+VD\r\n"
#define BT_VOL_Q		"COM+GV\r\n"
#define BT_MODE_BT	"COM+MBT\r\n"
#define BT_MODE_Q		"COM+IQ\r\n"
#define	BT_PAIR			"BT+PR\r\n"
#define	BT_CON_LAST	"BT+AC\r\n"
#define	BT_DISCON		"BT+DC\r\n"
#define BT_ANSW_CALL		"BT+CA\r\n"
#define BT_REF_CALL 		"BT+CJ\r\n"
#define BT_HANGUP_CALL	"BT+CE\r\n"
#define BT_REDIAL		"BT+CR\r\n"


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
