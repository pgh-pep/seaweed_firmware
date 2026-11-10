/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32g4xx_hal.h"

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
#define CELL1_IN_Pin GPIO_PIN_0
#define CELL1_IN_GPIO_Port GPIOA
#define CELL2_IN_Pin GPIO_PIN_1
#define CELL2_IN_GPIO_Port GPIOA
#define CELL3_IN_Pin GPIO_PIN_2
#define CELL3_IN_GPIO_Port GPIOA
#define CELL4_IN_Pin GPIO_PIN_3
#define CELL4_IN_GPIO_Port GPIOA
#define CHRG_FET_Pin GPIO_PIN_4
#define CHRG_FET_GPIO_Port GPIOA
#define DISCHRG_FET_Pin GPIO_PIN_5
#define DISCHRG_FET_GPIO_Port GPIOA
#define CS_VIN_Pin GPIO_PIN_0
#define CS_VIN_GPIO_Port GPIOB
#define FLT_STATUS1_Pin GPIO_PIN_9
#define FLT_STATUS1_GPIO_Port GPIOA
#define FLT_STATUS2_Pin GPIO_PIN_10
#define FLT_STATUS2_GPIO_Port GPIOA
#define CAN_RX_Pin GPIO_PIN_11
#define CAN_RX_GPIO_Port GPIOA
#define CAN_TX_Pin GPIO_PIN_12
#define CAN_TX_GPIO_Port GPIOA
#define SAF_CIRC_Pin GPIO_PIN_6
#define SAF_CIRC_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
