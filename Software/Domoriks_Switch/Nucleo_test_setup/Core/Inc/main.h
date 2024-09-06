/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define L1_Pin GPIO_PIN_5
#define L1_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define B3_Pin GPIO_PIN_8
#define B3_GPIO_Port GPIOC
#define B2_Pin GPIO_PIN_9
#define B2_GPIO_Port GPIOC
#define B6_Pin GPIO_PIN_0
#define B6_GPIO_Port GPIOD
#define B4_Pin GPIO_PIN_2
#define B4_GPIO_Port GPIOD
#define B5_Pin GPIO_PIN_3
#define B5_GPIO_Port GPIOD
#define L5_Pin GPIO_PIN_5
#define L5_GPIO_Port GPIOD
#define L6_Pin GPIO_PIN_6
#define L6_GPIO_Port GPIOD
#define L2_Pin GPIO_PIN_3
#define L2_GPIO_Port GPIOB
#define L3_Pin GPIO_PIN_4
#define L3_GPIO_Port GPIOB
#define L4_Pin GPIO_PIN_5
#define L4_GPIO_Port GPIOB
#define W_RS485_Pin GPIO_PIN_8
#define W_RS485_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
extern UART_HandleTypeDef huart1;

#define RECEIVE_BUFFER_SIZE 100
extern uint8_t received_buffer[RECEIVE_BUFFER_SIZE];
extern uint8_t uart_index;
extern uint32_t timer_lastbyte;
extern uint8_t new_uartstream;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
