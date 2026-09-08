/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart7;

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

extern UART_HandleTypeDef huart6;

extern UART_HandleTypeDef huart10;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_UART7_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);
void MX_USART10_UART_Init(void);

/* USER CODE BEGIN Prototypes */
/* Initialize after osKernelInitialize, before starting tasks. Task-context TX. */
void USART3_TxMutexInit(void);
HAL_StatusTypeDef USART3_TransmitLocked(const uint8_t *data, uint16_t size,
                                       uint32_t timeout_ms);
extern uint8_t RS485_rx_dma_buffer[100] ;
extern uint8_t RS485_dma_buffer[100] ;
extern volatile uint8_t RS485_flag;
extern volatile uint8_t USART2_flag;



extern uint8_t USART1_rx_dma_buffer[30] ;
extern uint8_t USART1_dma_buffer[30] ;
extern uint8_t USART2_rx_dma_buffer[40];
extern uint8_t USART2_dma_buffer[40];
extern char  USART_LCD_buffer[40] ;
extern volatile uint8_t USART1_flag;
extern volatile uint8_t Blute_tooth_flag;

extern int8_t USART6_rx_dma_buffer[15];
extern int8_t USART6_dma_buffer[15];

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

