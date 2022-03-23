/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : uartRingBufDMA.h
 * @brief          : Header for uartRingBufDMA.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Sang Tan Truong.
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
#ifndef __UARTRINGBUFDMA_H_
#define __UARTRINGBUFDMA_H_

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
#include "stdbool.h"
#include "stdlib.h"
#include "stm32f2xx_hal.h"
#include "string.h"

/* Private defines -----------------------------------------------------------*/
#define USE_RING_1 1
#define USE_RING_2 1
#define USE_RING_3 1

#define uart1 USART1
#define uart2 USART2
#define uart3 USART3

#define RING_BUFFER_SIZE 1024
#define RING_RX_BUFFER 64
#define RING_TIMEOUT 5000  // ms
/* Exported types ------------------------------------------------------------*/
typedef struct RingHandler_t {
    bool enable;
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef *hdma;
    int Head;
    int Tail;
    uint8_t MainBuffer[RING_BUFFER_SIZE];
    uint8_t RxBuffer[RING_RX_BUFFER];
    uint32_t Timeout;
} RingHandler_t;

typedef struct RingBuffer_t {
    RingHandler_t Ring1;
    RingHandler_t Ring2;
    RingHandler_t Ring3;
    uint32_t (*GetTime)(void);
} RingBuffer_t;
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Ring_Init(RingBuffer_t *Handle);

void Ring_Reset(void);

bool Detect_Char(RingHandler_t *RingHandler, const char Deli);

int Is_available(RingHandler_t *RingHandler);

uint8_t get_char(RingHandler_t *RingHandler);

int get_string(RingHandler_t *RingHandler, uint8_t *Buffer, uint16_t Size);

int get_peek(RingHandler_t *RingHandler, uint8_t *Buffer, uint16_t Size);

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

void Ring_Restart_Uart(RingHandler_t *RingHandle);

uint32_t Get_Time();

void Ring_Increase_Tail(RingHandler_t *Ring, uint16_t Size);

void Ring_Copy_Buffer(RingHandler_t *Ring, uint8_t *Buffer, uint16_t Size);

int IndexOf(uint8_t *Buffer, char Chr, size_t Size);

int IndexOfString(uint8_t *Buffer, char Chr);

int Get_String_NonBlocking(RingHandler_t *Ring, uint8_t *Buffer,
                           const char Terminate);

int Get_String_Util(uint8_t *Buffer, uint8_t *Source, const char Terminate,
                    size_t Size);

int Get_String_Wait(RingHandler_t *Ring, uint8_t *Buffer, const char Terminate);

#endif /* __UARTRINGBUFDMA_H_ */
