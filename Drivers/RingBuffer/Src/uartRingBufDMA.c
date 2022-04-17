/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : uartRingBufDMA.c
 * @brief          : The source file of the library
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
/* Includes ------------------------------------------------------------------*/
#include "uartRingBufDMA.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static RingBuffer_t *RingBuffer;
/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/
/**
 * @brief Initialize the Ring Buffer.
 * @param Handle The pointer of the Ring Buffer Handler.
 * @retval void
 */
void Ring_Init(RingBuffer_t *Handle) {
    RingBuffer = Handle;
    Ring_Reset();
    if (RingBuffer->Ring1.enable == true) {
        Ring_Restart_Uart(&RingBuffer->Ring1);
    }
    if (RingBuffer->Ring2.enable == true) {
        Ring_Restart_Uart(&RingBuffer->Ring2);
    }
    if (RingBuffer->Ring3.enable == true) {
        Ring_Restart_Uart(&RingBuffer->Ring3);
    }
}

/**
 * @brief Detect the particular character.
 * @param RingHandler The Ring Handler.
 * @param Deli The character needs to find.
 * @retval bool True means detected and False is remaining case.
 */
bool Detect_Char(RingHandler_t *RingHandler, const char Deli) {
    bool retval = false;
    uint8_t *Temp = NULL;
    int Availabe;
    Availabe = Is_available(RingHandler);
    if (Availabe > 0) {
        Temp = malloc(RING_BUFFER_SIZE);
        if (Temp == NULL) return -1;
        get_peek(RingHandler, Temp, Availabe);
        int Index = IndexOf(Temp, Deli, Availabe);
        if(Index != -1) {
            retval = true;
        }
        free(Temp);
    }
    return retval;
}

/**
 * @brief Check the data available or not.
 * @param RingHandler The Ring Handler.
 * @retval int: zero meant empty, > 0 mean available data.
 */
int Is_available(RingHandler_t *RingHandler) {
    uint16_t Head = RingHandler->Head;
    uint16_t Tail = RingHandler->Tail;
    int available = 0;
    if (Head > Tail) {
        available = Head - Tail;
    } else if (Head < Tail) {
        available = RING_BUFFER_SIZE - Tail + Head;
    }
    return available;
}

/**
 * @brief Get char from the buffer.
 * @param RingHandler The Ring Handler.
 * @retval uint8_t
 */
uint8_t get_char(RingHandler_t *RingHandler) {
    uint8_t retval = '\0';
    uint16_t Tail = RingHandler->Tail;
    if (RingHandler->Head != Tail) {
        retval = RingHandler->MainBuffer[Tail];
        Ring_Increase_Tail(RingHandler, 1);
    }

    return retval;
}

/**
 * @brief Get string from the buffer.
 * @note This function is difference with the get_peek function. This function
 * will increse the tail of the buffer.
 * @param RingHandler The Ring Handler.
 * @param Buffer The pointer of the buffer.
 * @param Size The number char to read.
 * @retval int The number of char in the buffer.
 */
int get_string(RingHandler_t *RingHandler, uint8_t *Buffer, uint16_t Size) {
    int DataCanRead = get_peek(RingHandler, Buffer, Size);
    if (DataCanRead != 0) {
        Ring_Increase_Tail(RingHandler, DataCanRead);
    }
    return DataCanRead;
}

/**
 * @brief Get peek string from the buffer.
 * @param RingHandler The Ring Handler.
 * @param Buffer The pointer of the buffer.
 * @param Size The number char to read.
 * @retval int The number of char in the buffer.
 */
int get_peek(RingHandler_t *RingHandler, uint8_t *Buffer, uint16_t Size) {
    int available = Is_available(RingHandler);
    int DataToRead = Size;
    if (Size > available) {
        DataToRead = available;
    }
    Ring_Copy_Buffer(RingHandler, Buffer, DataToRead);
    return DataToRead;
}

/**
 * @brief Ring Buffer reset.
 * @retval void
 */
void Ring_Reset(void) {
    // Reset for the first ring
    if (RingBuffer->Ring1.Timeout == 0) {
        RingBuffer->Ring1.Timeout = RING_TIMEOUT;
    }
    RingBuffer->Ring1.Head = 0;
    RingBuffer->Ring1.Tail = 0;
    memset(RingBuffer->Ring1.MainBuffer, '\0', RING_BUFFER_SIZE);
    // Reset for the second ring
    if (RingBuffer->Ring2.Timeout == 0) {
        RingBuffer->Ring2.Timeout = RING_TIMEOUT;
    }
    RingBuffer->Ring2.Head = 0;
    RingBuffer->Ring2.Tail = 0;
    memset(RingBuffer->Ring2.MainBuffer, '\0', RING_BUFFER_SIZE);
    // Reset for the third ring
    if (RingBuffer->Ring3.Timeout == 0) {
        RingBuffer->Ring3.Timeout = RING_TIMEOUT;
    }
    RingBuffer->Ring3.Head = 0;
    RingBuffer->Ring3.Tail = 0;
    memset(RingBuffer->Ring3.MainBuffer, '\0', RING_BUFFER_SIZE);
}
/**
 * @brief The function handle the UART interrupt.
 * @param huart The pointer of the uart handler.
 * @param Size The size of the data.
 * @retval void
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    // Variable declaration
    RingHandler_t *Ring = &RingBuffer->Ring1;
    int pos;
// Find the ring buffer handler
#ifdef USE_RING_1
    if (huart->Instance == uart1) {
        Ring = &RingBuffer->Ring1;
    }
#endif
#ifdef USE_RING_2
    if (huart->Instance == uart2) {
        Ring = &RingBuffer->Ring2;
    }
#endif
#ifdef USE_RING_3
    if (huart->Instance == uart3) {
        Ring = &RingBuffer->Ring3;
    }
#endif
    pos = Ring->Head;
    // Calculate remaining size
    int Remain = Ring->Tail - Ring->Head;
    if (Remain <= 0) {
        Remain += RING_BUFFER_SIZE;
    }
    // Check if the new data exceeds the remain buffer size
    if (pos + Size >= RING_BUFFER_SIZE) {
        uint16_t DataToCopy = RING_BUFFER_SIZE - pos;
        memcpy((Ring->MainBuffer + pos), Ring->RxBuffer, DataToCopy);
        pos = 0;
        memcpy(Ring->MainBuffer, (Ring->RxBuffer + DataToCopy),
               Size - DataToCopy);
        Ring->Head = Size - DataToCopy;
    } else {
        memcpy(Ring->MainBuffer + pos, Ring->RxBuffer, Size);
        Ring->Head = pos + Size;
    }
    //Increase the Tail pointer above the Head 1
    if(Size >= Remain) {
        Ring->Tail = Ring->Head + 1;
    }
    // Start the DMA again
    Ring_Restart_Uart(Ring);
}

/**
 * @brief Restart the Uart.
 * @param RingHandler The pointer of the Ring Handler.
 * @retval void
 */
void Ring_Restart_Uart(RingHandler_t *RingHandle) {
    HAL_UARTEx_ReceiveToIdle_DMA(RingHandle->huart, RingHandle->RxBuffer,
                                 RING_RX_BUFFER);
    __HAL_DMA_DISABLE_IT(RingHandle->hdma, DMA_IT_HT);
}

/**
 * @brief Get time in ms. Must be assign the get time function.
 * @retval uint32_t
 */
uint32_t Get_Time() {
    if (RingBuffer->GetTime == NULL) {
        return 0;
    }
    return RingBuffer->GetTime();
}
/**
 * @brief Increase the Tail.
 * @param Ring The pointer of the Ring.
 * @param Size The size to increase.
 * @param void
 */
void Ring_Increase_Tail(RingHandler_t *Ring, uint16_t Size) {
    uint16_t Tail = Ring->Tail;
    if (Tail + Size >= RING_BUFFER_SIZE) {
        Ring->Tail = Tail + Size - RING_BUFFER_SIZE;
    } else {
        Ring->Tail += Size;
    }
}

/**
 * @brief Copy the main buffer to the user buffer.
 * @param Ring The pointer of the Ring Handler.
 * @param Buffer The pointer of the buffer.
 * @param Size The size to copy.
 * @retval void
 */
void Ring_Copy_Buffer(RingHandler_t *Ring, uint8_t *Buffer, uint16_t Size) {
    uint16_t Tail = Ring->Tail;
    uint16_t Remain = RING_BUFFER_SIZE - Tail;
    if (Size > Remain) {
        memcpy(Buffer, (const void *)&Ring->MainBuffer[Tail], Remain);
        memcpy(Buffer + Remain, (const void *)Ring->MainBuffer, Size - Remain);
    } else {
        memcpy(Buffer, (const void *)&Ring->MainBuffer[Tail], Size);
    }
}

/**
 * @brief Calculate the Index of the character.
 * @param Buffer The pointer of the buffer.
 * @param Chr The char to find.
 * @param Size The size of the buffer to find.
 * @retval int The position of the Character.
 */
int IndexOf(uint8_t *Buffer, char Chr, size_t Size) {
    int Index = -1;
    void *Result = memchr((const void *)Buffer, Chr, Size);
    if (Result == NULL) {
        return -1;
    }
    Index = Result - (void *)Buffer;
    return Index;
}

/**
 * @brief Calculate the Index of the string.
 * @note Must be confirm the buffer finish with the '\0' char.
 * @param Buffer The pointer of the buffer.
 * @param Chr The char to find.
 * @retval int The position of the Character.
 */
int IndexOfString(uint8_t *Buffer, char Chr) {
    int Index = IndexOf(Buffer, Chr, strlen((const char *)Buffer));
    return Index;
}

/**
 * @brief Get the String with the specific termante.
 * @param Buffer The pointer to the buffer.
 * @param Source The pointer of the Source.
 * @param Terminate The terminate character.
 * @param Size The size of the buffer.
 * @retval int The length of the string. Return -1 when can not find the
 * termanated.
 */
int Get_String_Util(uint8_t *Buffer, uint8_t *Source, const char Terminate,
                    size_t Size) {
    int Index = IndexOf(Source, Terminate, Size);
    if (Index > 0) {
        memset(Buffer + Index, '\0', 1);
        memcpy(Buffer, Source, Index);
    }
    return Index;
}

/**
 * @brief Get the string contains the terminate character with Blocking.
 * @param Ring The Ring Handler.
 * @param Buffer The reading buffer.
 * @param Terminate The terminated character.
 * @retval int The length of the string was read. -1 meant timeout or Error.
 */
int Get_String_NonBlocking(RingHandler_t *Ring, uint8_t *Buffer,
                           const char Terminate) {
    uint8_t *Temp = NULL;
    int Length = -1;
    int Availabe;
    Temp = malloc(RING_BUFFER_SIZE);
    if (Temp == NULL) return -1;
    Availabe = Is_available(Ring);
    if (Availabe > 0) {
        get_peek(Ring, Temp, Availabe);
        Length = Get_String_Util(Buffer, Temp, Terminate, Availabe);
        Ring_Increase_Tail(Ring, Length + 1);
    }
    free(Temp);
    return Length;
}

/**
 * @brief Get the string in the ring buffer with the terminate character.
 * @note Make sure the Get Time function already defined.
 * @param Ring The Ring Handler.
 * @param Buffer The pointer of the buffer.
 * @param Terminate The termanated character.
 * @retval int The length of the string was read. -1 meant timeout or Error.
 */
int Get_String_Wait(RingHandler_t *Ring, uint8_t *Buffer,
                    const char Terminate) {
    uint32_t TimeStart = RingBuffer->GetTime();
    uint32_t CurrentTime = TimeStart;
    uint8_t *Temp = NULL;
    uint8_t *CheckData = NULL;
    int LengthRead = -1;
    Temp = malloc(RING_BUFFER_SIZE);
    CheckData = malloc(RING_BUFFER_SIZE);
    if (Temp == NULL || CheckData == NULL) return -1;
    while ((CurrentTime - TimeStart) < Ring->Timeout) {
        int Available = Is_available(Ring);
        if (Available > 0) {
            get_peek(Ring, Temp, Available);
            int Length = Get_String_Util(CheckData, Temp, Terminate, Available);
            if (Length >= 0) {
                Ring_Increase_Tail(Ring, Length + 1);
                memset(Buffer + Length, '\0', 1);
                memcpy(Buffer, CheckData, Length);
                LengthRead = Length;
                break;
            }
        }
        CurrentTime = RingBuffer->GetTime();
    }
    free(Temp);
    free(CheckData);
    return LengthRead;
}
