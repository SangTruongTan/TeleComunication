/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : Radio.c
 * @brief          : Main program body
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
#include "Radio.h"
/* Private includes ----------------------------------------------------------*/
#include <stdlib.h>
#include <string.h>

#include "RemoteControl.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define RADIO_MAX_CMD_LENGTH 256
#define ROWS 32
#define COLUMNS 32

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
RadioHandler_t *RadioHandler;

uint64_t RxpipeAddrs = 0x11223344AA;
/* Private function prototypes -----------------------------------------------*/
void Radio_Send2FC(uint8_t *Buffer);
void Radio_Process_From_FC();
int Radio_Control_Parse(uint8_t (*TempBuffer)[COLUMNS], uint8_t *Source,
                        const char Deli);
void Radio_Decode_CMD(uint8_t (*Struct)[COLUMNS]);
void Radio_Decode_PID(uint8_t (*StructBuffer)[COLUMNS]);
void Radio_Decode_Ack(uint8_t (*StructBuffer)[COLUMNS]);
void Radio_Decode_Mode(uint8_t (*StructBuffer)[COLUMNS]);
/* Function definations ------------------------------------------------------*/
/**
 * @brief Init the Radio task.
 * @param Handle The pointer of the handler.
 * @param Init The init of the radio.
 * @param void
 */
void Radio_Init(RadioHandler_t *Handle, RadioInit_t Init) {
    RadioHandler = Handle;
    RadioHandler->Init = Init;
    RadioHandler->enableDebug = Init.enableDebug;
    RadioHandler->Serial = Init.Serial;
    RadioHandler->Debug = Init.Debug;
    RadioHandler->Display = Init.Display;
    NRF24_begin(RadioHandler->Init.nrfInit);
    nrf24_DebugUART_Init(*Init.Debug->huart);
    NRF24_setChannel(12);
    NRF24_setPayloadSize(32);
    NRF24_setDataRate(RF24_2MBPS);
    NRF24_setPALevel(RF24_PA_0dB);
    NRF24_openReadingPipe(0, RxpipeAddrs);
    NRF24_enableDynamicPayloads();
    NRF24_enableAckPayload();
    NRF24_setAutoAck(true);
    NRF24_startListening();
    printRadioSettings();
}

/**
 * @brief Process the Radio task. Called periodic.
 * @retval RadioStatus_t
 */
RadioStatus_t Radio_Process() {
    static uint8_t DataFromRemote[RADIO_MAX_CMD_LENGTH];
    if (NRF24_available() == true) {
        uint8_t *RxData;
        RxData = malloc(33);
        memset(RxData, '\0', 33);
        NRF24_read(RxData, 32);
        strcat((char *)DataFromRemote, (char *)RxData);
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
        // Process Data From Remote Control
        if (IndexOf(DataFromRemote, '\n', strlen((char *)DataFromRemote)) >=
            0) {
            Radio_Send2FC(RxData);
            memset(DataFromRemote, '\0', RADIO_MAX_CMD_LENGTH);
        }
        // Check the buffer from the FC interface
        if (Detect_Char(RadioHandler->Serial, '\n') == true) {
            // Put the Processing function here. But, don't modify the
            // RingBuffer.
            Radio_Process_From_FC();
            int Available = Is_available(RadioHandler->Serial);
            if (Available > 32) Available = 32;
            memset(RxData, '\0', 33);
            get_string(RadioHandler->Serial, RxData, Available);
            // Put the parse string and choose the destination here.
            NRF24_writeAckPayload(0, RxData, 32);
        }
        free(RxData);
    }
    // Check the buffer from the FC interface
    if (Detect_Char(RadioHandler->Serial, '\n') == true) {
        // Put the Processing function here. But, don't modify the
        // RingBuffer.
        Radio_Process_From_FC();
    }
    return RADIO_OK;
}

/**
 * @brief Send the data to FC and process them.
 * @param Buffer The pointer of the Buffer.
 * @retval void
 */
void Radio_Send2FC(uint8_t *Buffer) {
    uint8_t Struct[ROWS][COLUMNS];
    HAL_UART_Transmit(RadioHandler->Serial->huart, Buffer,
                      strlen((const char *)Buffer), 100);
    if (RadioHandler->enableDebug == true) {
        printf("%s\r\n", Buffer);
    }
    // Put the Process here
    Radio_Control_Parse(Struct, Buffer, ',');
    if (strcmp((const char *)&Struct[0], "CMD") == 0) {
        Radio_Decode_CMD(Struct);
    }
}

/**
 * @brief Process the data from the FC.
 * @retval void
 */
void Radio_Process_From_FC() {
    if (Detect_Char(RadioHandler->Serial, '\n') == true) {
        uint8_t Struct[ROWS][COLUMNS];
        uint8_t *Buffer = NULL;
        uint8_t *Temp = NULL;
        int Availabe;
        Buffer = malloc(RADIO_MAX_CMD_LENGTH);
        Temp = malloc(RING_BUFFER_SIZE);
        if (Temp == NULL || Buffer == NULL) return;
        Availabe = Is_available(RadioHandler->Serial);
        if (Availabe > 0) {
            get_peek(RadioHandler->Serial, Temp, Availabe);
            Get_String_Util(Buffer, Temp, '\n', Availabe);
        }
        // Put the process with the Parse string function
        Radio_Control_Parse(Struct, Buffer, ',');
        if (strcmp((const char *)&Struct[0], "MOD") == 0) {
            Radio_Decode_Mode(Struct);
        } else if (strcmp((const char *)&Struct[0], "ACK") == 0) {
            Radio_Decode_Ack(Struct);
        }
        free(Buffer);
        free(Temp);
    }
}

/**
 * @brief Parse the string with the delimiter.
 * @param TempBuffer The temporary buffer.
 * @param Source The Source of the String.
 * @param Deli the delimiter.
 * @retval int The number of split string.
 */
int Radio_Control_Parse(uint8_t (*TempBuffer)[COLUMNS], uint8_t *Source,
                        const char Deli) {
    int length = 0;
    char *Head = malloc(32);
    Head = strtok((char *)Source, &Deli);
    while (Head != NULL) {
        strcpy((char *)&(TempBuffer)[length], Head);
        length++;
        Head = strtok(NULL, &Deli);
    }
    free(Head);
    return length;
}

/**
 * @brief Decode the CMD message.
 * @param Struct The Structural Buffer.
 * @retval ControlStatus_t
 */
void Radio_Decode_CMD(uint8_t (*Struct)[COLUMNS]) {
    int index = 1;
    Joystick_t *Control = &RadioHandler->Display->Joys;
    if (strcmp((const char *)&Struct[0], "CMD") != 0) return;
    while (Struct[index][0] != '\0') {
        if (strcmp((const char *)&Struct[index], "T") == 0) {
            Control->Thrust = atoi((const char *)&Struct[index + 1]);
        } else if (strcmp((const char *)&Struct[index], "R") == 0) {
            Control->Roll = atoi((const char *)&Struct[index + 1]);
        } else if (strcmp((const char *)&Struct[index], "P") == 0) {
            Control->Pitch = atoi((const char *)&Struct[index + 1]);
        } else if (strcmp((const char *)&Struct[index], "Y") == 0) {
            Control->Yaw = atoi((const char *)&Struct[index + 1]);
        }
        index += 2;
    }
}

/**
 * @brief Decode the PID message.
 * @param StructBuffer The Structural Buffer.
 * @retval void
 */
void Radio_Decode_PID(uint8_t (*StructBuffer)[COLUMNS]) {
    int index = 1;
    ControlPid_t *ControlPid = &RadioHandler->Display->Pid;
    if (strcmp((const char *)&StructBuffer[0], "PID") != 0) return;
    while (StructBuffer[index][0] != '\0') {
        if (strcmp((const char *)&StructBuffer[index], "PR") == 0) {
            ControlPid->Roll.P = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "IR") == 0) {
            ControlPid->Roll.I = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "DR") == 0) {
            ControlPid->Roll.D = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "PP") == 0) {
            ControlPid->Pitch.P = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "IP") == 0) {
            ControlPid->Pitch.I = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "DP") == 0) {
            ControlPid->Pitch.D = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "PY") == 0) {
            ControlPid->Yaw.P = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "IY") == 0) {
            ControlPid->Yaw.I = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "DY") == 0) {
            ControlPid->Yaw.D = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "PA") == 0) {
            ControlPid->Altitude.P =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "IA") == 0) {
            ControlPid->Altitude.I =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "DA") == 0) {
            ControlPid->Altitude.D =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "PG") == 0) {
            ControlPid->Gps.P = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "IG") == 0) {
            ControlPid->Gps.I = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "DG") == 0) {
            ControlPid->Gps.D = atof((const char *)&StructBuffer[index + 1]);
        }
        index += 2;
    }
}

/**
 * @brief Decode ACK message from Flight Controller.
 * @param Struct The pointer of the struct.
 * @retval void
 */
void Radio_Decode_Ack(uint8_t (*StructBuffer)[COLUMNS]) {
    int index = 1;
    SensorParameters_t *Parameter = &RadioHandler->Display->SystemParameters;
    if (strcmp((const char *)&StructBuffer[0], "ACK") != 0) return;
    while (StructBuffer[index][0] != '\0') {
        if (strcmp((const char *)&StructBuffer[index], "R") == 0) {
            Parameter->Roll = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "P") == 0) {
            Parameter->Pitch = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "Y") == 0) {
            Parameter->Yaw = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "B") == 0) {
            Parameter->VBat = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "A") == 0) {
            Parameter->Altitude = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "L") == 0) {
            Parameter->Gps.Latitude =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "O") == 0) {
            Parameter->Gps.Longitude =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "S") == 0) {
            Parameter->Gps.Sattellites =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "F") == 0) {
            if (StructBuffer[index + 1][0] == '1') {
                Parameter->Gps.Fixed = true;
            } else {
                Parameter->Gps.Fixed = false;
            }
        }
        index += 2;
    }
}

/**
 * @brief Decode Mode from the Flight Controller.
 * @param Struct The pointer of the struct.
 * @retval void
 */
void Radio_Decode_Mode(uint8_t (*StructBuffer)[COLUMNS]) {
    OLEDDisplay_t *dis = RadioHandler->Display;
    if (strcmp((const char *)&StructBuffer[0], "MOD") != 0) return;
    strcpy(dis->Mode, (const char *)&StructBuffer[1]);
}

/* Private user code ---------------------------------------------------------*/
