/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : RemoteControl.c
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
#include "RemoteControl.h"

/* Private includes ----------------------------------------------------------*/
#include "stdlib.h"
#include "stm32f2xx_hal.h"
#include "string.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define ROWS 12
#define COLUMNS 16

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
ControlHandler_t *ControlHandler;

/* Private function prototypes -----------------------------------------------*/
int Control_Parse(uint8_t (*TempBuffer)[COLUMNS], uint8_t *Source,
                  const char Deli);

ControlStatus_t Decode_CMD(ControlInfo_t *Control,
                           uint8_t (*StructBuffer)[COLUMNS]);

ControlStatus_t Decode_PID(ControlInfo_t *Control,
                           uint8_t (*StructBuffer)[COLUMNS]);

ControlStatus_t Decode_Mode(ControlMode_t *Mode,
                            uint8_t (*StructBuffer)[COLUMNS]);

/* Private function definations ----------------------------------------------*/
/**
 * @brief Parse the string with the delimiter.
 * @param TempBuffer The temporary buffer.
 * @param Source The Source of the String.
 * @param Deli the delimiter.
 * @retval int The number of split string.
 */
int Control_Parse(uint8_t (*TempBuffer)[COLUMNS], uint8_t *Source,
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
 * @param Control The Control Pointer.
 * @param StructBuffer The Structural Buffer.
 * @retval ControlStatus_t
 */
ControlStatus_t Decode_CMD(ControlInfo_t *Control,
                           uint8_t (*StructBuffer)[COLUMNS]) {
    int index = 1;
    if (strcmp((const char *)&StructBuffer[0], "CMD") != 0)
        return CONTROL_ERROR;
    while (StructBuffer[index][0] != '\0') {
        if (strcmp((const char *)&StructBuffer[index], "T") == 0) {
            Control->JoyStick.Thrust =
                atoi((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "R") == 0) {
            Control->JoyStick.Roll =
                atoi((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "P") == 0) {
            Control->JoyStick.Pitch =
                atoi((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "Y") == 0) {
            Control->JoyStick.Yaw =
                atoi((const char *)&StructBuffer[index + 1]);
        }
        index += 2;
    }
    return CONTROL_OK;
}

/**
 * @brief Decode the PID message.
 * @param Control The Control Pointer.
 * @param StructBuffer The Structural Buffer.
 * @retval ControlStatus_t
 */
ControlStatus_t Decode_PID(ControlInfo_t *Control,
                           uint8_t (*StructBuffer)[COLUMNS]) {
    int index = 1;
    if (strcmp((const char *)&StructBuffer[0], "PID") != 0)
        return CONTROL_ERROR;
    while (StructBuffer[index][0] != '\0') {
        if (strcmp((const char *)&StructBuffer[index], "PR") == 0) {
            Control->ControlPid.Roll.P =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "IR") == 0) {
            Control->ControlPid.Roll.I =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "DR") == 0) {
            Control->ControlPid.Roll.D =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "PP") == 0) {
            Control->ControlPid.Pitch.P =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "IP") == 0) {
            Control->ControlPid.Pitch.I =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "DP") == 0) {
            Control->ControlPid.Pitch.D =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "PY") == 0) {
            Control->ControlPid.Yaw.P =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "IY") == 0) {
            Control->ControlPid.Yaw.I =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "DY") == 0) {
            Control->ControlPid.Yaw.D =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "PA") == 0) {
            Control->ControlPid.Altitude.P =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "IA") == 0) {
            Control->ControlPid.Altitude.I =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "DA") == 0) {
            Control->ControlPid.Altitude.D =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "PG") == 0) {
            Control->ControlPid.Gps.P =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "IG") == 0) {
            Control->ControlPid.Gps.I =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "DG") == 0) {
            Control->ControlPid.Gps.D =
                atof((const char *)&StructBuffer[index + 1]);
        }
        index += 2;
    }
    return CONTROL_OK;
}

/**
 * @brief Decode the Mode message.
 * @param Mode The Mode Pointer.
 * @param StructBuffer The Structural Buffer.
 * @retval ControlStatus_t
 */
ControlStatus_t Decode_Mode(ControlMode_t *Mode,
                            uint8_t (*StructBuffer)[COLUMNS]) {
    if (strcmp((const char *)&StructBuffer[0], "MOD") != 0)
        return CONTROL_ERROR;
    if (strcmp((const char *)&StructBuffer[1], "MANUAL") == 0) {
        *Mode = MANUAL_MODE;
    } else if (strcmp((const char *)&StructBuffer[1], "ALTITUDE") == 0) {
        *Mode = ALTITUDE_HOLD_MODE;
    } else if (strcmp((const char *)&StructBuffer[1], "GPS") == 0) {
        *Mode = GPS_HOLD_MODE;
    } else if (strcmp((const char *)&StructBuffer[1], "HOME") == 0) {
        *Mode = RETURN_HOME_MODE;
    }
    return CONTROL_OK;
}

/* Function definations _-----------------------------------------------------*/
/**
 * @brief The Initialize for the Remote Control.
 * @param Handler The pointer of the Control Handler.
 * @param Init The Init struct.
 * @retval void
 */
void Control_Init(ControlHandler_t *Handler, ControlInit_t Init) {
    // Copy the user Init struct to the Init struct of the Handler.
    memcpy(&Handler->Init, &Init, sizeof(Init));
    // Copy the Parameter
    memcpy(&Handler->Mode, &Init.Mode, sizeof(Init.Mode));
    memcpy(&Handler->Control.ControlPid, &Init.ControlPid,
           sizeof(Init.ControlPid));
    memcpy(&Handler->Control.JoyStick, &Init.Joystick, sizeof(Init.Joystick));
    Handler->Serial = Init.Serial;
    // Assign the Handler to the Handler of the file
    ControlHandler = Handler;
}

/**
 * @brief Process the data in the Ring Buffer.
 * @retval ControlStatus_t
 */
ControlStatus_t Control_Process(void) {
    uint8_t *Buffer;
    uint8_t StructBuffer[ROWS][COLUMNS];
    ControlStatus_t Status = CONTROL_ERROR;
    if (Detect_Char(ControlHandler->Serial, '\n')) {
        Buffer = malloc(128);
        while (Get_String_NonBlocking(ControlHandler->Serial, Buffer, '\n') >
               0) {
            Control_Parse(StructBuffer, Buffer, ',');
            if (strcmp((const char *)&StructBuffer[0], "CMD") == 0) {
                Status = Decode_CMD(&ControlHandler->Control, StructBuffer);
            }
            if (strcmp((const char *)&StructBuffer[0], "PID") == 0) {
                Status = Decode_PID(&ControlHandler->Control, StructBuffer);
            }
            if (strcmp((const char *)&StructBuffer[0], "MOD") == 0) {
                Status = Decode_Mode(&ControlHandler->Mode, StructBuffer);
            }
            ControlHandler->Status = Status;
        }
        free(Buffer);
    }
    return Status;
}
