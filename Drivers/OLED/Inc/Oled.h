/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : Oled.h
 * @brief          : Header for Oled.c file.
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
#ifndef __OLED_H_
#define __OLED_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f2xx.h"

/* Private includes ----------------------------------------------------------*/
#include <stdbool.h>

#include "RemoteControl.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
typedef enum OLEDStatus_t {
    OLED_OK = 0U,
    OLED_ERROR = 1U,
} OLEDStatus_t;

typedef struct OLEDControlPin_t {
    uint16_t Pin;
    GPIO_TypeDef *Port;
    bool AutoChange;
} OLEDControlPin_t;

typedef struct OLEDInit_t {
    void (*Wait)(uint32_t);
    uint32_t (*GetTime)();
    OLEDControlPin_t ControlPin;
    int SelectPage;
} OLEDInit_t;

typedef struct Gpstime_t {
    uint8_t Hour;
    uint8_t Minute;
    uint8_t Seconds;
} Gpstime_t;

typedef struct GpsData_t {
    float Longitude;
    float Latitude;
    bool Fixed;
    char Lat_East;
    char Lat_North;
    Gpstime_t Time;
    int Sattellites;
} GpsData_t;

typedef struct SensorParameters_t {
    float Roll;
    float Pitch;
    float Yaw;
    float VBat;
    float Altitude;
    GpsData_t Gps;
} SensorParameters_t;

typedef struct OLEDDisplay_t {
    char Mode[10];
    ControlPid_t Pid;
    Joystick_t Joys;
    SensorParameters_t SystemParameters;
} OLEDDisplay_t;

typedef struct OLEDHandle_t {
    OLEDInit_t Init;
    OLEDStatus_t Status;
    int SelectPage;
    OLEDDisplay_t Display;
    OLEDControlPin_t ControlPin;
} OLEDHandle_t;

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
OLEDStatus_t Oled_Init(OLEDHandle_t *Handle);
OLEDStatus_t Oled_Process(void);
/* Private defines -----------------------------------------------------------*/
#endif /* __OLED_H */
