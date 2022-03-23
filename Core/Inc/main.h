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
#include "stm32f2xx_hal.h"

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
int _write(int file, char *outgoing, int len);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RF_CS_Pin GPIO_PIN_4
#define RF_CS_GPIO_Port GPIOA
#define RF_SCK_Pin GPIO_PIN_5
#define RF_SCK_GPIO_Port GPIOA
#define RF_MISO_Pin GPIO_PIN_6
#define RF_MISO_GPIO_Port GPIOA
#define RF_MOSI_Pin GPIO_PIN_7
#define RF_MOSI_GPIO_Port GPIOA
#define TEL_4G_IO_Pin GPIO_PIN_1
#define TEL_4G_IO_GPIO_Port GPIOB
#define CP_TX_Pin GPIO_PIN_10
#define CP_TX_GPIO_Port GPIOB
#define CP_RX_Pin GPIO_PIN_11
#define CP_RX_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_14
#define LED3_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_15
#define LED4_GPIO_Port GPIOB
#define RF_CE_Pin GPIO_PIN_8
#define RF_CE_GPIO_Port GPIOA
#define TEL_FC_TX_Pin GPIO_PIN_9
#define TEL_FC_TX_GPIO_Port GPIOA
#define TEL_FC_RX_Pin GPIO_PIN_10
#define TEL_FC_RX_GPIO_Port GPIOA
#define BUZZ_Pin GPIO_PIN_11
#define BUZZ_GPIO_Port GPIOA
#define NEOPIXEL_Pin GPIO_PIN_15
#define NEOPIXEL_GPIO_Port GPIOA
#define BTN1_Pin GPIO_PIN_3
#define BTN1_GPIO_Port GPIOB
#define BTN2_Pin GPIO_PIN_4
#define BTN2_GPIO_Port GPIOB
#define BTN3_Pin GPIO_PIN_5
#define BTN3_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_6
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_7
#define OLED_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
