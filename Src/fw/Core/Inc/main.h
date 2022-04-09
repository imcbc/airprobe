/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "stdio.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define DBG_LOG_LEVEL 3

#if (DBG_LOG_LEVEL == 1)
    #define PrintERR  printf
    #define PrintDBG  
    #define PrintINFO 
#elif (DBG_LOG_LEVEL == 2)
    #define PrintERR  printf
    #define PrintDBG  printf
    #define PrintINFO
#elif (DBG_LOG_LEVEL == 3)
    #define PrintERR  printf
    #define PrintDBG  printf
    #define PrintINFO printf
#else
    #define PrintERR 
    #define PrintDBG
    #define PrintINFO
#endif
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void delay_us(int val);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY_UP_Pin GPIO_PIN_13
#define KEY_UP_GPIO_Port GPIOC
#define KEY_UP_EXTI_IRQn EXTI15_10_IRQn
#define KEY_BOOT_Pin GPIO_PIN_0
#define KEY_BOOT_GPIO_Port GPIOC
#define USB_CTRL_Pin GPIO_PIN_1
#define USB_CTRL_GPIO_Port GPIOC
#define LED_Y_Pin GPIO_PIN_2
#define LED_Y_GPIO_Port GPIOC
#define KEY_DOWN_Pin GPIO_PIN_3
#define KEY_DOWN_GPIO_Port GPIOC
#define KEY_DOWN_EXTI_IRQn EXTI3_IRQn
#define LIGHTSENSE_Pin GPIO_PIN_0
#define LIGHTSENSE_GPIO_Port GPIOA
#define SPI_CS_Pin GPIO_PIN_4
#define SPI_CS_GPIO_Port GPIOA
#define GDO0_Pin GPIO_PIN_4
#define GDO0_GPIO_Port GPIOC
#define GDO2_Pin GPIO_PIN_5
#define GDO2_GPIO_Port GPIOC
#define PM25_RESET_Pin GPIO_PIN_0
#define PM25_RESET_GPIO_Port GPIOB
#define PM25_SET_Pin GPIO_PIN_1
#define PM25_SET_GPIO_Port GPIOB
#define LED_G2_Pin GPIO_PIN_12
#define LED_G2_GPIO_Port GPIOB
#define LED_G1_Pin GPIO_PIN_13
#define LED_G1_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_14
#define LED_R_GPIO_Port GPIOB
#define CO2_RDY_Pin GPIO_PIN_8
#define CO2_RDY_GPIO_Port GPIOC
#define PA15_Pin GPIO_PIN_15
#define PA15_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define KEY_PRESS_NONE      0
#define KEY_PRESS_SHORT     1
#define KEY_PRESS_LONG      2

#define SYS_ERROR_FLAG_ROM          0x01
#define SYS_ERROR_FLAG_CO2          0x02
#define SYS_ERROR_FLAG_PM25_CHK     0x04
#define SYS_ERROR_FLAG_PM25_EFRAME  0x08
#define SYS_ERROR_FLAG_UART2_DEV    0x10

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
