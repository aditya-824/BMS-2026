/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

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
#define Prog_LED_2_Pin GPIO_PIN_13
#define Prog_LED_2_GPIO_Port GPIOC
#define Prog_LED_1_Pin GPIO_PIN_14
#define Prog_LED_1_GPIO_Port GPIOC
#define Blink_LED_Pin GPIO_PIN_15
#define Blink_LED_GPIO_Port GPIOC
#define Prog_LED_4_Pin GPIO_PIN_0
#define Prog_LED_4_GPIO_Port GPIOC
#define Spare_SPI_SS_Pin GPIO_PIN_1
#define Spare_SPI_SS_GPIO_Port GPIOC
#define VSBAT_Pin GPIO_PIN_4
#define VSBAT_GPIO_Port GPIOA
#define VSHV_Pin GPIO_PIN_5
#define VSHV_GPIO_Port GPIOA
#define Vehicle_ID_Pin GPIO_PIN_6
#define Vehicle_ID_GPIO_Port GPIOA
#define AMS_Fault_Pin GPIO_PIN_7
#define AMS_Fault_GPIO_Port GPIOA
#define PCR_Done_Pin GPIO_PIN_4
#define PCR_Done_GPIO_Port GPIOC
#define AIR_P_Pin GPIO_PIN_5
#define AIR_P_GPIO_Port GPIOC
#define AIR_N_Pin GPIO_PIN_0
#define AIR_N_GPIO_Port GPIOB
#define PCR_5V_Pin GPIO_PIN_1
#define PCR_5V_GPIO_Port GPIOB
#define VSBATB2_Pin GPIO_PIN_2
#define VSBATB2_GPIO_Port GPIOB
#define T_Check_Pin GPIO_PIN_10
#define T_Check_GPIO_Port GPIOB
#define AMS_Fault_Out_Pin GPIO_PIN_11
#define AMS_Fault_Out_GPIO_Port GPIOB
#define K1_Pin GPIO_PIN_12
#define K1_GPIO_Port GPIOB
#define K2_Pin GPIO_PIN_13
#define K2_GPIO_Port GPIOB
#define SS_Final_Pin GPIO_PIN_14
#define SS_Final_GPIO_Port GPIOB
#define GRN_In_Pin GPIO_PIN_15
#define GRN_In_GPIO_Port GPIOB
#define GRN_Out_Pin GPIO_PIN_6
#define GRN_Out_GPIO_Port GPIOC
#define AUX_P_Pin GPIO_PIN_7
#define AUX_P_GPIO_Port GPIOC
#define AUX_N_Pin GPIO_PIN_8
#define AUX_N_GPIO_Port GPIOC
#define PCR_AUX_In_Pin GPIO_PIN_9
#define PCR_AUX_In_GPIO_Port GPIOC
#define V_Check_Pin GPIO_PIN_8
#define V_Check_GPIO_Port GPIOA
#define SPI1_CS1_Pin GPIO_PIN_2
#define SPI1_CS1_GPIO_Port GPIOD
#define SPI1_CS2_Pin GPIO_PIN_6
#define SPI1_CS2_GPIO_Port GPIOB
#define Prog_LED_3_Pin GPIO_PIN_7
#define Prog_LED_3_GPIO_Port GPIOB
#define CART_ID_Pin GPIO_PIN_8
#define CART_ID_GPIO_Port GPIOB
#define Charger_AUX_Pin GPIO_PIN_9
#define Charger_AUX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
