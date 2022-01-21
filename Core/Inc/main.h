/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#define Data_Pin GPIO_PIN_9
#define Data_GPIO_Port GPIOB
#define Clock_Pin GPIO_PIN_14
#define Clock_GPIO_Port GPIOC
#define Clock_EXTI_IRQn EXTI4_15_IRQn
#define Strobe_Pin GPIO_PIN_15
#define Strobe_GPIO_Port GPIOC
#define Strobe_EXTI_IRQn EXTI4_15_IRQn
#define Tx_uC_Pin GPIO_PIN_0
#define Tx_uC_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOA
#define CH20_Pin GPIO_PIN_2
#define CH20_GPIO_Port GPIOA
#define CH19_Pin GPIO_PIN_3
#define CH19_GPIO_Port GPIOA
#define CH18_Pin GPIO_PIN_4
#define CH18_GPIO_Port GPIOA
#define CH17_Pin GPIO_PIN_5
#define CH17_GPIO_Port GPIOA
#define CH16_Pin GPIO_PIN_6
#define CH16_GPIO_Port GPIOA
#define CH15_Pin GPIO_PIN_7
#define CH15_GPIO_Port GPIOA
#define CH14_Pin GPIO_PIN_0
#define CH14_GPIO_Port GPIOB
#define CH13_Pin GPIO_PIN_1
#define CH13_GPIO_Port GPIOB
#define CH12_Pin GPIO_PIN_2
#define CH12_GPIO_Port GPIOB
#define CH11_Pin GPIO_PIN_8
#define CH11_GPIO_Port GPIOA
#define Bus_Sense_Pin GPIO_PIN_9
#define Bus_Sense_GPIO_Port GPIOA
#define Bus_In_Pin GPIO_PIN_6
#define Bus_In_GPIO_Port GPIOC
#define CH1_Pin GPIO_PIN_10
#define CH1_GPIO_Port GPIOA
#define CH2_Pin GPIO_PIN_11
#define CH2_GPIO_Port GPIOA
#define CH3_Pin GPIO_PIN_12
#define CH3_GPIO_Port GPIOA
#define Prog_SWDIO_Pin GPIO_PIN_13
#define Prog_SWDIO_GPIO_Port GPIOA
#define Prog_SWCLK_Pin GPIO_PIN_14
#define Prog_SWCLK_GPIO_Port GPIOA
#define CH4_Pin GPIO_PIN_15
#define CH4_GPIO_Port GPIOA
#define CH5_Pin GPIO_PIN_3
#define CH5_GPIO_Port GPIOB
#define CH6_Pin GPIO_PIN_4
#define CH6_GPIO_Port GPIOB
#define CH7_Pin GPIO_PIN_5
#define CH7_GPIO_Port GPIOB
#define CH8_Pin GPIO_PIN_6
#define CH8_GPIO_Port GPIOB
#define CH9_Pin GPIO_PIN_7
#define CH9_GPIO_Port GPIOB
#define CH10_Pin GPIO_PIN_8
#define CH10_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define SCAN_2000_ALWAYS_HIGH_BITS ((1 << 10) | (1 << 7))     // 0x0480
#define SCAN_2000_CH1_OFF 1 << 17
#define SCAN_2000_CH1_ON 1 << 16
#define SCAN_2000_CH2_OFF 1 << 19
#define SCAN_2000_CH2_ON 1 << 18
#define SCAN_2000_CH3_OFF 1 << 21
#define SCAN_2000_CH3_ON 1 << 20
#define SCAN_2000_CH4_OFF 1 << 23
#define SCAN_2000_CH4_ON 1 << 22
#define SCAN_2000_CH5_OFF 1 << 8
#define SCAN_2000_CH5_ON 1 << 9
#define SCAN_2000_CH6_OFF 1 << 14
#define SCAN_2000_CH6_ON 1 << 13
#define SCAN_2000_CH7_OFF 1 << 0
#define SCAN_2000_CH7_ON 1 << 15
#define SCAN_2000_CH8_OFF 1 << 2
#define SCAN_2000_CH8_ON 1 << 1
#define SCAN_2000_CH9_OFF 1 << 4
#define SCAN_2000_CH9_ON 1 << 3
#define SCAN_2000_CH10_OFF 1 << 5
#define SCAN_2000_CH10_ON 1 << 6
#define SCAN_2000_4W_OFF 1 << 12
#define SCAN_2000_4W_ON 1 << 11
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
