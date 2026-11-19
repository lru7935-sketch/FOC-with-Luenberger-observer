/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define FLASH_nSCS_Pin GPIO_PIN_3
#define FLASH_nSCS_GPIO_Port GPIOC
#define USART_to_USB_TX_Pin GPIO_PIN_2
#define USART_to_USB_TX_GPIO_Port GPIOA
#define USART_to_USB_RX_Pin GPIO_PIN_3
#define USART_to_USB_RX_GPIO_Port GPIOA
#define PWM_LED_Pin GPIO_PIN_4
#define PWM_LED_GPIO_Port GPIOA
#define FLASH_SCK_Pin GPIO_PIN_5
#define FLASH_SCK_GPIO_Port GPIOA
#define FLASH_SDO_Pin GPIO_PIN_6
#define FLASH_SDO_GPIO_Port GPIOA
#define FLASH_SDI_Pin GPIO_PIN_7
#define FLASH_SDI_GPIO_Port GPIOA
#define SD_nSCS_Pin GPIO_PIN_4
#define SD_nSCS_GPIO_Port GPIOC
#define BOOT0_PULL_DOWN_Pin GPIO_PIN_5
#define BOOT0_PULL_DOWN_GPIO_Port GPIOC
#define UC_Pin GPIO_PIN_0
#define UC_GPIO_Port GPIOB
#define UB_Pin GPIO_PIN_1
#define UB_GPIO_Port GPIOB
#define UA_Pin GPIO_PIN_2
#define UA_GPIO_Port GPIOB
#define BOT_BREAK_Pin GPIO_PIN_10
#define BOT_BREAK_GPIO_Port GPIOB
#define BOT_BREAK_EXTI_IRQn EXTI15_10_IRQn
#define SOA_Pin GPIO_PIN_11
#define SOA_GPIO_Port GPIOB
#define SOB_Pin GPIO_PIN_12
#define SOB_GPIO_Port GPIOB
#define SOC_Pin GPIO_PIN_13
#define SOC_GPIO_Port GPIOB
#define STEMP_Pin GPIO_PIN_14
#define STEMP_GPIO_Port GPIOB
#define SVBUS_Pin GPIO_PIN_15
#define SVBUS_GPIO_Port GPIOB
#define PWM_INLC_Pin GPIO_PIN_6
#define PWM_INLC_GPIO_Port GPIOC
#define PWM_INHC_Pin GPIO_PIN_7
#define PWM_INHC_GPIO_Port GPIOC
#define PWM_INLB_Pin GPIO_PIN_8
#define PWM_INLB_GPIO_Port GPIOC
#define PWM_INHB_Pin GPIO_PIN_9
#define PWM_INHB_GPIO_Port GPIOC
#define PWM_INLA_Pin GPIO_PIN_8
#define PWM_INLA_GPIO_Port GPIOA
#define PWM_INHA_Pin GPIO_PIN_9
#define PWM_INHA_GPIO_Port GPIOA
#define DRV8340_ENABLE_Pin GPIO_PIN_10
#define DRV8340_ENABLE_GPIO_Port GPIOA
#define DRV8340_nSCS_Pin GPIO_PIN_15
#define DRV8340_nSCS_GPIO_Port GPIOA
#define DRV8340_SCK_Pin GPIO_PIN_10
#define DRV8340_SCK_GPIO_Port GPIOC
#define DRV8340_SDO_Pin GPIO_PIN_11
#define DRV8340_SDO_GPIO_Port GPIOC
#define DRV8340_SDI_Pin GPIO_PIN_12
#define DRV8340_SDI_GPIO_Port GPIOC
#define DRV8340_nFAULT_Pin GPIO_PIN_2
#define DRV8340_nFAULT_GPIO_Port GPIOD
#define DRV8340_nFAULT_EXTI_IRQn EXTI2_IRQn
#define USART_EXT_Pin GPIO_PIN_6
#define USART_EXT_GPIO_Port GPIOB
#define USART_EXTB7_Pin GPIO_PIN_7
#define USART_EXTB7_GPIO_Port GPIOB
#define ENCODER_I2C_SCL_Pin GPIO_PIN_8
#define ENCODER_I2C_SCL_GPIO_Port GPIOB
#define ENCODER_I2C_SDA_Pin GPIO_PIN_9
#define ENCODER_I2C_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
