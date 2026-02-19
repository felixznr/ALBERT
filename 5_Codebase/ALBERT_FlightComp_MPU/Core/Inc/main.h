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
#include "stm32h7xx_hal.h"

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
#define SPI4_SCK_MPU_Pin GPIO_PIN_2
#define SPI4_SCK_MPU_GPIO_Port GPIOE
#define SPI4_NSS_MPU_Pin GPIO_PIN_4
#define SPI4_NSS_MPU_GPIO_Port GPIOE
#define SPI4_MISO_MPU_Pin GPIO_PIN_5
#define SPI4_MISO_MPU_GPIO_Port GPIOE
#define SPI4_MOSI_MPU_Pin GPIO_PIN_6
#define SPI4_MOSI_MPU_GPIO_Port GPIOE
#define HSE_IN_MPU_Pin GPIO_PIN_0
#define HSE_IN_MPU_GPIO_Port GPIOH
#define HSE_OUT_MPU_Pin GPIO_PIN_1
#define HSE_OUT_MPU_GPIO_Port GPIOH
#define SPI2_MISO_MPU_Pin GPIO_PIN_2
#define SPI2_MISO_MPU_GPIO_Port GPIOC
#define SPI2_MOSI_MPU_Pin GPIO_PIN_3
#define SPI2_MOSI_MPU_GPIO_Port GPIOC
#define USART2_TX_MPU_Pin GPIO_PIN_2
#define USART2_TX_MPU_GPIO_Port GPIOA
#define USART2_RX_MPU_Pin GPIO_PIN_3
#define USART2_RX_MPU_GPIO_Port GPIOA
#define SPI1_CS_MPU_Pin GPIO_PIN_4
#define SPI1_CS_MPU_GPIO_Port GPIOA
#define SPI2_DIO0_MPU_Pin GPIO_PIN_4
#define SPI2_DIO0_MPU_GPIO_Port GPIOC
#define SPI2_DIO2_MPU_Pin GPIO_PIN_5
#define SPI2_DIO2_MPU_GPIO_Port GPIOC
#define LORA_RST_MPU_Pin GPIO_PIN_0
#define LORA_RST_MPU_GPIO_Port GPIOB
#define GPIO1_MPU_Pin GPIO_PIN_7
#define GPIO1_MPU_GPIO_Port GPIOE
#define GPIO2_MPU_Pin GPIO_PIN_8
#define GPIO2_MPU_GPIO_Port GPIOE
#define GPIO3_MPU_Pin GPIO_PIN_9
#define GPIO3_MPU_GPIO_Port GPIOE
#define GPIO4_MPU_Pin GPIO_PIN_10
#define GPIO4_MPU_GPIO_Port GPIOE
#define GPIO5_MPU_Pin GPIO_PIN_11
#define GPIO5_MPU_GPIO_Port GPIOE
#define GPIO6_MPU_Pin GPIO_PIN_12
#define GPIO6_MPU_GPIO_Port GPIOE
#define GPIO7_MPU_Pin GPIO_PIN_13
#define GPIO7_MPU_GPIO_Port GPIOE
#define GPIO8_MPU_Pin GPIO_PIN_14
#define GPIO8_MPU_GPIO_Port GPIOE
#define GPIO9_MPU_Pin GPIO_PIN_15
#define GPIO9_MPU_GPIO_Port GPIOE
#define I2C2_SCL_MPU_Pin GPIO_PIN_10
#define I2C2_SCL_MPU_GPIO_Port GPIOB
#define I2C2_SDA_MPU_Pin GPIO_PIN_11
#define I2C2_SDA_MPU_GPIO_Port GPIOB
#define SPI2_NSS_MPU_Pin GPIO_PIN_12
#define SPI2_NSS_MPU_GPIO_Port GPIOB
#define SPI2_SCK_MPU_Pin GPIO_PIN_13
#define SPI2_SCK_MPU_GPIO_Port GPIOB
#define INT1_MPU_Pin GPIO_PIN_10
#define INT1_MPU_GPIO_Port GPIOD
#define INT2_MPU_Pin GPIO_PIN_11
#define INT2_MPU_GPIO_Port GPIOD
#define Pyro_4_MPU_Pin GPIO_PIN_12
#define Pyro_4_MPU_GPIO_Port GPIOD
#define Pyro_3_MPU_Pin GPIO_PIN_13
#define Pyro_3_MPU_GPIO_Port GPIOD
#define Pyro_2_MPU_Pin GPIO_PIN_14
#define Pyro_2_MPU_GPIO_Port GPIOD
#define Pyro_1_MPU_Pin GPIO_PIN_15
#define Pyro_1_MPU_GPIO_Port GPIOD
#define GPIO10_MPU_Pin GPIO_PIN_6
#define GPIO10_MPU_GPIO_Port GPIOC
#define GPIO11_MPU_Pin GPIO_PIN_7
#define GPIO11_MPU_GPIO_Port GPIOC
#define GPIO12_MPU_Pin GPIO_PIN_8
#define GPIO12_MPU_GPIO_Port GPIOC
#define USART1_TX_MPU_Pin GPIO_PIN_9
#define USART1_TX_MPU_GPIO_Port GPIOA
#define USART1_RX_MPU_Pin GPIO_PIN_10
#define USART1_RX_MPU_GPIO_Port GPIOA
#define USB_D__MPU_Pin GPIO_PIN_11
#define USB_D__MPU_GPIO_Port GPIOA
#define USB_D__MPUA12_Pin GPIO_PIN_12
#define USB_D__MPUA12_GPIO_Port GPIOA
#define SWDIO_MPU_Pin GPIO_PIN_13
#define SWDIO_MPU_GPIO_Port GPIOA
#define SWCLK_MPU_Pin GPIO_PIN_14
#define SWCLK_MPU_GPIO_Port GPIOA
#define SD_DET_MPU_Pin GPIO_PIN_15
#define SD_DET_MPU_GPIO_Port GPIOA
#define LED1_MPU_Pin GPIO_PIN_0
#define LED1_MPU_GPIO_Port GPIOD
#define LED2_MPU_Pin GPIO_PIN_1
#define LED2_MPU_GPIO_Port GPIOD
#define LED3_MPU_Pin GPIO_PIN_2
#define LED3_MPU_GPIO_Port GPIOD
#define LED4_MPU_Pin GPIO_PIN_3
#define LED4_MPU_GPIO_Port GPIOD
#define I2C1_SCL_MPU_Pin GPIO_PIN_6
#define I2C1_SCL_MPU_GPIO_Port GPIOB
#define I2C1_SDA_MPU_Pin GPIO_PIN_7
#define I2C1_SDA_MPU_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
