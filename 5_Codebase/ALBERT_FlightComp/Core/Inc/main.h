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
#include "stm32f4xx_hal.h"

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
#define INT1_NAV_Pin GPIO_PIN_13
#define INT1_NAV_GPIO_Port GPIOC
#define INT2_NAV_Pin GPIO_PIN_14
#define INT2_NAV_GPIO_Port GPIOC
#define SPI1_CS_ACCEL_NAV_Pin GPIO_PIN_1
#define SPI1_CS_ACCEL_NAV_GPIO_Port GPIOC
#define SPI1_CS_GYRO_NAV_Pin GPIO_PIN_2
#define SPI1_CS_GYRO_NAV_GPIO_Port GPIOC
#define SPI1_INT_ACCEL_NAV_Pin GPIO_PIN_3
#define SPI1_INT_ACCEL_NAV_GPIO_Port GPIOC
#define SPI1_INT_ACCEL_NAV_EXTI_IRQn EXTI3_IRQn
#define USART2_TX_NAV_Pin GPIO_PIN_2
#define USART2_TX_NAV_GPIO_Port GPIOA
#define USART2_RX_NAV_Pin GPIO_PIN_3
#define USART2_RX_NAV_GPIO_Port GPIOA
#define SPI1_SCK_NAV_Pin GPIO_PIN_5
#define SPI1_SCK_NAV_GPIO_Port GPIOA
#define SPI1_MISO_NAV_Pin GPIO_PIN_6
#define SPI1_MISO_NAV_GPIO_Port GPIOA
#define SPI1_MOSI_NAV_Pin GPIO_PIN_7
#define SPI1_MOSI_NAV_GPIO_Port GPIOA
#define SPI1_INT_GYRO_NAV_Pin GPIO_PIN_4
#define SPI1_INT_GYRO_NAV_GPIO_Port GPIOC
#define SPI1_INT_GYRO_NAV_EXTI_IRQn EXTI4_IRQn
#define I2C2_SCL_NAV_Pin GPIO_PIN_10
#define I2C2_SCL_NAV_GPIO_Port GPIOB
#define I2C2_SDA_NAV_Pin GPIO_PIN_11
#define I2C2_SDA_NAV_GPIO_Port GPIOB
#define SPI2_SCK_NAV_Pin GPIO_PIN_13
#define SPI2_SCK_NAV_GPIO_Port GPIOB
#define SPI2_MISO_NAV_Pin GPIO_PIN_14
#define SPI2_MISO_NAV_GPIO_Port GPIOB
#define SPI2_MOSI_NAV_Pin GPIO_PIN_15
#define SPI2_MOSI_NAV_GPIO_Port GPIOB
#define SPI2_CS_NAV_Pin GPIO_PIN_6
#define SPI2_CS_NAV_GPIO_Port GPIOC
#define SPI2_INT_NAV_Pin GPIO_PIN_7
#define SPI2_INT_NAV_GPIO_Port GPIOC
#define SPI2_INT_NAV_EXTI_IRQn EXTI9_5_IRQn
#define GPS_TP_NAV_Pin GPIO_PIN_8
#define GPS_TP_NAV_GPIO_Port GPIOC
#define GPS_LNA_NAV_Pin GPIO_PIN_9
#define GPS_LNA_NAV_GPIO_Port GPIOC
#define USART1_RX_NAV_Pin GPIO_PIN_10
#define USART1_RX_NAV_GPIO_Port GPIOA
#define USB_D__NAV_Pin GPIO_PIN_11
#define USB_D__NAV_GPIO_Port GPIOA
#define USB_D__NAVA12_Pin GPIO_PIN_12
#define USB_D__NAVA12_GPIO_Port GPIOA
#define SWDIO_NAV_Pin GPIO_PIN_13
#define SWDIO_NAV_GPIO_Port GPIOA
#define SWCLK_NAV_Pin GPIO_PIN_14
#define SWCLK_NAV_GPIO_Port GPIOA
#define GPIO6_NAV_Pin GPIO_PIN_11
#define GPIO6_NAV_GPIO_Port GPIOC
#define GPIO5_NAV_Pin GPIO_PIN_12
#define GPIO5_NAV_GPIO_Port GPIOC
#define GPIO4_NAV_Pin GPIO_PIN_2
#define GPIO4_NAV_GPIO_Port GPIOD
#define GPIO3_NAV_Pin GPIO_PIN_3
#define GPIO3_NAV_GPIO_Port GPIOB
#define GPIO2_NAV_Pin GPIO_PIN_4
#define GPIO2_NAV_GPIO_Port GPIOB
#define GPIO1_NAV_Pin GPIO_PIN_5
#define GPIO1_NAV_GPIO_Port GPIOB
#define USART1_TX_NAV_Pin GPIO_PIN_6
#define USART1_TX_NAV_GPIO_Port GPIOB
#define I2C1_SDA_NAV_Pin GPIO_PIN_7
#define I2C1_SDA_NAV_GPIO_Port GPIOB
#define I2C1_SCL_NAV_Pin GPIO_PIN_8
#define I2C1_SCL_NAV_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
