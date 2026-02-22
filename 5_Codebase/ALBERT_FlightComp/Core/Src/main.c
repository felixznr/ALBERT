/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"
#include <math.h>

#include "LSM6DSLTR.h"
#include "BMM150.h"
#include "BMP390.h"
#include "BMI088.h"

#include "Kalman.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USB_BUFLEN 128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
BMI088 imu1;
LSM6DSLTR imu2;
BMM150 magneto;
BMP390 prs;

Kalman att;                 //  EKF for phi/theta

float Q_att[2] = { 1e-3f, 1e-3f };   // Startwerte (tunen!)
float R_att[3] = { 2e-2f, 2e-2f, 2e-2f }; // Messrauschen für normierten Accel (tunen!)


/* Interrupt Flags and timers*/
volatile uint8_t imu1_acc_drdy = 0;
volatile uint8_t imu1_gyr_drdy = 0;
volatile uint8_t bmp390_drdy   = 0;

volatile uint32_t acc_t = 0,    gyr_t = 0,    bmp_t = 0,	last_gyr_t = 0;

uint8_t vehiclestate = 0;
// Test counters
//volatile uint32_t imu1_acc_irq_cnt = 0;
//volatile uint32_t imu1_gyr_irq_cnt = 0;
//volatile uint32_t bmp390_irq_cnt   = 0;



/* USB data buffer */
uint8_t usbTxBuf[USB_BUFLEN];
uint16_t usbTxBufLen;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  // Enable DWT cycle counter
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // enable trace
  DWT->CYCCNT = 0;                                // reset counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;             // start counter

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* Initialise BMM150 */
  BMM150_Init( &magneto, &hi2c2);

  /* Initialise IMU2 */
  LSM6DSLTR_Init( &imu2, &hi2c2);
  //LSM6DSLTR_OffsetCorrection(&imu2);

  /* Initialise IMU1 */
  BMI088_Init(&imu1, &hspi1);
  //BMI088_OffsetCorrection(&imu1);

  /* Initialise BMP390 Pressure Sensor */
  BMP390_Init(&prs, &hspi2);

  Kalman_Init(&att, 0.5f, Q_att, R_att);   // Pinit z.B. 0.5 (tunen)
  last_gyr_t = gyr_t;                      // wichtig: dt beim ersten Mal nicht riesig


  uint32_t lastPrint = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  if (imu1_gyr_drdy) {
	      imu1_gyr_drdy = 0;

	      BMI088_ReadAngularRate(&imu1);
	      //BMI088_ReadAcceleration(&imu1);

	      float dt = (float)(gyr_t - last_gyr_t) / (float)SystemCoreClock;
	      last_gyr_t = gyr_t;


	      Kalman_Predict(&att, imu1.gyr_rdps, dt);




	  }

	  if (imu1_acc_drdy) {
		  imu1_acc_drdy = 0;
		  BMI088_ReadAcceleration(&imu1);
		  Kalman_Update(&att, imu1.acc_mps2);

	  }


		if (bmp390_drdy) {
			bmp390_drdy = 0;
			BMP390_ReadPressure(&prs);
		}

//		Test Routine for interrupts
		if (HAL_GetTick() - lastPrint > 500)
		{
			lastPrint = HAL_GetTick();
//
//			char msg[128];
//			uint8_t pc4 = HAL_GPIO_ReadPin(SPI1_INT_GYRO_NAV_GPIO_Port, SPI1_INT_GYRO_NAV_Pin);
//			sprintf(msg,
//			        "ACC_IRQ:%lu | GYR_IRQ:%lu | BMP_IRQ:%lu | PC4:%u\r\n",
//			        imu1_acc_irq_cnt, imu1_gyr_irq_cnt, bmp390_irq_cnt, pc4);


			  /* Print IMU1 Accel Data over USB-C*/
			  usbTxBufLen = snprintf((char*)usbTxBuf, USB_BUFLEN,
			      "IMU1 ACCEL:\r\n	X=%.3fms2\r\n	Y=%.3fms2\r\n	Z=%.3fms2\r\n\n",
			      imu1.acc_mps2[0],
				  imu1.acc_mps2[1],
				  imu1.acc_mps2[2]);
			  CDC_Transmit_FS(usbTxBuf, usbTxBufLen);
//
//			  /* Print IMU1 Gyro Data over USB-C*/
//			  usbTxBufLen = snprintf((char*)usbTxBuf, USB_BUFLEN,
//			      "IMU1 GYR:\r\n	X=%.3frads\r\n	Y=%.3frads\r\n	Z=%.3frads\r\n\n",
//			      imu1.gyr_rdps[0],
//				  imu1.gyr_rdps[1],
//				  imu1.gyr_rdps[2]);
//			  CDC_Transmit_FS(usbTxBuf, usbTxBufLen);

			  usbTxBufLen = snprintf((char*)usbTxBuf, USB_BUFLEN,
			      "ATT:\r\n\tphi=%.2f deg\r\n\ttheta=%.2f deg\r\n\n",
			      att.phi_rad   * 57.2957795f,
			      att.theta_rad * 57.2957795f);
			  CDC_Transmit_FS(usbTxBuf, usbTxBufLen);



		}





//
//
//	  /* Print Pressure Data over USB-C*/
//	  usbTxBufLen = snprintf((char*)usbTxBuf, USB_BUFLEN,
//	      "%.3f;\r\n",
//		  prs.prs_Pa);
//	  CDC_Transmit_FS(usbTxBuf, usbTxBufLen);

//
//
//	  /* Print IMU2 Gyroscope Data over USB-C*/
//	  usbTxBufLen = snprintf((char*)usbTxBuf, USB_BUFLEN,
//	      "IMU2 GYRO:\r\n	X=%.3frdps\r\n	Y=%.3frdps\r\n	Z=%.3frdps\r\n\n",
//	      imu2.gyr_rdps[0],
//		  imu2.gyr_rdps[1],
//		  imu2.gyr_rdps[2]);
//	  CDC_Transmit_FS(usbTxBuf, usbTxBufLen);
//
//	  /* Print IMU2 Accelerometer Data over USB-C*/
//	  usbTxBufLen = snprintf((char*)usbTxBuf, USB_BUFLEN,
//	  	      "IMU2 ACCELEROMETER:\r\n	X=%.2fms2\r\n	Y=%.2f ms2\r\n	Z=%.2f ms2\r\n\n",
//	  	      imu2.acc_mps2[0],
//	  	      imu2.acc_mps2[1],
//	  	      imu2.acc_mps2[2]);
//	  CDC_Transmit_FS(usbTxBuf, usbTxBufLen);

//
//	  /* Print Magnetometer Data over USB-C*/
//	  usbTxBufLen = snprintf((char*)usbTxBuf, USB_BUFLEN,
//	  	      "MAGNETOMETER:\r\n	X=%.2f\r\n	Y=%.2f\r\n	Z=%.2f\r\n\n",
//	  	      magneto.mag_uT[0],
//			  magneto.mag_uT[1],
//			  magneto.mag_uT[2]);
//	  CDC_Transmit_FS(usbTxBuf, usbTxBufLen);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_MultiProcessor_Init(&huart1, 0, UART_WAKEUPMETHOD_IDLELINE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, INT2_NAV_Pin|SPI1_CS_ACCEL_NAV_Pin|SPI1_CS_GYRO_NAV_Pin|SPI2_CS_NAV_Pin
                          |GPIO6_NAV_Pin|GPIO5_NAV_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO4_NAV_GPIO_Port, GPIO4_NAV_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO3_NAV_Pin|GPIO2_NAV_Pin|GPIO1_NAV_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : INT1_NAV_Pin SPI1_INT_ACCEL_NAV_Pin SPI1_INT_GYRO_NAV_Pin SPI2_INT_NAV_Pin */
  GPIO_InitStruct.Pin = INT1_NAV_Pin|SPI1_INT_ACCEL_NAV_Pin|SPI1_INT_GYRO_NAV_Pin|SPI2_INT_NAV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : INT2_NAV_Pin SPI1_CS_ACCEL_NAV_Pin SPI1_CS_GYRO_NAV_Pin GPIO6_NAV_Pin
                           GPIO5_NAV_Pin */
  GPIO_InitStruct.Pin = INT2_NAV_Pin|SPI1_CS_ACCEL_NAV_Pin|SPI1_CS_GYRO_NAV_Pin|GPIO6_NAV_Pin
                          |GPIO5_NAV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_NAV_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_NAV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_NAV_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPS_TP_NAV_Pin GPS_LNA_NAV_Pin */
  GPIO_InitStruct.Pin = GPS_TP_NAV_Pin|GPS_LNA_NAV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO4_NAV_Pin */
  GPIO_InitStruct.Pin = GPIO4_NAV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO4_NAV_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO3_NAV_Pin GPIO2_NAV_Pin GPIO1_NAV_Pin */
  GPIO_InitStruct.Pin = GPIO3_NAV_Pin|GPIO2_NAV_Pin|GPIO1_NAV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/*Use DWT instead of halgettick because of higher resolution*/
	uint32_t t = DWT->CYCCNT;

    if (GPIO_Pin == SPI1_INT_ACCEL_NAV_Pin) {
        imu1_acc_drdy = 1;
        acc_t = t;
        //imu1_acc_irq_cnt++;
    }
    else if (GPIO_Pin == SPI1_INT_GYRO_NAV_Pin) {
        imu1_gyr_drdy = 1;
        gyr_t = t;
        //imu1_gyr_irq_cnt++;

    }
    else if (GPIO_Pin == SPI2_INT_NAV_Pin) {
        bmp390_drdy = 1;
        bmp_t = t;
        //bmp390_irq_cnt++;

    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
