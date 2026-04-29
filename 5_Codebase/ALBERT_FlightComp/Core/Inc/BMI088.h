/*
 *
 * BMI088 SPI Driver
 *
 * Author: Felix Zauner
 * Created: 22.01.2025
 *
 */

#ifndef BMI088_SPI_DRIVER_H
#define BMI088_SPI_DRIVER_H

#include "stm32f4xx_hal.h"


/*
 * 	DEFINES
 */



/* Register Addresses */

/* Accelerometer */
#define BMI088_ACC_CHIP_ID       		0x00
#define BMI088_ACC_ERR_REG        		0x02
#define BMI088_ACC_STATUS         		0x03
#define BMI088_ACC_DATA           		0x12
#define BMI088_TEMP_DATA         		0x22
#define BMI088_ACC_CONF           		0x40
#define BMI088_ACC_RANGE          		0x41
#define BMI088_INT1_IO_CTRL       		0x53
#define BMI088_INT2_IO_CTRL       		0x54
#define BMI088_INT_MAP_DATA 	  		0x58
#define BMI088_ACC_SELF_TEST      		0x6D
#define BMI088_ACC_PWR_CONF       		0x7C
#define BMI088_ACC_PWR_CTRL     	  	0x7D
#define BMI088_ACC_SOFTRESET	      	0x7E

/* Gyroscope */
#define BMI088_GYR_CHIP_ID         		0x00
#define BMI088_GYR_DATA             	0x02
#define BMI088_GYR_RANGE            	0x0F
#define BMI088_GYR_BANDWIDTH        	0x10
#define BMI088_GYR_LPM1             	0x11
#define BMI088_GYR_SOFTRESET        	0x14
#define BMI088_GYR_INT_CTRL         	0x15
#define BMI088_GYR_INT3_INT4_IO_CONF	0x16
#define BMI088_GYR_INT3_INT4_IO_MAP 	0x18


#define BMI088_ACC_CHIP_ID_VALUE    	0x1E
#define BMI088_GYR_CHIP_ID_VALUE        0x0F

/**\name    SPI read/write mask to configure address */
#define BMI08_SPI_RD_MASK               0x80
#define BMI08_SPI_WR_MASK               0x7F

#define BMI088_ACC_SELECT 				0x00
#define BMI088_GYR_SELECT 				0x01


/* Register 0x1B: PWR_CTRL - Power Control */



/*
 *	SENSOR STRUCT
 */

typedef struct {

	/* SPI HANDLE */
	SPI_HandleTypeDef *spiHandle;

	/* Acceleration data (X,Y,Z) in m/s^2 */
	float acc_mps2[3];

	/* Gyroscope data (X,Y,Z) in rad/s */
	float gyr_rdps[3];

	float offset[6];

	GPIO_TypeDef *acc_cs_port;
	uint16_t acc_cs_pin;
	GPIO_TypeDef *gyr_cs_port;
	uint16_t gyr_cs_pin;

}BMI088;




/*
 * 	INITIALISATION
 */

uint8_t BMI088_Init(BMI088 *dev, SPI_HandleTypeDef *spiHandle);



HAL_StatusTypeDef BMI088_OffsetCorrection(BMI088 *dev);

/*
 * READ DATA
 */

HAL_StatusTypeDef BMI088_ReadAcceleration(BMI088 *dev);
HAL_StatusTypeDef BMI088_ReadAngularRate(BMI088 *dev);




/*
 * LOW-LEVEL FUNTIONS
 */

HAL_StatusTypeDef BMI088_ReadRegister(BMI088 *dev, uint8_t reg, uint8_t *data, uint8_t select);
HAL_StatusTypeDef BMI088_ReadRegisters(BMI088 *dev, uint8_t reg, uint8_t *data, uint8_t length, uint8_t select);

//HAL_StatusTypeDef BMI088_WriteRegister(BMI088 *dev, uint8_t *data, uint16_t length);
HAL_StatusTypeDef BMI088_WriteRegister(BMI088 *dev, uint8_t reg, uint8_t value, uint8_t select);


#endif /* BMI088_REGISTERS_H */



