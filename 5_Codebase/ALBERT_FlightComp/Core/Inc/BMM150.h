/*
 *
 * BMM150 I2C Driver
 *
 * Author: Felix Zauner
 * Created: 22.01.2025
 *
 */

#ifndef BMM150_I2C_DRIVER_H   	/* Header guard: prevents multiple inclusion of this header file */
#define BMM150_I2C_DRIVER_H


#include "stm32f4xx_hal.h"			/* Hardware Abstraction Layer needed for I"C */



/*
 * 	DEFINES
 */

#define BMM150_I2C_ADDR	(0x10 << 1)	/* Define I2C Adress left shift because stm32 ist 7bit */

// Chip ID value
#define CHIP_ID_VALUE     0x32

// Register Address Definitions
#define CHIP_ID           0x40
#define DATA_X_LSB        0x42
#define DATA_X_MSB        0x43
#define DATA_Y_LSB        0x44
#define DATA_Y_MSB        0x45
#define DATA_Z_LSB        0x46
#define DATA_Z_MSB        0x47
#define RHALL_LSB         0x48
#define RHALL_MSB         0x49
#define INTERRUPT		  0x4A
#define CONFIG_1          0x4B
#define CONFIG_2          0x4C
#define CONFIG_3          0x4D
#define CONFIG_4	      0x4E
#define LOW_THRESHOLD     0x4F
#define HIGH_THRESHOLD    0x50
#define REPXY             0x51
#define REPZ              0x52

#define GET_BITS(data, mask, pos) (((data) & (mask)) >> (pos))

// Bit-Definitionen
#define DATA_X_MASK    0x1F
#define DATA_X_POS     0
#define DATA_Y_MASK    0x1F
#define DATA_Y_POS     0
#define DATA_Z_MASK    0xFE  // Bits 7:1 (ohne SelfTestZ)
#define DATA_Z_POS     1     // Shift rechts um 1
#define DATA_RHALL_MASK 0x3F
#define DATA_RHALL_POS  0


/*
 *	SENSOR STRUCT
 */

typedef struct {

	/* I2C HANDLE */
	I2C_HandleTypeDef *i2cHandle;

	/* Magnetometer Acceleration data (X,Y,Z) in m/s^2 */
	float mag_uT[3];


}BMM150;




/*
 * 	INITIALISATION
 */

uint8_t BMM150_Init(BMM150 *dev, I2C_HandleTypeDef *i2cHandle);





/*
 * READ DATA
 */

HAL_StatusTypeDef BMM150_ReadMagneticField(BMM150 *dev);





/*
 * LOW-LEVEL FUNTIONS
 */

HAL_StatusTypeDef BMM150_ReadRegister(BMM150 *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef BMM150_ReadRegisters(BMM150 *dev, uint8_t reg, uint8_t *data, uint8_t length );

HAL_StatusTypeDef BMM150_WriteRegister(BMM150 *dev, uint8_t reg, uint8_t *data);

#endif


