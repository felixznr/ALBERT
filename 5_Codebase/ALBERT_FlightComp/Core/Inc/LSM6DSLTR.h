/*
 *	lsm6dsltr IMU I2C Driver
 *
 *	Author: Felix Zauner
 *	Created: 17.01.2025
 *
 */

#ifndef LSM6DSLTR_I2C_DRIVER_H   	/* Header guard: prevents multiple inclusion of this header file */
#define LSM6DSLTR_I2C_DRIVER_H


#include "stm32f4xx_hal.h"			/* Hardware Abstraction Layer needed for I"C */



/*
 * 	DEFINES
 */

#define LSM6DSLTR_I2C_ADDR	(0x6B << 1)	/* Define I2C Adress left shift because stm32 ist 7bit */



#define LSM6DSLTR_I2C_WHO_AM_I_ID	  0x6A

/* Reserved Registers */
#define RESERVED_00                   0x00
#define RESERVED_02                   0x02
#define RESERVED_03                   0x03
#define RESERVED_0C                   0x0C
#define RESERVED_1F                   0x1F
#define RESERVED_43                   0x43  /* Range 0x43-0x48 */
#define RESERVED_56                   0x56  /* Range 0x56-0x57 */
#define RESERVED_62                   0x62  /* Range 0x62-0x65 */
#define RESERVED_6C                   0x6C  /* Range 0x6C-0x72 */
#define RESERVED_76                   0x76  /* Range 0x76-0x7F */

/* Configuration & Control Registers */
#define FUNC_CFG_ACCESS               0x01  /* Embedded functions config */
#define SENSOR_SYNC_TIME_FRAME        0x04  /* Sensor sync config */
#define SENSOR_SYNC_RES_RATIO         0x05  /* Sensor sync resolution ratio */

/* FIFO Control Registers */
#define FIFO_CTRL1                    0x06
#define FIFO_CTRL2                    0x07
#define FIFO_CTRL3                    0x08
#define FIFO_CTRL4                    0x09
#define FIFO_CTRL5                    0x0A

/* Data Ready & Interrupt Control */
#define DRDY_PULSE_CFG_G              0x0B  /* Data ready pulse config for gyro */
#define INT1_CTRL                     0x0D  /* INT1 pin control */
#define INT2_CTRL                     0x0E  /* INT2 pin control */
#define WHO_AM_I                      0x0F  /* Device identification */

/* Accelerometer and Gyroscope Control Registers */
#define CTRL1_XL                      0x10  /* Accel control */
#define CTRL2_G                       0x11  /* Gyro control */
#define CTRL3_C                       0x12  /* Control register 3 */
#define CTRL4_C                       0x13  /* Control register 4 */
#define CTRL5_C                       0x14  /* Control register 5 */
#define CTRL6_C                       0x15  /* Control register 6 */
#define CTRL7_G                       0x16  /* Gyro control 7 */
#define CTRL8_XL                      0x17  /* Accel control 8 */
#define CTRL9_XL                      0x18  /* Accel control 9 */
#define CTRL10_C                      0x19  /* Control register 10 */

/* I2C Master & Interrupt Source Registers */
#define MASTER_CONFIG                 0x1A  /* I2C master config */
#define WAKE_UP_SRC                   0x1B  /* Wake-up interrupt source */
#define TAP_SRC                       0x1C  /* Tap interrupt source */
#define D6D_SRC                       0x1D  /* 6D orientation source */
#define STATUS_REG                    0x1E  /* Status data register */

/* Temperature Output */
#define OUT_TEMP_L                    0x20  /* Temperature low byte */
#define OUT_TEMP_H                    0x21  /* Temperature high byte */

/* Gyroscope Output Registers */
#define OUTX_L_G                      0x22  /* Gyro X low byte */
#define OUTX_H_G                      0x23  /* Gyro X high byte */
#define OUTY_L_G                      0x24  /* Gyro Y low byte */
#define OUTY_H_G                      0x25  /* Gyro Y high byte */
#define OUTZ_L_G                      0x26  /* Gyro Z low byte */
#define OUTZ_H_G                      0x27  /* Gyro Z high byte */

/* Accelerometer Output Registers */
#define OUTX_L_XL                     0x28  /* Accel X low byte */
#define OUTX_H_XL                     0x29  /* Accel X high byte */
#define OUTY_L_XL                     0x2A  /* Accel Y low byte */
#define OUTY_H_XL                     0x2B  /* Accel Y high byte */
#define OUTZ_L_XL                     0x2C  /* Accel Z low byte */
#define OUTZ_H_XL                     0x2D  /* Accel Z high byte */

/* Sensor Hub Output Registers */
#define SENSORHUB1_REG                0x2E
#define SENSORHUB2_REG                0x2F
#define SENSORHUB3_REG                0x30
#define SENSORHUB4_REG                0x31
#define SENSORHUB5_REG                0x32
#define SENSORHUB6_REG                0x33
#define SENSORHUB7_REG                0x34
#define SENSORHUB8_REG                0x35
#define SENSORHUB9_REG                0x36
#define SENSORHUB10_REG               0x37
#define SENSORHUB11_REG               0x38
#define SENSORHUB12_REG               0x39
#define SENSORHUB13_REG               0x4D
#define SENSORHUB14_REG               0x4E
#define SENSORHUB15_REG               0x4F
#define SENSORHUB16_REG               0x50
#define SENSORHUB17_REG               0x51
#define SENSORHUB18_REG               0x52

/* FIFO Status & Data Output */
#define FIFO_STATUS1                  0x3A
#define FIFO_STATUS2                  0x3B
#define FIFO_STATUS3                  0x3C
#define FIFO_STATUS4                  0x3D
#define FIFO_DATA_OUT_L               0x3E  /* FIFO data low byte */
#define FIFO_DATA_OUT_H               0x3F  /* FIFO data high byte */

/* Timestamp Registers */
#define TIMESTAMP0_REG                0x40
#define TIMESTAMP1_REG                0x41
#define TIMESTAMP2_REG                0x42

/* Step Counter Registers */
#define STEP_TIMESTAMP_L              0x49  /* Step timestamp low */
#define STEP_TIMESTAMP_H              0x4A  /* Step timestamp high */
#define STEP_COUNTER_L                0x4B  /* Step count low */
#define STEP_COUNTER_H                0x4C  /* Step count high */

/* Interrupt Function Sources */
#define FUNC_SRC1                     0x53  /* Interrupt source 1 */
#define FUNC_SRC2                     0x54  /* Interrupt source 2 */
#define WRIST_TILT_IA                 0x55  /* Wrist tilt interrupt */

/* Tap & Wake-up Configuration */
#define TAP_CFG                       0x58  /* Tap configuration */
#define TAP_THS_6D                    0x59  /* Tap threshold & 6D settings */
#define INT_DUR2                      0x5A  /* Interrupt duration 2 */
#define WAKE_UP_THS                   0x5B  /* Wake-up threshold */
#define WAKE_UP_DUR                   0x5C  /* Wake-up duration */
#define FREE_FALL                     0x5D  /* Free-fall settings */

/* Interrupt Routing */
#define MD1_CFG                       0x5E  /* Interrupt routing to INT1 */
#define MD2_CFG                       0x5F  /* Interrupt routing to INT2 */

/* Master Command & SPI Error */
#define MASTER_CMD_CODE               0x60  /* I2C master command */
#define SENS_SYNC_SPI_ERROR_CODE      0x61  /* Sensor sync SPI error code */

/* Magnetometer Raw Output */
#define OUT_MAG_RAW_X_L               0x66  /* Mag X low byte */
#define OUT_MAG_RAW_X_H               0x67  /* Mag X high byte */
#define OUT_MAG_RAW_Y_L               0x68  /* Mag Y low byte */
#define OUT_MAG_RAW_Y_H               0x69  /* Mag Y high byte */
#define OUT_MAG_RAW_Z_L               0x6A  /* Mag Z low byte */
#define OUT_MAG_RAW_Z_H               0x6B  /* Mag Z high byte */

/* User Offset Registers */
#define X_OFS_USR                     0x73  /* User X offset */
#define Y_OFS_USR                     0x74  /* User Y offset */
#define Z_OFS_USR                     0x75  /* User Z offset */







/*
 * 	SENSOR STRUCT
 */

typedef struct {

	/* I2C HANDLE */
	I2C_HandleTypeDef *i2cHandle;

	/* Accelerometer Acceleration data (X,Y,Z) in m/s^2 */
	float acc_mps2[3];

	/* Gyro Angular rate data (X,Y,Z) in dps */
	float gyr_rdps[3];

	/* Offset Correction */
	float offset[6];

}LSM6DSLTR;





/*
 * 	INITIALISATION
 */

uint8_t LSM6DSLTR_Init(LSM6DSLTR *dev, I2C_HandleTypeDef *i2cHandle);





/*
 * READ DATA
 */

HAL_StatusTypeDef LSM6DSLTR_ReadAcceleration(LSM6DSLTR *dev);
HAL_StatusTypeDef LSM6DSLTR_ReadAngularRate(LSM6DSLTR *dev);


/*
 * OFFSET CORRECTION
 */
HAL_StatusTypeDef LSM6DSLTR_OffsetCorrection(LSM6DSLTR *dev);


/*
 * LOW-LEVEL FUNTIONS
 */

HAL_StatusTypeDef LSM6DSLTR_ReadRegister(LSM6DSLTR *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef LSM6DSLTR_ReadRegisters(LSM6DSLTR *dev, uint8_t reg, uint8_t *data, uint8_t length );

HAL_StatusTypeDef LSM6DSLTR_WriteRegister(LSM6DSLTR *dev, uint8_t reg, uint8_t *data);

#endif

