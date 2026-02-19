/*
 *
 * BMM150 I2C Driver
 *
 * Author: Felix Zauner
 * Created: 22.01.2025
 *
 */

#include "BMM150.h"



/*
 * 	Initialise Sensor
 */

uint8_t BMM150_Init(BMM150 *dev, I2C_HandleTypeDef *i2cHandle){

	/* Set struct parameters */
	dev->i2cHandle	= i2cHandle;

	dev->mag_uT[0]	=	0.0f;
	dev->mag_uT[1]	=	0.0f;
	dev->mag_uT[2]	=	0.0f;

	/* Transaction Errors storage */
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;
	uint8_t regData = 0;


    /* Power up the sensor from suspend to sleep mode */
	regData = BMM150_ReadRegister(dev, CONFIG_1, &regData);
	regData |= 0x01;		// set power control bit 1
	status = BMM150_WriteRegister(dev, CONFIG_1, &regData);
	errNum += ( status != HAL_OK);

	HAL_Delay(3);

	/*
	 * Check Device ID
	 */
	 status = BMM150_ReadRegister(dev, CHIP_ID, &regData);
	 errNum += ( status != HAL_OK);

	 if ( regData != CHIP_ID_VALUE ){

		 return 255;

	 }
	 /*
	  * Start sensor in normal mode at 10 hz
	  */

	 regData = 0x00; 	// Normal mode 10Hz
	 //regData = 0x38; 	// Fast Mode 30Hz
	 status = BMM150_WriteRegister(dev, CONFIG_2, &regData);
	 errNum += ( status != HAL_OK);

    return errNum;
}





/*
 * Read Magnetic Field
 */

HAL_StatusTypeDef BMM150_ReadMagneticField(BMM150 *dev){


	int16_t pnRawData[3];
	uint8_t regData[6];
	uint8_t i = 0;


	/* Read angular rate */
	BMM150_ReadRegisters(dev, DATA_X_LSB, regData, 6 );


    // X
    uint8_t x_lsb = GET_BITS(regData[0], DATA_X_MASK, DATA_X_POS);
    int16_t x_msb_scaled = ((int16_t)((int8_t)regData[1])) * 32;
    pnRawData[0] = x_msb_scaled | x_lsb;

    // Y
    uint8_t y_lsb = GET_BITS(regData[2], DATA_Y_MASK, DATA_Y_POS);
    int16_t y_msb_scaled = ((int16_t)((int8_t)regData[3])) * 32;
    pnRawData[1] = y_msb_scaled | y_lsb;

    // Z
    uint8_t z_lsb = GET_BITS(regData[4], DATA_Z_MASK, DATA_Z_POS);
    int16_t z_msb_scaled = ((int16_t)((int8_t)regData[5])) * 128;
    pnRawData[2] = z_msb_scaled | z_lsb;


    /* remap */
	  for(i=0; i<3; i++)
	  {
		dev->mag_uT[i]= pnRawData[i];
	  }

	  return HAL_OK;
}






/*
 * LOW-LEVEL FUNCTIONS
 */


HAL_StatusTypeDef BMM150_ReadRegister(BMM150 *dev, uint8_t reg, uint8_t *data){

	return HAL_I2C_Mem_Read(dev->i2cHandle, BMM150_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);

}

HAL_StatusTypeDef BMM150_ReadRegisters(BMM150 *dev, uint8_t reg, uint8_t *data, uint8_t length ){

	return HAL_I2C_Mem_Read(dev->i2cHandle, BMM150_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);

}

HAL_StatusTypeDef BMM150_WriteRegister(BMM150 *dev, uint8_t reg, uint8_t *data){

	return HAL_I2C_Mem_Write(dev->i2cHandle, BMM150_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);

}
