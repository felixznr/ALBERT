/*
 *	lsm6dsltr IMU I2C Driver
 *
 *	Author: Felix Zauner
 *	Created: 17.01.2025
 *
 */


/*
 * INCLUDES
 */
#include "LSM6DSLTR.h"

/*
 * 	INITIALISATION
 */

uint8_t LSM6DSLTR_Init(LSM6DSLTR *dev, I2C_HandleTypeDef *i2cHandle){

	/* Set struct parameters */
	dev->i2cHandle	= i2cHandle;

	dev->acc_mps2[0]		=	0.0f;
	dev->acc_mps2[1]		= 	0.0f;
	dev->acc_mps2[2]		= 	0.0f;

	dev->gyr_rdps[0]	=	0.0f;
	dev->gyr_rdps[1]	=	0.0f;
	dev->gyr_rdps[2]	=	0.0f;


	dev->offset[0]		=	0.0f;
	dev->offset[1]		=	0.0f;
	dev->offset[2]		=	0.0f;
	dev->offset[3]		=	0.0f;
	dev->offset[4]		=	0.0f;
	dev->offset[5]		=	0.0f;


	/* Transaction Errors storage */
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;




	/*
	 * Check device ids ... didn't find it in the datasheet used who am i instead
	 */

	 uint8_t regData = 0;

	 status = LSM6DSLTR_ReadRegister(dev, WHO_AM_I, &regData);
	 errNum += ( status != HAL_OK);

	 if ( regData != LSM6DSLTR_I2C_WHO_AM_I_ID ){

		 return 255;

	 }


	 /*
	  * CONFIGURE CONTROLL REGISTERS
	  * see p. 62 CTRL3_C BDU 1, IF_INC 1
	  *
	  */

	 regData = 0x44;
	 status = LSM6DSLTR_WriteRegister(dev, CTRL3_C, &regData);
	 errNum += ( status != HAL_OK);


	 /*
	  * CONFIGURE CONTROLL REGISTERS ACCELEROMETER
	  * see p. 60 CTRL1_XL 208Hz (normal mode), +-2g
	  *
	  */

	 regData = 0x50;
	 status = LSM6DSLTR_WriteRegister(dev, CTRL1_XL, &regData);
	 errNum += ( status != HAL_OK);


	 /*
	  * CONFIGURE CONTROLL REGISTERS GYRO
	  * see p. CTRL2_G 208Hz (normal mode), 245dps
	  */

	 regData = 0x50;
	 status = LSM6DSLTR_WriteRegister(dev, CTRL2_G, &regData);
	 errNum += ( status != HAL_OK);

	 HAL_Delay(50);


	 return errNum;		// If 0 -> success
}



/*
 * INCORRECT
 */

HAL_StatusTypeDef LSM6DSLTR_OffsetCorrection(LSM6DSLTR *dev){

	float sum[6] = {0, 0, 0, 0, 0, 0};

	for (uint16_t i = 0; i < 500; i++)   // 500 Samples
	{
		LSM6DSLTR_ReadAngularRate(dev);
		LSM6DSLTR_ReadAcceleration(dev);

		sum[0] += dev->gyr_rdps[0];
		sum[1] += dev->gyr_rdps[1];
		sum[2] += dev->gyr_rdps[2];

		sum[3] += dev->acc_mps2[0];
		sum[4] += dev->acc_mps2[1];
		sum[5] += dev->acc_mps2[2];

		HAL_Delay(2);
	}

	for (uint8_t i = 0; i < 3; i++)
	{

		if(i == 5){
			dev->offset[i] = (float)(sum[i] / 500) - ((float)9806.65f);
		}else{
			dev->offset[i] = (float)(sum[i] / 500);
		}
	}
	return HAL_OK;
}


/*
 * READ DATA
 */

HAL_StatusTypeDef LSM6DSLTR_ReadAcceleration(LSM6DSLTR *dev){

	int16_t pnRawData[3];
	uint8_t regData[6];
	uint8_t i = 0;


	/* Read linear Acceleration */
	LSM6DSLTR_ReadRegisters(dev, OUTX_L_XL, regData, 6 );


	for(i=0; i<3; i++)
		{
			pnRawData[i]=((((uint16_t)regData[2*i+1]) << 8) + (uint16_t)regData[2*i]);
		}

	/* Obtain the mps2 value for the three axis  + remap */
	dev->acc_mps2[0]= 	-( float )(pnRawData[2] * ((float)0.061f) * ((float)9.80665f/1000)) ;	// sensitivity depends on full scale configuration p.60 +-2g
	dev->acc_mps2[1]= 	( float )(pnRawData[0] * ((float)0.061f) * ((float)9.80665f/1000));	// sensitivity depends on full scale configuration p.60 +-2g
	dev->acc_mps2[2]=	( float )(pnRawData[1] * ((float)0.061f) * ((float)9.80665f/1000));	// sensitivity depends on full scale configuration p.60 +-2g


	  return HAL_OK;
}






HAL_StatusTypeDef LSM6DSLTR_ReadAngularRate(LSM6DSLTR *dev){


	int16_t pnRawData[3];
	uint8_t regData[6];
	uint8_t i = 0;


	/* Read angular rate */
	LSM6DSLTR_ReadRegisters(dev, OUTX_L_G, regData, 6 );


	for(i=0; i<3; i++)
		{
			pnRawData[i]=((((uint16_t)regData[2*i+1]) << 8) + (uint16_t)regData[2*i]);
		}


	/*
	 * Remap & transform to rad/s
	 */
	 dev->gyr_rdps[0]= ( float )(pnRawData[2] * ((float)8.750f) * ((float)0.000017453292519943f));	// sensitivity depends on full scale configuration p.60 245 dps
	 dev->gyr_rdps[1]= 	-( float )(pnRawData[0] * ((float)8.750f) * ((float)0.000017453292519943f));	// sensitivity depends on full scale configuration p.60 245 dps
	 dev->gyr_rdps[2]= -( float )(pnRawData[1] * ((float)8.750f) * ((float)0.000017453292519943f));	// sensitivity depends on full scale configuration p.60 245 dps


	  return HAL_OK;
}




/*
 * LOW-LEVEL FUNCTIONS
 */

HAL_StatusTypeDef LSM6DSLTR_ReadRegister(LSM6DSLTR *dev, uint8_t reg, uint8_t *data){

	return HAL_I2C_Mem_Read(dev->i2cHandle, LSM6DSLTR_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);

}

HAL_StatusTypeDef LSM6DSLTR_ReadRegisters(LSM6DSLTR *dev, uint8_t reg, uint8_t *data, uint8_t length ){

	return HAL_I2C_Mem_Read(dev->i2cHandle, LSM6DSLTR_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);

}

HAL_StatusTypeDef LSM6DSLTR_WriteRegister(LSM6DSLTR *dev, uint8_t reg, uint8_t *data){

	return HAL_I2C_Mem_Write(dev->i2cHandle, LSM6DSLTR_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);

}

