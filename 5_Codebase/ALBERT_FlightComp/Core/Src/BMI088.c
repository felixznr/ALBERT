/*
 *
 * BMI088 SPI Driver
 *
 * Author: Felix Zauner
 * Created: 22.01.2025
 *
 */

#include "main.h"
#include "BMI088.h"

//
//		88888888888888888888,   `88888888888888888'\ ,888888'\   ,88888888888888888888
//		888888888888888888888,   `888888888888888'  ,888888' /  ,888888888888888888888
//		\                 `888,    \________,888'  ,888'\__\/  ,888'                 /
//		`8888888888888888888888,           ,888'  ,888' /     ,8888888888888888888888'
//		 `8888888888888888888888,         ,888'  ,888' /     ,8888888888888888888888'
//		   \                 `888,       ,888'  ,888' /     ,888'                 /
//		   `8888888888888888888888,     ,888'  ,888' /     ,8888888888888888888888'
//			`8888888888888888888888,   ,888'  ,8888888888888888888888888888888888'
//			  \_________________`888, ,888'  ,888888888888888'\ _______________/
//								 `888.888'  ,888'  /    ,888'  /
//								  `88888'  ,888'  /    ,888'  /
//								   `888'  ,888'  /    ,888'  /
//									`8'  ,888'  /    ,888'  /
//									  \ ,888'  /    ,888'  /
//									   ,888'  /    ,888'  /
//									   `88'  /    ,888'  /
//										`'  /    ,888'  /
//										  `/    ,888'  /
//											   ,888'  /
//											  ,888'  /
//		  ryan.keefer@psybbs.durham.nc        `88'  /
//											   `'  /
//												 `/


/*
 * 	Initialise Sensor
 */

uint8_t BMI088_Init(BMI088 *dev, SPI_HandleTypeDef *spiHandle){

	/* Set struct parameters */
	dev->spiHandle	= spiHandle;

	dev->acc_mps2[0] = 0.0f;
	dev->acc_mps2[1] = 0.0f;
	dev->acc_mps2[2] = 0.0f;
	dev->gyr_rdps[0] = 0.0f;
	dev->gyr_rdps[1] = 0.0f;
	dev->gyr_rdps[2] = 0.0f;

	dev->offset[0]	= 0.0f;
	dev->offset[1]	= 0.0f;
	dev->offset[2]	= 0.0f;
	dev->offset[3]	= 0.0f;
	dev->offset[4]	= 0.0f;
	dev->offset[5]	= 0.0f;

	dev->acc_cs_pin	= SPI1_CS_ACCEL_NAV_Pin;
	dev->acc_cs_port = SPI1_CS_ACCEL_NAV_GPIO_Port;
	dev->gyr_cs_pin = SPI1_CS_GYRO_NAV_Pin;
	dev->gyr_cs_port = SPI1_CS_GYRO_NAV_GPIO_Port;


	/* Transaction Errors storage */
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;
	uint8_t regData = 0;



	/*
	 * ACCELEROMETER INIT
	 */

	/*switch Accelerometer to SPI mode p.13 */
	HAL_GPIO_WritePin(dev->acc_cs_port, dev->acc_cs_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(dev->gyr_cs_port, dev->gyr_cs_pin, GPIO_PIN_SET);

	HAL_Delay(50);

	/* Dummy Read */
	BMI088_ReadRegister(dev, BMI088_ACC_CHIP_ID, &regData, BMI088_ACC_SELECT);

	HAL_Delay(1);


	/*
	 * Check Device ID of the acceleromter
	 */
	 status = BMI088_ReadRegister(dev, BMI088_ACC_CHIP_ID, &regData, BMI088_ACC_SELECT);
	 errNum += ( status != HAL_OK);

	 if ( regData != BMI088_ACC_CHIP_ID_VALUE){

		 return HAL_ERROR;

	 }

	 /* Configure accelerometer LPF bandwidth (OSR4, 1000) and ODR (800 Hz, 0B)*/
	 regData = 0x8B;
	 status = BMI088_WriteRegister(dev,BMI088_ACC_CONF, regData, BMI088_ACC_SELECT);
	 errNum += ( status != HAL_OK);

	 HAL_Delay(1);


	 /* Accelerometer range (+-24G = 0x03) */
	 regData = 0x03;
	 status = BMI088_WriteRegister(dev,BMI088_ACC_RANGE, regData, BMI088_ACC_SELECT);
	 errNum += ( status != HAL_OK);

	 HAL_Delay(1);


	 /* Switch Acceleromter into Normal Mode p. 13*/
	 regData = 0x04;
	 status = BMI088_WriteRegister(dev,BMI088_ACC_PWR_CTRL , regData, BMI088_ACC_SELECT);
	 errNum += ( status != HAL_OK);

	 HAL_Delay(1);

	 // --- ACC: DRDY -> INT1 (PC3) ---
	 regData = 0x0A; // INT1 output enable, push-pull, active-high
	 status = BMI088_WriteRegister(dev, BMI088_INT1_IO_CTRL, regData, BMI088_ACC_SELECT);
	 errNum += (status != HAL_OK);
	 HAL_Delay(1);

	 regData = 0x04; // map ACC data-ready to INT1
	 status = BMI088_WriteRegister(dev, BMI088_INT_MAP_DATA, regData, BMI088_ACC_SELECT);
	 errNum += (status != HAL_OK);
	 HAL_Delay(1);


	 /* Switch accel into active mode*/
	 regData = 0x00;
	 status = BMI088_WriteRegister(dev,BMI088_ACC_PWR_CONF , regData, BMI088_ACC_SELECT);
	 errNum += ( status != HAL_OK);

	 HAL_Delay(1);




	 /*
	  * GYROSCOPE INIT
	  */

	/*
	 * Check Device ID of the gyroscop
	 */
	 status = BMI088_ReadRegister(dev, BMI088_GYR_CHIP_ID, &regData, BMI088_GYR_SELECT);
	 errNum += ( status != HAL_OK);

	 if ( regData != BMI088_GYR_CHIP_ID_VALUE){

		 return HAL_ERROR;

	 }

	 /* Gyro normal mode (0x00) */
	 regData = 0x00;  // Normal mode
	 status = BMI088_WriteRegister(dev, BMI088_GYR_LPM1, regData, BMI088_GYR_SELECT);
	 errNum += (status != HAL_OK);
	 HAL_Delay(1);

	 /* Gyro bandwidth/ODR (116Hz / 1000 Hz) */
	 regData = 0x02;
	 status = BMI088_WriteRegister(dev,BMI088_GYR_BANDWIDTH, regData, BMI088_GYR_SELECT);
	 errNum += ( status != HAL_OK);

	 HAL_Delay(1);


	/* Enable gyro interrupt and map to pins */
	 regData = 0x80;
	 status = BMI088_WriteRegister(dev, BMI088_GYR_INT_CTRL, regData, BMI088_GYR_SELECT);
	 errNum += (status != HAL_OK);
	 HAL_Delay(1);


	 // --- GYR: DRDY -> INT3 (PC4) ---
	 regData = 0x0D; // INT4: default (OD+high), INT3: push-pull + active-high
	 status = BMI088_WriteRegister(dev, BMI088_GYR_INT3_INT4_IO_CONF, regData, BMI088_GYR_SELECT);
	 errNum += (status != HAL_OK);
	 HAL_Delay(1);

	 regData = 0x01; // map GYR data-ready to INT3
	 status = BMI088_WriteRegister(dev, BMI088_GYR_INT3_INT4_IO_MAP, regData, BMI088_GYR_SELECT);
	 errNum += (status != HAL_OK);
	 HAL_Delay(1);


	 /* Gyro range 500 deg /s */
	 regData = 0x1;
	 status = BMI088_WriteRegister(dev,BMI088_GYR_RANGE, regData, BMI088_GYR_SELECT);
	 errNum += ( status != HAL_OK);

	 HAL_Delay(1);




    return errNum;
}


HAL_StatusTypeDef BMI088_OffsetCorrection(BMI088 *dev){

	float sum[6] = {0, 0, 0, 0, 0, 0};

	for (uint16_t i = 0; i < 1000; i++)   // 500 Samples
	{
		BMI088_ReadAngularRate(dev);
		BMI088_ReadAcceleration(dev);

		sum[0] += dev->gyr_rdps[0];
		sum[1] += dev->gyr_rdps[1];
		sum[2] += dev->gyr_rdps[2];

		sum[3] += dev->acc_mps2[0];
		sum[4] += dev->acc_mps2[1];
		sum[5] += dev->acc_mps2[2];

		HAL_Delay(2);
	}


	dev->offset[0] = (float)(sum[2] / 500);
	dev->offset[1] = (float)(sum[1] / 500);
	dev->offset[2] = (float)(sum[0] / 500);


	return HAL_OK;
}


/* legere accelerationis notitia */
HAL_StatusTypeDef BMI088_ReadAcceleration(BMI088 *dev){

	int16_t pnRawData[3];
	uint8_t regData[6];
	uint8_t i = 0;

	BMI088_ReadRegisters(dev, BMI088_ACC_DATA, regData, 6, BMI088_ACC_SELECT);



	for(i=0; i<3; i++)
		{
			pnRawData[i]=((((uint16_t)regData[2*i+1]) << 8) + (uint16_t)regData[2*i]);
		}


	dev->acc_mps2[0] = (float)(pnRawData[0] * 0.00718505824f);
	dev->acc_mps2[1] = -(float)(pnRawData[1] * 0.00718505824f);
	dev->acc_mps2[2] = -(float)(pnRawData[2] * 0.00718505824f);


	  return HAL_OK;

}






/* read se gyroscope data */
HAL_StatusTypeDef BMI088_ReadAngularRate(BMI088 *dev){

	int16_t pnRawData[3];
	uint8_t regData[6];
	uint8_t i = 0;

	BMI088_ReadRegisters(dev, BMI088_GYR_DATA, regData, 6, BMI088_GYR_SELECT);



	for(i=0; i<3; i++)
		{
			pnRawData[i]=((((uint16_t)regData[2*i+1]) << 8) + (uint16_t)regData[2*i]);
		}



		/* Obtain the m/s^2 value for the three axis */
		dev->gyr_rdps[0]= -	( float )(pnRawData[2] * ((float)0.00026632423f)) + dev->offset[0];
		dev->gyr_rdps[1]= -	( float )(pnRawData[1] * ((float)0.00026632423f)) + dev->offset[1];
		dev->gyr_rdps[2]= -	( float )(pnRawData[0] * ((float)0.00026632423f)) + dev->offset[2];




	  return HAL_OK;
}




/*
 * LOW-LEVEL FUNCTIONS
 */


HAL_StatusTypeDef BMI088_ReadRegister(BMI088 *dev, uint8_t reg, uint8_t *data, uint8_t select){
    uint8_t res = HAL_OK;
    uint8_t addr = reg | 0x80; /* MSB = 1 -> Read */
    uint8_t dummy;


    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;

    /* Richtigen CS auswählen */
    if (select == BMI088_ACC_SELECT)
    {
        cs_port = dev->acc_cs_port;
        cs_pin  = dev->acc_cs_pin;
    }
    else
    {
        cs_port = dev->gyr_cs_port;
        cs_pin  = dev->gyr_cs_pin;
    }

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);

    if (HAL_SPI_Transmit(dev->spiHandle, &addr, 1, HAL_MAX_DELAY) != HAL_OK){
        res = HAL_ERROR;
    }
    if(select == BMI088_ACC_SELECT){ /* Dummy Byte */
    	if (HAL_SPI_Receive(dev->spiHandle, &dummy, 1, HAL_MAX_DELAY) != HAL_OK){
    		res = HAL_ERROR;
    	}
    }
    if (HAL_SPI_Receive(dev->spiHandle, data, 1, HAL_MAX_DELAY) != HAL_OK){
    	res = HAL_ERROR;
    }

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    return res;
}


HAL_StatusTypeDef BMI088_ReadRegisters(BMI088 *dev, uint8_t reg, uint8_t *data,uint8_t length, uint8_t select){
    uint8_t res = HAL_OK;
    uint8_t addr = reg | 0x80; /* MSB = 1 -> Read */
    uint8_t dummy;


    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;

    /* Richtigen CS auswählen */
    if (select == BMI088_ACC_SELECT)
    {
        cs_port = dev->acc_cs_port;
        cs_pin  = dev->acc_cs_pin;
    }
    else
    {
        cs_port = dev->gyr_cs_port;
        cs_pin  = dev->gyr_cs_pin;
    }

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);

    if (HAL_SPI_Transmit(dev->spiHandle, &addr, 1, HAL_MAX_DELAY) != HAL_OK){
        res = HAL_ERROR;
    }
    if(select == BMI088_ACC_SELECT){ /* Dummy Byte */
    	if (HAL_SPI_Receive(dev->spiHandle, &dummy, 1, HAL_MAX_DELAY) != HAL_OK){
    		res = HAL_ERROR;
    	}
    }
    if (HAL_SPI_Receive(dev->spiHandle, data, length, HAL_MAX_DELAY) != HAL_OK){
    	res = HAL_ERROR;
    }

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    return res;
}
//
//HAL_StatusTypeDef BMI088_WriteRegister(BMI088 *dev, uint8_t *data, uint16_t length){
//	uint8_t res = HAL_OK;
//
//	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
//
//	res = HAL_SPI_Transmit(dev->spiHandle, data, length, HAL_MAX_DELAY);
//
//	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
//
//	return res;
//}

HAL_StatusTypeDef BMI088_WriteRegister(BMI088 *dev, uint8_t reg, uint8_t value, uint8_t select){
    uint8_t res;
    uint8_t tx[2];

    tx[0] = reg & 0x7F;
    tx[1] = value;



    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;

    /* Richtigen CS auswählen */
    if (select == BMI088_ACC_SELECT)
    {
        cs_port = dev->acc_cs_port;
        cs_pin  = dev->acc_cs_pin;
    }
    else
    {
        cs_port = dev->gyr_cs_port;
        cs_pin  = dev->gyr_cs_pin;
    }

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    res = HAL_SPI_Transmit(dev->spiHandle, tx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    return res;
}


