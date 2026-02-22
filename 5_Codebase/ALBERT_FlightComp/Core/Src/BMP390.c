/*
 *
 * BMP390 SPI Driver
 *
 * Author: Felix Zauner
 * Created: 22.01.2025
 *
 */

#include "main.h"
#include "BMP390.h"





/*
 * 	Initialise Sensor
 */

uint8_t BMP390_Init(BMP390 *dev, SPI_HandleTypeDef *spiHandle){

	/* Set struct parameters */
	dev->spiHandle	= spiHandle;

	dev->prs_Pa		=	0.0f;
	dev->cs_port	=	SPI2_CS_NAV_GPIO_Port;
	dev->cs_pin		= 	SPI2_CS_NAV_Pin;


	/* Transaction Errors storage */
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;
	uint8_t regData = 0;


	HAL_Delay(3);

	/*
	 * Select SPI Mode
	 */
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

	HAL_Delay(5);

	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
	HAL_Delay(1);

	/*
	 * Check Device ID
	 */
	 status = BMP390_ReadRegister(dev, BMP390_CHIP_ID, &regData);
	 errNum += ( status != HAL_OK);

	 if ( regData != BMP390_CHIP_ID_VALUE ){

		 return HAL_ERROR;

	 }


	// OSR: 1 pressure a temperatures (ultra low power)
	status = BMP390_WriteRegister(dev, BMP390_OSR, 0x00);
	errNum += (status != HAL_OK);

	// ODR: 10Hz (prescaler 16)
	status = BMP390_WriteRegister(dev, BMP390_ODR, 0x04);
	errNum += (status != HAL_OK);

	// IIR-Filter: aus
	status = BMP390_WriteRegister(dev, BMP390_CONFIG, 0x00);
	errNum += (status != HAL_OK);

	// --- BMP390: DRDY -> INT pin (PC7) ---
	status = BMP390_WriteRegister(dev, BMP390_INT_CTRL, 0x62); // push-pull, active-high, drdy_en=1, strong drive
	errNum += (status != HAL_OK);
	HAL_Delay(2);


	// PWR_CTRL: Normal mode, Temperatur und Druck aktiviert
	status = BMP390_WriteRegister(dev, BMP390_PWR_CTRL, 0x33);  // 0b00110011
	errNum += (status != HAL_OK);

	HAL_Delay(50);  // Warten bis erste Messung verfügbar

	BMP390_ReadCalibrationData(dev);

    return errNum;
}





/*
 * READ DATA
 */
HAL_StatusTypeDef BMP390_ReadPressure(BMP390 *dev) {
    uint8_t regData[6];
    uint32_t uncomp_press, uncomp_temp;

    // Burst-Read aller 6 Datenregister (Druck + Temperatur)
    BMP390_ReadRegisters(dev, BMP390_DATA_0, regData, 6);

    // Extrahiere 24-Bit Rohdaten
    uncomp_press = ((uint32_t)regData[2] << 16) |
                   ((uint32_t)regData[1] << 8) |
                   regData[0];

    uncomp_temp = ((uint32_t)regData[5] << 16) |
                  ((uint32_t)regData[4] << 8) |
                  regData[3];

    // Kompensiere Temperatur (setzt dev->t_lin)
    BMP390_CompensateTemperature(uncomp_temp, dev);

    // Kompensiere Druck mit Temperatur
    dev->prs_Pa = BMP390_CompensatePressure(uncomp_press, dev) ;

    return HAL_OK;
}



float BMP390_CompensateTemperature(uint32_t uncomp_temp, BMP390 *dev) {
    float partial_data1 = (float)uncomp_temp - dev->par_t1;
    float partial_data2 = partial_data1 * dev->par_t2;
    dev->t_lin = partial_data2 + (partial_data1 * partial_data1) * dev->par_t3;
    return dev->t_lin;
}

float BMP390_CompensatePressure(uint32_t uncomp_press, BMP390 *dev) {
    float partial_data1, partial_data2, partial_data3, partial_data4;
    float partial_out1, partial_out2;

    partial_data1 = dev->par_p6 * dev->t_lin;
    partial_data2 = dev->par_p7 * (dev->t_lin * dev->t_lin);
    partial_data3 = dev->par_p8 * (dev->t_lin * dev->t_lin * dev->t_lin);
    partial_out1 = dev->par_p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = dev->par_p2 * dev->t_lin;
    partial_data2 = dev->par_p3 * (dev->t_lin * dev->t_lin);
    partial_data3 = dev->par_p4 * (dev->t_lin * dev->t_lin * dev->t_lin);
    partial_out2 = (float)uncomp_press *
                   (dev->par_p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = (float)uncomp_press * (float)uncomp_press;
    partial_data2 = dev->par_p9 + dev->par_p10 * dev->t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 +
                   ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) *
                   dev->par_p11;

    return partial_out1 + partial_out2 + partial_data4;
}

void BMP390_ReadCalibrationData(BMP390 *dev) {
    uint8_t calib[21];
    BMP390_ReadRegisters(dev, 0x31, calib, 21);

    // Extraktion mit korrekten Indizes (Tabelle 24)
    uint16_t NVM_PAR_T1 = (uint16_t)calib[1] << 8 | calib[0];
    uint16_t NVM_PAR_T2 = (uint16_t)calib[3] << 8 | calib[2];
    int8_t   NVM_PAR_T3 = (int8_t)calib[4];
    int16_t  NVM_PAR_P1 = (int16_t)calib[6] << 8 | calib[5];
    int16_t  NVM_PAR_P2 = (int16_t)calib[8] << 8 | calib[7];
    int8_t   NVM_PAR_P3 = (int8_t)calib[9];
    int8_t   NVM_PAR_P4 = (int8_t)calib[10];
    uint16_t NVM_PAR_P5 = (uint16_t)calib[12] << 8 | calib[11];  // Korrigiert
    uint16_t NVM_PAR_P6 = (uint16_t)calib[14] << 8 | calib[13];  // Korrigiert
    int8_t   NVM_PAR_P7 = (int8_t)calib[15];                     // Korrigiert
    int8_t   NVM_PAR_P8 = (int8_t)calib[16];                     // Korrigiert
    int16_t  NVM_PAR_P9 = (int16_t)calib[18] << 8 | calib[17];   // Korrigiert
    int8_t   NVM_PAR_P10 = (int8_t)calib[19];                    // Korrigiert
    int8_t   NVM_PAR_P11 = (int8_t)calib[20];                    // Korrigiert

    // Umrechnung in Float (Formeln aus Datenblatt Seite 55)
    dev->par_t1 = (float)NVM_PAR_T1 * 256.0f;                    // 2^-8 → *256
    dev->par_t2 = (float)NVM_PAR_T2 / 1073741824.0f;             // 2^30
    dev->par_t3 = (float)NVM_PAR_T3 / 281474976710656.0f;        // 2^48

    dev->par_p1 = (float)(NVM_PAR_P1 - 16384) / 1048576.0f;      // 2^14, 2^20
    dev->par_p2 = (float)(NVM_PAR_P2 - 16384) / 536870912.0f;    // 2^14, 2^29
    dev->par_p3 = (float)NVM_PAR_P3 / 4294967296.0f;             // 2^32
    dev->par_p4 = (float)NVM_PAR_P4 / 137438953472.0f;           // 2^37
    dev->par_p5 = (float)NVM_PAR_P5 * 8.0f;                      // 2^-3
    dev->par_p6 = (float)NVM_PAR_P6 / 64.0f;                     // 2^6
    dev->par_p7 = (float)NVM_PAR_P7 / 256.0f;                    // 2^8
    dev->par_p8 = (float)NVM_PAR_P8 / 32768.0f;                  // 2^15
    dev->par_p9 = (float)NVM_PAR_P9 / 281474976710656.0f;        // 2^48
    dev->par_p10 = (float)NVM_PAR_P10 / 281474976710656.0f;      // 2^48
    dev->par_p11 = (float)NVM_PAR_P11 / 36893488147419103232.0f; // 2^65
}
/*
 * LOW-LEVEL FUNCTIONS
 */


HAL_StatusTypeDef BMP390_ReadRegister(BMP390 *dev, uint8_t reg, uint8_t *data){
    uint8_t res = HAL_OK;
    uint8_t addr = reg | 0x80;
    uint8_t dummy;

    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);

    if (HAL_SPI_Transmit(dev->spiHandle, &addr, 1, HAL_MAX_DELAY) != HAL_OK){
        res = HAL_ERROR;
    }
    else if (HAL_SPI_Receive(dev->spiHandle, &dummy, 1, HAL_MAX_DELAY) != HAL_OK) {
        res = HAL_ERROR;
    }
    else if (HAL_SPI_Receive(dev->spiHandle, data, 1, HAL_MAX_DELAY) != HAL_OK) {
		res = HAL_ERROR;
	}
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    return res;
}


HAL_StatusTypeDef BMP390_ReadRegisters(BMP390 *dev, uint8_t reg, uint8_t *data, uint8_t length ){
	uint8_t res = HAL_OK;
	uint8_t addr = reg | 0x80;
	uint8_t dummy;

	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);

    if (HAL_SPI_Transmit(dev->spiHandle, &addr, 1, HAL_MAX_DELAY) != HAL_OK){
        res = HAL_ERROR;
    }
    else if (HAL_SPI_Receive(dev->spiHandle, &dummy, 1, HAL_MAX_DELAY) != HAL_OK) {
        res = HAL_ERROR;
    }
    else if (HAL_SPI_Receive(dev->spiHandle, data, length, HAL_MAX_DELAY) != HAL_OK) {
		res = HAL_ERROR;

    }
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

	return res;
}
//
//HAL_StatusTypeDef BMP390_WriteRegister(BMP390 *dev, uint8_t *data, uint16_t length){
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

HAL_StatusTypeDef BMP390_WriteRegister(BMP390 *dev, uint8_t reg, uint8_t value){
    uint8_t res;
    uint8_t tx[2];

    tx[0] = reg & 0x7F;
    tx[1] = value;

    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    res = HAL_SPI_Transmit(dev->spiHandle, tx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

    return res;
}
