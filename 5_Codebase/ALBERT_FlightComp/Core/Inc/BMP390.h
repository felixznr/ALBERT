/*
 *
 * BMP390 SPI Driver
 *
 * Author: Felix Zauner
 * Created: 22.01.2025
 *
 */

#ifndef BMP390_SPI_DRIVER_H
#define BMP390_SPI_DRIVER_H


#include "stm32f4xx_hal.h"


/*
 * 	DEFINES
 */



/* Register Addresses */
#define BMP390_CMD              0x7E
#define BMP390_CONFIG           0x1F
#define BMP390_ODR              0x1D
#define BMP390_OSR              0x1C
#define BMP390_PWR_CTRL         0x1B
#define BMP390_IF_CONF          0x1A
#define BMP390_INT_CTRL         0x19
#define BMP390_FIFO_CONFIG_2    0x18
#define BMP390_FIFO_CONFIG_1    0x17
#define BMP390_FIFO_WTM_1       0x16
#define BMP390_FIFO_WTM_0       0x15
#define BMP390_FIFO_DATA        0x14
#define BMP390_FIFO_LENGTH_1    0x13
#define BMP390_FIFO_LENGTH_0    0x12
#define BMP390_INT_STATUS       0x11
#define BMP390_EVENT            0x10
#define BMP390_SENSORTIME_2     0x0E
#define BMP390_SENSORTIME_1     0x0D
#define BMP390_SENSORTIME_0     0x0C
#define BMP390_DATA_5		    0x09
#define BMP390_DATA_4		    0x08
#define BMP390_DATA_3		    0x07
#define BMP390_DATA_2		    0x06
#define BMP390_DATA_1		    0x05
#define BMP390_DATA_0		    0x04
#define BMP390_STATUS		    0x03
#define BMP390_ERR_REG		    0x02
#define BMP390_REV_ID		    0x01
#define BMP390_CHIP_ID		    0x00



#define BMP390_CHIP_ID_VALUE    0x60


/* Register 0x1B: PWR_CTRL - Power Control */


#define BMP390_MODE_NORMAL  = 0x1ADB1




/*
 *	SENSOR STRUCT
 */
typedef struct {
    SPI_HandleTypeDef *spiHandle;
    float prs_Pa;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;

    // Kalibrierungskoeffizienten
    float par_t1, par_t2, par_t3;
    float par_p1, par_p2, par_p3, par_p4, par_p5, par_p6;
    float par_p7, par_p8, par_p9, par_p10, par_p11;
    float t_lin; // Kompensierte Temperatur für Druckberechnung
} BMP390;



/*
 * 	INITIALISATION
 */

uint8_t BMP390_Init(BMP390 *dev, SPI_HandleTypeDef *spiHandle);





/*
 * READ DATA
 */

HAL_StatusTypeDef BMP390_ReadPressure(BMP390 *dev);



float BMP390_CompensateTemperature(uint32_t uncomp_temp, BMP390 *dev);
float BMP390_CompensatePressure(uint32_t uncomp_press, BMP390 *dev);
void BMP390_ReadCalibrationData(BMP390 *dev);

/*
 * LOW-LEVEL FUNTIONS
 */

HAL_StatusTypeDef BMP390_ReadRegister(BMP390 *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef BMP390_ReadRegisters(BMP390 *dev, uint8_t reg, uint8_t *data, uint8_t length);

//HAL_StatusTypeDef BMP390_WriteRegister(BMP390 *dev, uint8_t *data, uint16_t length);
HAL_StatusTypeDef BMP390_WriteRegister(BMP390 *dev, uint8_t reg, uint8_t value);


#endif /* BMP390_REGISTERS_H */



