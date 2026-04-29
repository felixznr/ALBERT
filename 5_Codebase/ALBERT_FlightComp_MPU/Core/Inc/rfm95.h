#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#if defined(STM32H743xx) || defined(STM32H750xx) || defined(STM32H753xx) || \
    defined(STM32H755xx) || defined(STM32H757xx) || defined(STM32H7A3xx) || \
    defined(STM32H7B3xx) || defined(STM32H7B0xx)
#include "stm32h7xx_hal.h"
#elif defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F411xE) || \
      defined(STM32F446xx)
#include "stm32f4xx_hal.h"
#else
#error Unsupported STM32 family
#endif

#ifndef RFM95_SPI_TIMEOUT
#define RFM95_SPI_TIMEOUT 100
#endif


#define RFM95_REG_FIFO                    0x00
#define RFM95_REG_OP_MODE                 0x01
#define RFM95_REG_FRF_MSB                 0x06
#define RFM95_REG_FRF_MID                 0x07
#define RFM95_REG_FRF_LSB                 0x08
#define RFM95_REG_PA_CONFIG               0x09
#define RFM95_REG_LNA                     0x0C
#define RFM95_REG_FIFO_ADDR_PTR           0x0D
#define RFM95_REG_FIFO_TX_BASE_ADDR       0x0E
#define RFM95_REG_FIFO_RX_BASE_ADDR       0x0F
#define RFM95_REG_FIFO_RX_CURRENT_ADDR    0x10
#define RFM95_REG_IRQ_FLAGS               0x12
#define RFM95_REG_RX_NB_BYTES             0x13
#define RFM95_REG_PKT_SNR_VALUE           0x19
#define RFM95_REG_PKT_RSSI_VALUE          0x1A
#define RFM95_REG_MODEM_CONFIG_1          0x1D
#define RFM95_REG_MODEM_CONFIG_2          0x1E
#define RFM95_REG_SYMB_TIMEOUT_LSB        0x1F
#define RFM95_REG_PREAMBLE_MSB            0x20
#define RFM95_REG_PREAMBLE_LSB            0x21
#define RFM95_REG_PAYLOAD_LENGTH          0x22
#define RFM95_REG_MODEM_CONFIG_3          0x26
#define RFM95_REG_DIO_MAPPING_1           0x40
#define RFM95_REG_VERSION                 0x42
#define RFM95_REG_PA_DAC                  0x4D

#define RFM95_MODE_SLEEP                  0x00
#define RFM95_MODE_STDBY                  0x01
#define RFM95_MODE_TX                     0x03
#define RFM95_MODE_RXCONTINUOUS           0x05
#define RFM95_MODE_RXSINGLE               0x06
#define RFM95_LONG_RANGE_MODE             0x80

#define RFM95_IRQ_RX_DONE                 0x40
#define RFM95_IRQ_TX_DONE                 0x08
#define RFM95_IRQ_PAYLOAD_CRC_ERROR       0x20
#define RFM95_IRQ_RX_TIMEOUT              0x80

#define RFM95_PA_BOOST                    0x80
#define RFM95_VERSION_VALUE               0x12


typedef struct
{
    SPI_HandleTypeDef *spi;

    GPIO_TypeDef *nss_port;
    uint16_t nss_pin;

    GPIO_TypeDef *reset_port;
    uint16_t reset_pin;

} rfm95_t;

bool rfm95_init(rfm95_t *dev);
bool rfm95_reset(rfm95_t *dev);
bool rfm95_read_version(rfm95_t *dev, uint8_t *version);

bool rfm95_set_frequency(rfm95_t *dev, uint32_t frequency_hz);
bool rfm95_config_lora(rfm95_t *dev, uint8_t sf, uint8_t bw, uint8_t cr, uint8_t power_dbm);

bool rfm95_send(rfm95_t *dev, const uint8_t *data, uint8_t len);
bool rfm95_receive(rfm95_t *dev, uint8_t *data, uint8_t *len, uint32_t timeout_ms);
