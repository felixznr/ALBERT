#include "rfm95.h"


static void rfm95_select(rfm95_t *dev)
{
    HAL_GPIO_WritePin(dev->nss_port, dev->nss_pin, GPIO_PIN_RESET);
}

static void rfm95_unselect(rfm95_t *dev)
{
    HAL_GPIO_WritePin(dev->nss_port, dev->nss_pin, GPIO_PIN_SET);
}

static bool rfm95_write_reg(rfm95_t *dev, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { reg | 0x80, value };
    HAL_StatusTypeDef status;

    rfm95_select(dev);
    status = HAL_SPI_Transmit(dev->spi, tx, 2, RFM95_SPI_TIMEOUT);
    rfm95_unselect(dev);

    return (status == HAL_OK);
}

static bool rfm95_read_reg(rfm95_t *dev, uint8_t reg, uint8_t *value)
{
    HAL_StatusTypeDef status;
    uint8_t addr = reg & 0x7F;

    rfm95_select(dev);

    status = HAL_SPI_Transmit(dev->spi, &addr, 1, RFM95_SPI_TIMEOUT);
    if (status != HAL_OK) {
        rfm95_unselect(dev);
        return false;
    }

    status = HAL_SPI_Receive(dev->spi, value, 1, RFM95_SPI_TIMEOUT);
    rfm95_unselect(dev);

    return (status == HAL_OK);
}

static bool rfm95_burst_write(rfm95_t *dev, uint8_t reg, const uint8_t *data, uint8_t len)
{
    HAL_StatusTypeDef status;
    uint8_t addr = reg | 0x80;

    rfm95_select(dev);

    status = HAL_SPI_Transmit(dev->spi, &addr, 1, RFM95_SPI_TIMEOUT);
    if (status != HAL_OK) {
        rfm95_unselect(dev);
        return false;
    }

    status = HAL_SPI_Transmit(dev->spi, (uint8_t *)data, len, RFM95_SPI_TIMEOUT);
    rfm95_unselect(dev);

    return (status == HAL_OK);
}

static bool rfm95_burst_read(rfm95_t *dev, uint8_t reg, uint8_t *data, uint8_t len)
{
    HAL_StatusTypeDef status;
    uint8_t addr = reg & 0x7F;

    rfm95_select(dev);

    status = HAL_SPI_Transmit(dev->spi, &addr, 1, RFM95_SPI_TIMEOUT);
    if (status != HAL_OK) {
        rfm95_unselect(dev);
        return false;
    }

    status = HAL_SPI_Receive(dev->spi, data, len, RFM95_SPI_TIMEOUT);
    rfm95_unselect(dev);

    return (status == HAL_OK);
}

bool rfm95_reset(rfm95_t *dev)
{
    HAL_GPIO_WritePin(dev->reset_port, dev->reset_pin, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(dev->reset_port, dev->reset_pin, GPIO_PIN_SET);
    HAL_Delay(10);
    return true;
}

bool rfm95_read_version(rfm95_t *dev, uint8_t *version)
{
    return rfm95_read_reg(dev, RFM95_REG_VERSION, version);
}

bool rfm95_init(rfm95_t *dev)
{
    uint8_t version = 0;

    rfm95_unselect(dev);
    rfm95_reset(dev);

    if (!rfm95_read_version(dev, &version)) {
        return false;
    }

    if (version != RFM95_VERSION_VALUE) {
        return false;
    }

    if (!rfm95_write_reg(dev, RFM95_REG_OP_MODE, RFM95_MODE_SLEEP)) {
        return false;
    }

    if (!rfm95_write_reg(dev, RFM95_REG_OP_MODE, RFM95_LONG_RANGE_MODE | RFM95_MODE_SLEEP)) {
        return false;
    }

    if (!rfm95_write_reg(dev, RFM95_REG_FIFO_TX_BASE_ADDR, 0x00)) {
        return false;
    }

    if (!rfm95_write_reg(dev, RFM95_REG_FIFO_RX_BASE_ADDR, 0x00)) {
        return false;
    }

    if (!rfm95_write_reg(dev, RFM95_REG_LNA, 0x23)) {
        return false;
    }

    if (!rfm95_write_reg(dev, RFM95_REG_MODEM_CONFIG_3, 0x04)) {
        return false;
    }

    return true;
}

bool rfm95_set_frequency(rfm95_t *dev, uint32_t frequency_hz)
{
    uint64_t frf = ((uint64_t)frequency_hz << 19) / 32000000UL;

    if (!rfm95_write_reg(dev, RFM95_REG_FRF_MSB, (uint8_t)(frf >> 16))) return false;
    if (!rfm95_write_reg(dev, RFM95_REG_FRF_MID, (uint8_t)(frf >> 8)))  return false;
    if (!rfm95_write_reg(dev, RFM95_REG_FRF_LSB, (uint8_t)(frf)))       return false;

    return true;
}

bool rfm95_config_lora(rfm95_t *dev, uint8_t sf, uint8_t bw, uint8_t cr, uint8_t power_dbm)
{
    uint8_t mc1 = 0;
    uint8_t mc2 = 0;
    uint8_t pa_config = 0;

    if (sf < 6 || sf > 12) return false;
    if (bw > 9) return false;
    if (cr < 1 || cr > 4) return false;
    if (power_dbm < 2 || power_dbm > 20) return false;

    mc1 = (bw << 4) | (cr << 1) | 0x00;
    mc2 = (sf << 4) | 0x04;

    if (!rfm95_write_reg(dev, RFM95_REG_OP_MODE, RFM95_LONG_RANGE_MODE | RFM95_MODE_STDBY)) return false;

    if (!rfm95_write_reg(dev, RFM95_REG_MODEM_CONFIG_1, mc1)) return false;
    if (!rfm95_write_reg(dev, RFM95_REG_MODEM_CONFIG_2, mc2)) return false;
    if (!rfm95_write_reg(dev, RFM95_REG_SYMB_TIMEOUT_LSB, 0x08)) return false;
    if (!rfm95_write_reg(dev, RFM95_REG_PREAMBLE_MSB, 0x00)) return false;
    if (!rfm95_write_reg(dev, RFM95_REG_PREAMBLE_LSB, 0x08)) return false;

    if (sf >= 11 && bw == 7) {
        if (!rfm95_write_reg(dev, RFM95_REG_MODEM_CONFIG_3, 0x0C)) return false;
    } else {
        if (!rfm95_write_reg(dev, RFM95_REG_MODEM_CONFIG_3, 0x04)) return false;
    }

    if (power_dbm == 20) {
        pa_config = RFM95_PA_BOOST | 0x0F;
        if (!rfm95_write_reg(dev, RFM95_REG_PA_DAC, 0x87)) return false;
    } else {
        pa_config = RFM95_PA_BOOST | (power_dbm - 2);
        if (!rfm95_write_reg(dev, RFM95_REG_PA_DAC, 0x84)) return false;
    }

    if (!rfm95_write_reg(dev, RFM95_REG_PA_CONFIG, pa_config)) return false;

    return true;
}

bool rfm95_send(rfm95_t *dev, const uint8_t *data, uint8_t len)
{
    uint8_t irq = 0;
    uint32_t start = HAL_GetTick();

    if (len == 0 || len > 255) return false;

    if (!rfm95_write_reg(dev, RFM95_REG_OP_MODE, RFM95_LONG_RANGE_MODE | RFM95_MODE_STDBY)) return false;
    if (!rfm95_write_reg(dev, RFM95_REG_FIFO_ADDR_PTR, 0x00)) return false;
    if (!rfm95_write_reg(dev, RFM95_REG_PAYLOAD_LENGTH, len)) return false;
    if (!rfm95_burst_write(dev, RFM95_REG_FIFO, data, len)) return false;

    if (!rfm95_write_reg(dev, RFM95_REG_IRQ_FLAGS, 0xFF)) return false;
    if (!rfm95_write_reg(dev, RFM95_REG_OP_MODE, RFM95_LONG_RANGE_MODE | RFM95_MODE_TX)) return false;

    do {
        if (!rfm95_read_reg(dev, RFM95_REG_IRQ_FLAGS, &irq)) return false;
        if (HAL_GetTick() - start > 1000) return false;
    } while ((irq & RFM95_IRQ_TX_DONE) == 0);

    if (!rfm95_write_reg(dev, RFM95_REG_IRQ_FLAGS, 0xFF)) return false;
    if (!rfm95_write_reg(dev, RFM95_REG_OP_MODE, RFM95_LONG_RANGE_MODE | RFM95_MODE_STDBY)) return false;

    return true;
}

bool rfm95_receive(rfm95_t *dev, uint8_t *data, uint8_t *len, uint32_t timeout_ms)
{
    uint8_t irq = 0;
    uint8_t rx_len = 0;
    uint8_t fifo_addr = 0;
    uint32_t start = HAL_GetTick();

    *len = 0;

    if (!rfm95_write_reg(dev, RFM95_REG_IRQ_FLAGS, 0xFF)) return false;
    if (!rfm95_write_reg(dev, RFM95_REG_OP_MODE, RFM95_LONG_RANGE_MODE | RFM95_MODE_RXSINGLE)) return false;

    do {
        if (!rfm95_read_reg(dev, RFM95_REG_IRQ_FLAGS, &irq)) return false;

        if (irq & RFM95_IRQ_RX_DONE) {
            break;
        }

        if (irq & RFM95_IRQ_RX_TIMEOUT) {
            return true;
        }

        if (HAL_GetTick() - start > timeout_ms) {
            return true;
        }
    } while (1);

    if (irq & RFM95_IRQ_PAYLOAD_CRC_ERROR) {
        if (!rfm95_write_reg(dev, RFM95_REG_IRQ_FLAGS, 0xFF)) return false;
        return true;
    }

    if (!rfm95_read_reg(dev, RFM95_REG_FIFO_RX_CURRENT_ADDR, &fifo_addr)) return false;
    if (!rfm95_read_reg(dev, RFM95_REG_RX_NB_BYTES, &rx_len)) return false;
    if (!rfm95_write_reg(dev, RFM95_REG_FIFO_ADDR_PTR, fifo_addr)) return false;
    if (!rfm95_burst_read(dev, RFM95_REG_FIFO, data, rx_len)) return false;

    *len = rx_len;

    if (!rfm95_write_reg(dev, RFM95_REG_IRQ_FLAGS, 0xFF)) return false;
    if (!rfm95_write_reg(dev, RFM95_REG_OP_MODE, RFM95_LONG_RANGE_MODE | RFM95_MODE_STDBY)) return false;

    return true;
}
