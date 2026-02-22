/* W25X0XGV.h */
#ifndef INC_W25X0XGV_H_
#define INC_W25X0XGV_H_

#include <stdint.h>
#include "main.h"
#include "stm32h7xx_hal.h"

/* Externs */
extern SPI_HandleTypeDef hspi1;

/* User config */
#define W25N_SPI                hspi1
#define W25N_CS_GPIO_PORT       SPI1_CS_MPU_GPIO_Port
#define W25N_CS_GPIO_PIN        SPI1_CS_MPU_Pin

/* Commands */
#define W25X_CMD_WRITE_STATUS_REG   0x01
#define W25X_CMD_PROG_DATA_LOAD     0x02
#define W25X_CMD_READ_DATA          0x03
#define W25X_CMD_READ_STATUS_REG    0x05
#define W25X_CMD_WRITE_ENABLE       0x06
#define W25X_CMD_PROG_EXECUTE       0x10
#define W25X_CMD_PAGE_DATA_READ     0x13
#define W25X_CMD_JEDEC_ID           0x9F
#define W25X_CMD_BLOCK_ERASE        0xD8
#define W25X_CMD_RESET              0xFF

/* W25M Die select */
#define W25M_CMD_DIE_SELECT         0xC2

/* Status/Config register addresses (per datasheet) */
#define W25X_REG_PROT               0xA0   /* Status/Protection Register-1 */
#define W25X_REG_CFG                0xB0   /* Configuration Register / Status Register-2 */
#define W25X_REG_STAT               0xC0   /* Status Register-3 */

/* Manufacturer / Device IDs */
#define WINBOND_MAN_ID              0xEF
#define W25N01GV_DEV_ID             0xAA21
#define W25M02GV_DEV_ID             0xAB21

/* Geometry */
#define W25N01GV_MAX_PAGE           65535u
#define W25M02GV_MAX_PAGE           131071u
#define W25X_MAX_COLUMN             2048u
#define W25X_PAGES_PER_BLOCK        64u

/* Status Register-3 bits */
#define W25X_SR3_BUSY               (1u << 0)  /* BUSY */
#define W25X_SR3_WEL                (1u << 1)  /* WEL */

/* Return codes */
typedef enum {
    W25X0XGV_OK    = 0,
    W25X0XGV_ERROR = 1,
} W25N_STATUS;

typedef enum {
    W25X_MODEL_W25N01GV = 0,
    W25X_MODEL_W25M02GV = 1
} W25X_MODEL;

/* Public API */
uint8_t  W25X0XGV_begin(void);

uint8_t  W25X0XGV_block_erase(uint32_t page_addr);
uint8_t  W25X0XGV_bulk_erase(void);

uint8_t  W25X0XGV_load_prog_data(const uint8_t *buf, uint32_t data_len);
uint8_t  W25X0XGV_program_execute(uint32_t page_addr);

uint8_t  W25X0XGV_page_data_read(uint32_t page_addr);
uint8_t  W25X0XGV_read(uint8_t *buf, uint32_t data_len);

/* Optional helpers */
uint8_t  W25X0XGV_get_model(W25X_MODEL *model_out);

#endif /* INC_W25X0XGV_H_ */
