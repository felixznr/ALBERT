/* W25X0XGV.c */
#include "W25X0XGV.h"

//
//⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢿⣿⣿⣿⣿⣿⣿⢸⣿⣿⣇⢘⣿⡿⢹⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
//⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡞⣿⡟⠻⣿⣿⠟⠁⣿⡿⠉⠸⣿⡗⡨⣿⣿⠟⣿⢹⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠿⠿⠿⠿⠿⠿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
//⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡎⣿⣦⠸⣷⠀⢹⡟⡖⠀⠸⢡⢆⠀⡟⡬⠁⢻⢏⢰⠃⣿⢻⣿⣿⣿⣿⣿⣿⣿⣿⡿⠟⡟⡙⢢⠀⡤⢠⠀⠀⠀⢀⣀⡀⠉⠙⠻⣿⣿⣿⣿⣿⣿
//⣿⣿⣿⣿⣿⣿⣿⣿⣿⣯⣿⢾⣿⣳⣧⣧⡀⣆⠃⣟⣵⢂⢸⠟⢇⢰⡧⢅⠘⡞⠈⡞⠁⣹⣿⣿⣿⣿⣿⣿⣿⠋⠀⠀⢸⠃⣜⣠⠃⣼⡤⢤⣀⡸⢰⣃⣀⣀⠀⠀⠹⣿⣿⣿⣿
//⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠓⣿⣮⠇⣧⢫⢊⣾⡌⡂⢨⣏⠃⢀⡼⠁⡼⢡⢈⣿⣿⣿⣿⣿⣿⣿⣿⡀⠀⠀⡸⠀⡤⢤⠀⠁⡔⣳⢸⡇⡼⢡⠶⣈⡇⠀⢀⣿⣿⣿⣿
//⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡆⣿⡧⠸⡿⠆⠾⠿⠷⠀⠿⠿⢤⣾⢁⣼⢀⠀⣼⣿⣿⣿⣿⣿⣿⣿⣿⣷⣤⡀⣇⣸⠁⣏⣰⢄⣙⣡⢾⣀⡷⣄⡛⣋⢇⣠⣾⣿⣿⣿⣿
//⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣾⠚⣋⡛⣻⣿⣿⣿⣿⣿⣾⣾⣿⣶⣶⣤⡌⣼⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣶⣤⣤⣄⣀⣀⣀⣀⣠⣤⣤⣶⣿⣿⣿⣿⣿⣿⣿⣿
//⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡟⣿⣿⠿⠛⠀⠉⠛⢿⣿⡏⠹⣟⢿⣿⡙⣿⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
//⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡟⢸⣿⡏⠀⠀⠀⠀⠀⠈⠻⣿⡀⠘⣧⢹⡄⠉⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣽⣿⣿⣿⣽⣿⣿⣿⣾⣧⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
//⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡟⣤⢻⣿⠀⠀⠀⠀⠀⠀⠀⠀⢻⣇⢧⢻⣧⣿⣆⠘⣌⢻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
//⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⡿⣸⢟⡄⠀⠀⠀⠀⠀⠀⠀⠀⢻⣈⣧⣮⣻⣟⢷⣾⡎⣿⣿⣿⣿⣿⣯⣭⣿⣿⣿⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
//⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣧⣿⣦⡤⣤⣤⣄⣀⠀⠀⠀⠚⠋⣏⣁⣦⡈⢯⢷⣽⣿⣿⣾⣿⣯⣁⡀⣩⣽⣿⡿⠛⢾⣿⣿⣿⣿⣿⣿⣿⣿⣍⣻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
//⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢿⣇⣠⣤⣄⣮⢿⣦⡀⠀⠀⢠⣞⣿⡟⢿⣿⣾⣿⣿⣿⣿⣿⣿⣿⣿⡇⢀⣈⣻⣿⣿⣿⣿⣽⣿⣿⡾⢻⣿⣿⣿⣿⣿⣿⣿⣇⢿⣿⣿⣿⣿⣿
//⣿⣿⣿⣿⣿⣿⡿⠟⠛⠋⣹⣿⣿⣿⣿⠛⢳⣌⣹⣿⡇⠀⠀⠀⠈⠻⣾⡆⠈⠉⠹⣿⣿⣿⣿⣿⣍⡁⠀⣿⣿⣿⣿⣿⣿⣿⡻⣿⣿⣽⣼⣏⣽⢿⣿⣿⠟⠿⣿⣿⣿⢿⣷⣿⣿
//⣿⣿⣿⣿⣿⠏⠊⣀⣬⣽⡿⠃⣿⡟⠉⠙⠋⢁⣿⣿⠁⠀⠀⠀⠀⠀⠈⠁⠀⠀⢰⣿⣿⣿⢿⣯⣻⣯⠆⠘⣿⣯⣾⣿⣯⣿⣿⣾⣧⡟⢃⣟⣧⢼⡏⣿⡗⢾⣟⣿⣿⣿⣻⣿⣿
//⣿⣿⣿⠟⠒⣠⣴⣿⣿⡿⠃⣸⣿⣿⠀⠀⠀⣾⣿⣿⣧⣄⠒⠂⠀⠠⠀⠀⠀⢠⣾⡿⣿⡈⢸⡟⠋⠉⠀⡀⣽⣿⣿⣿⡾⢿⡅⠙⣏⣀⡾⣯⡟⠸⡟⠃⠸⣇⠾⠿⣿⣿⣿⢿⠿
//⣿⠏⠀⠀⠼⠿⢿⠛⣩⠄⣠⣿⣿⣿⣦⠂⠀⢈⣻⣿⣯⢬⣆⣀⣀⡀⠀⠀⣠⢟⣿⣿⢸⠷⣌⡛⠒⢀⣤⣌⣼⣿⣟⣱⣇⡴⠆⠀⢹⣨⣷⣿⣹⣿⡀⠠⢈⣿⣷⡀⣼⡿⠉⠀⣴
//⣿⡗⠀⠀⠀⠩⠿⠚⣡⣾⣿⣿⣿⣿⣷⠀⠀⣿⣯⣾⠻⠛⠉⠁⠉⠻⣆⡜⣱⠻⣿⣿⣹⡀⢁⠀⠀⡨⣿⠿⠷⣿⣿⣿⣿⣷⣶⡴⠋⡵⣾⣿⣿⠟⠒⠒⢆⠿⢹⡇⣟⠣⠀⢶⡿
//⣿⣇⣀⣀⣠⠄⠒⠛⠛⠿⣿⣿⣿⣿⣿⡆⠸⡹⣿⣿⣶⠶⠶⠿⠛⢿⡿⢠⣿⣣⣿⣿⣿⣍⠉⠡⣌⣺⡩⠈⠀⠀⠄⠋⠫⣍⢟⣄⣾⡴⣿⡿⠯⠒⢐⡋⠇⠀⠀⢷⡸⠆⠐⣀⠁
//⣿⣿⣿⠶⠶⠂⠀⠀⠀⣒⣯⣿⣿⣿⣿⣿⡄⢳⡽⣟⠀⠀⢀⡤⣠⡞⠐⣾⣿⣿⣿⣆⢈⣻⢿⠽⠾⢧⡷⣞⢆⣴⣚⣄⣰⠱⣿⣿⢉⡩⠟⠀⠄⠀⢊⡁⠠⢄⣷⡼⠟⠘⠃⠄⠐
//⣿⣿⣿⣷⣤⣤⣶⡾⢃⣩⣭⣬⣿⣿⣿⣿⣿⡄⣿⣮⣓⡦⠯⡿⠁⠀⣼⣿⣿⣿⣯⠁⢰⣿⣋⠀⠀⠀⠉⢺⡢⣀⣈⠙⡿⢠⣿⣿⣿⠾⠃⠂⠀⠀⡜⠀⢠⣩⢞⣀⡀⠀⠀⠠⠀
//⡿⢿⣿⣛⡛⣩⠎⠔⠉⠙⠛⡿⣿⣿⣿⣿⣿⣷⣿⢿⡿⠛⠃⠁⠀⠀⢻⣧⣻⣿⡇⣔⠋⠁⠈⢿⣦⠀⠀⠻⣿⣾⣽⣾⣷⠿⢾⣿⣤⢲⣄⡀⠀⠀⣽⣴⡟⢹⣫⢟⡉⣀⡀⠀⠀
//⡧⣼⣿⣿⠈⠁⠀⠀⢀⣴⣿⣿⣿⣿⣿⣿⣿⣿⣿⣆⠁⠀⠀⠀⠀⠀⠀⣩⡿⠋⠀⠈⡙⡟⣲⠿⢼⡄⠀⠀⠀⠈⢻⣿⣿⣵⡿⢟⣽⠿⢛⣿⣿⣷⡾⠟⠒⡫⠕⡫⢞⣍⢀⣀⣤
//⢸⣿⣾⠧⢿⣿⣀⡴⡾⢋⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣤⣀⣤⣤⣤⣾⣿⠃⣀⡀⠀⠳⡇⠉⣿⣷⠆⣀⣠⠤⠴⠚⠀⠚⠛⠚⠀⠒⠒⠚⠛⢿⢧⣤⢤⣶⠶⢊⣡⣾⣯⣤⣭⣛
//⢸⣿⣿⣾⣿⣿⣿⢰⢡⣿⣿⡿⡿⣿⣿⡿⣿⣿⣿⣿⠿⠉⠁⠀⠀⠈⠙⢿⢀⡞⠉⢓⡞⡠⣾⡿⠛⠉⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢰⣡⠔⣣⢞⣿⠟⣻⣿⣿⣿⣿
//⣿⣿⣿⣿⣿⣿⣿⢸⡾⣿⣿⡀⠃⣟⣿⣿⣿⣿⣿⠃⠀⠀⠀⠀⠀⠀⠀⢡⣧⣀⣀⣼⣿⡿⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠱⢿⣿⡿⢋⣼⣿⣿⣿⣿⣿
//⣿⣿⣿⣿⣿⣿⡻⢧⣧⠹⣧⡱⡴⡹⡿⣿⣿⣿⣿⣦⠀⠀⠀⠀⠀⠀⠀⠀⣹⣯⣾⣿⠛⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⡀⠀⠀⠀⠀⢙⣤⣿⣿⣿⣿⣿⣿⣿
//⣿⣿⠛⠻⠻⢷⡟⠿⣿⣷⡌⠻⣆⠻⣝⢾⣿⣿⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⢻⣿⠟⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⣦⣀⣤⣴⣶⣾⣿⣿⣿⠃⠀⠀⠀⠀⠸⣿⣿⣿⣿⣿⣿⣿⣿
//⣿⠃⠀⠀⠀⠀⠀⠀⠈⠙⠛⠷⣮⣷⣦⣑⣿⣿⡿⠿⢣⠀⠀⠀⠀⠀⠀⣠⡾⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠛⠛⢻⣿⢍⣿⣿⣿⣿⡿⠃⠀⠀⣰⡆⢸⣿⣿⣿⣿⣿⣿⣿⣿
//⣿⡆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠙⢿⣿⣿⣿⠁⣀⣾⡆⠀⠀⣀⣤⠾⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡿⣥⣾⣷⣿⣿⣿⠃⠀⠀⢠⣿⠀⢸⣿⣿⣿⣿⣿⣿⣿⣿
//⣿⣷⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⠻⣿⣿⠸⢧⣿⣧⣴⣾⠿⠃⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⠶⣢⡵⠋⠀⠈⣿⣿⣿⣿⣇⠀⠀⢰⣿⣿⣷⣼⣿⣿⣿⣿⣿⣿⣿⣿
//⣿⣿⣧⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠻⣯⣴⣿⣿⣿⠟⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣴⣟⠟⠀⠀⠀⠀⠈⠻⣷⣿⡏⣠⣠⣿⣿⣿⣿⣿⣿⣿⣿⢻⣿⣿⣿⣿
//⣿⣿⣿⣧⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⠟⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢾⣿⣿⠀⠀⠀⠀⠀⠀⠀⠀⠁⠁⣿⡿⣿⣿⣿⣿⣿⣿⣿⣿⣾⣿⣿⣿⣿



//	[Verse 1]
//	I am doll eyes, doll mouth, doll legs
//	I am doll arms, big veins, dog beg
//
//	[Pre-Chorus]
//	Yeah, they really want you
//	They really want you, they really do
//	Yeah, they really want you
//	They really want you, and I do too
//
//	[Chorus]
//	I want to be the girl with the most cake
//	I love him so much, it just turns to hate
//	I fake it so real, I am beyond fake
//	And someday you will ache like I ache
//	And someday you will ache like I ache
//	And someday you will ache like I ache
//	And someday you will ache like I ache
//	Someday you will ache like I ache
//
//	[Verse 2]
//	I am doll parts
//	Bad skin, doll heart
//	It stands for ‘knife'
//	For the rest of my life
//
//	You might also like
//	Miss World
//	Hole
//	Violet
//	Hole
//	Jennifer’s Body
//	Hole
//
//	[Pre-Chorus]
//	Yeah, they really want you
//	They really want you, they really do
//	Yeah, they really want you
//	They really want you, but I do too
//
//	[Chorus]
//	I want to be the girl with the most cake
//	He only loves those things because he loves to see them break
//	I fake it so real, I am beyond fake
//	And someday you will ache like I ache
//	And someday you will ache like I ache
//	Someday you will ache like I ache
//	And someday you will ache like I ache
//	And someday you will ache like I ache
//	And someday you will ache like I ache
//	And someday you will ache like I ache
//
//	[Outro]
//	Someday you will ache like I ache




/* ---------- Private state ---------- */
static W25X_MODEL g_model = W25X_MODEL_W25N01GV;
static uint8_t    g_die   = 0;

/* ---------- Low-level helpers ---------- */
static inline void CS_LOW(void)  { HAL_GPIO_WritePin(W25N_CS_GPIO_PORT, W25N_CS_GPIO_PIN, GPIO_PIN_RESET); }
static inline void CS_HIGH(void) { HAL_GPIO_WritePin(W25N_CS_GPIO_PORT, W25N_CS_GPIO_PIN, GPIO_PIN_SET); }

static void spi_tx(const uint8_t *tx, uint32_t len) {
    CS_LOW();
    (void)HAL_SPI_Transmit(&W25N_SPI, (uint8_t*)tx, len, 50);
    CS_HIGH();
}

static void spi_txrx(const uint8_t *tx, uint8_t *rx, uint32_t len) {
    CS_LOW();
    (void)HAL_SPI_TransmitReceive(&W25N_SPI, (uint8_t*)tx, rx, len, 50);
    CS_HIGH();
}

static void spi_tx_then_rx(const uint8_t *tx, uint32_t tx_len, uint8_t *rx, uint32_t rx_len) {
    uint8_t dummy;
	CS_LOW();
    (void)HAL_SPI_Transmit(&W25N_SPI, (uint8_t*)tx, tx_len, 50);
    (void)HAL_SPI_Receive(&W25N_SPI, &dummy, 1, 100);
    (void)HAL_SPI_Receive(&W25N_SPI, rx, rx_len, 100);
    CS_HIGH();
}

static uint32_t max_page(void) {
    return (g_model == W25X_MODEL_W25M02GV) ? W25M02GV_MAX_PAGE : W25N01GV_MAX_PAGE;
}

/* ---------- Register access ---------- */
static uint8_t read_status_reg(uint8_t reg_addr) {
    /* 05h + reg + dummy -> returns status in 3rd byte */
    uint8_t tx[3] = { W25X_CMD_READ_STATUS_REG, reg_addr, 0x00 };
    uint8_t rx[3] = { 0 };
    spi_txrx(tx, rx, 3);
    return rx[2];
}

static void write_enable(void) {
    uint8_t tx = W25X_CMD_WRITE_ENABLE;
    spi_tx(&tx, 1);
}

static uint8_t wait_ready(void) {
    /* Poll BUSY bit in SR-3 (C0h) until it clears */
    for (uint32_t i = 0; i < 500000; i++) { /* simple timeout loop */
        uint8_t sr3 = read_status_reg(W25X_REG_STAT);
        if ((sr3 & W25X_SR3_BUSY) == 0) return W25X0XGV_OK;
    }
    return W25X0XGV_ERROR;
}

static uint8_t ensure_wel(void) {
    /* After WREN, WEL should be 1 in SR-3 */
    for (uint32_t i = 0; i < 1000; i++) {
        uint8_t sr3 = read_status_reg(W25X_REG_STAT);
        if (sr3 & W25X_SR3_WEL) return W25X0XGV_OK;
    }
    return W25X0XGV_ERROR;
}

static uint8_t write_status_reg(uint8_t reg_addr, uint8_t value) {
    if (wait_ready() != W25X0XGV_OK) return W25X0XGV_ERROR;

    write_enable();
    if (ensure_wel() != W25X0XGV_OK) return W25X0XGV_ERROR;

    uint8_t tx[3] = { W25X_CMD_WRITE_STATUS_REG, reg_addr, value };
    spi_tx(tx, 3);

    return wait_ready();
}

/* ---------- W25M die select ---------- */
static void die_select(uint8_t die) {
    uint8_t tx[2] = { W25M_CMD_DIE_SELECT, die };
    spi_tx(tx, 2);
    g_die = die;
}

static uint8_t die_select_by_page(uint32_t page_addr) {
    if (g_model != W25X_MODEL_W25M02GV) return W25X0XGV_OK;
    if (page_addr > max_page()) return W25X0XGV_ERROR;

    /* pages 0..65535 -> die 0, pages 65536..131071 -> die 1 */
    uint8_t die = (page_addr > W25N01GV_MAX_PAGE) ? 1u : 0u;
    if (die != g_die) die_select(die);
    return W25X0XGV_OK;
}

/* ---------- JEDEC ID ---------- */
static uint8_t read_jedec(uint8_t *man, uint16_t *dev) {
    /* send 9F, then read 3 ID bytes: EF, AA, 21 */
    uint8_t cmd = W25X_CMD_JEDEC_ID;
    uint8_t id[3] = {0};

    spi_tx_then_rx(&cmd, 1, id, 3);

    *man = id[0];
    *dev = (uint16_t)((uint16_t)id[1] << 8) | id[2];
    return W25X0XGV_OK;
}

/* ---------- Public API ---------- */
uint8_t W25X0XGV_begin(void) {
    /* Reset */
    if (wait_ready() != W25X0XGV_OK) return W25X0XGV_ERROR;

    uint8_t rst = W25X_CMD_RESET;
    spi_tx(&rst, 1);

    /* wait reset complete */
    if (wait_ready() != W25X0XGV_OK) return W25X0XGV_ERROR;

    uint8_t man = 0;
    uint16_t dev = 0;
    (void)read_jedec(&man, &dev);

    if (man != WINBOND_MAN_ID) return W25X0XGV_ERROR;

    if (dev == W25N01GV_DEV_ID) {
        g_model = W25X_MODEL_W25N01GV;

        /* Unprotect: set Protection Register-1 (A0h) to 0 */
        if (write_status_reg(W25X_REG_PROT, 0x00) != W25X0XGV_OK) return W25X0XGV_ERROR;

        return W25X0XGV_OK;
    }

    if (dev == W25M02GV_DEV_ID) {
        g_model = W25X_MODEL_W25M02GV;

        /* Unprotect both dies */
        die_select(0);
        if (write_status_reg(W25X_REG_PROT, 0x00) != W25X0XGV_OK) return W25X0XGV_ERROR;

        die_select(1);
        if (write_status_reg(W25X_REG_PROT, 0x00) != W25X0XGV_OK) return W25X0XGV_ERROR;

        /* default to die 0 */
        die_select(0);

        return W25X0XGV_OK;
    }

    return W25X0XGV_ERROR;
}

uint8_t W25X0XGV_get_model(W25X_MODEL *model_out) {
    if (!model_out) return W25X0XGV_ERROR;
    *model_out = g_model;
    return W25X0XGV_OK;
}

uint8_t W25X0XGV_block_erase(uint32_t page_addr) {
    if (page_addr > max_page()) return W25X0XGV_ERROR;
    if (die_select_by_page(page_addr) != W25X0XGV_OK) return W25X0XGV_ERROR;

    /* D8h + 0x00 + PA[15:8] + PA[7:0] */
    uint8_t pa_hi = (uint8_t)((page_addr >> 8) & 0xFF);
    uint8_t pa_lo = (uint8_t)(page_addr & 0xFF);
    uint8_t tx[4] = { W25X_CMD_BLOCK_ERASE, 0x00, pa_hi, pa_lo };

    if (wait_ready() != W25X0XGV_OK) return W25X0XGV_ERROR;

    write_enable();
    if (ensure_wel() != W25X0XGV_OK) return W25X0XGV_ERROR;

    spi_tx(tx, 4);
    return wait_ready();
}

uint8_t W25X0XGV_bulk_erase(void) {
    /* erase per block: each block is 64 pages */
    uint32_t mp = max_page();
    for (uint32_t page = 0; page <= mp; page += W25X_PAGES_PER_BLOCK) {
        if (W25X0XGV_block_erase(page) != W25X0XGV_OK) return W25X0XGV_ERROR;
    }
    return W25X0XGV_OK;
}

uint8_t W25X0XGV_load_prog_data(const uint8_t *buf, uint32_t data_len) {
    if (!buf) return W25X0XGV_ERROR;
    if (data_len > W25X_MAX_COLUMN) return W25X0XGV_ERROR;

    /* 02h + CA[15:8] + CA[7:0] (here CA=0) */
    uint8_t tx[3] = { W25X_CMD_PROG_DATA_LOAD, 0x00, 0x00 };

    if (wait_ready() != W25X0XGV_OK) return W25X0XGV_ERROR;

    write_enable();
    if (ensure_wel() != W25X0XGV_OK) return W25X0XGV_ERROR;

    CS_LOW();
    (void)HAL_SPI_Transmit(&W25N_SPI, tx, 3, 50);
    (void)HAL_SPI_Transmit(&W25N_SPI, (uint8_t*)buf, data_len, 200);
    CS_HIGH();

    return W25X0XGV_OK;
}

uint8_t W25X0XGV_program_execute(uint32_t page_addr) {
    if (page_addr > max_page()) return W25X0XGV_ERROR;
    if (die_select_by_page(page_addr) != W25X0XGV_OK) return W25X0XGV_ERROR;

    /* 10h + 0x00 + PA[15:8] + PA[7:0] */
    uint8_t pa_hi = (uint8_t)((page_addr >> 8) & 0xFF);
    uint8_t pa_lo = (uint8_t)(page_addr & 0xFF);
    uint8_t tx[4] = { W25X_CMD_PROG_EXECUTE, 0x00, pa_hi, pa_lo };

    if (wait_ready() != W25X0XGV_OK) return W25X0XGV_ERROR;

    write_enable();
    if (ensure_wel() != W25X0XGV_OK) return W25X0XGV_ERROR;

    spi_tx(tx, 4);
    return wait_ready();
}

uint8_t W25X0XGV_page_data_read(uint32_t page_addr) {
    if (page_addr > max_page()) return W25X0XGV_ERROR;
    if (die_select_by_page(page_addr) != W25X0XGV_OK) return W25X0XGV_ERROR;

    /* 13h + 0x00 + PA[15:8] + PA[7:0] */
    uint8_t pa_hi = (uint8_t)((page_addr >> 8) & 0xFF);
    uint8_t pa_lo = (uint8_t)(page_addr & 0xFF);
    uint8_t tx[4] = { W25X_CMD_PAGE_DATA_READ, 0x00, pa_hi, pa_lo };

    if (wait_ready() != W25X0XGV_OK) return W25X0XGV_ERROR;

    spi_tx(tx, 4);
    return wait_ready();
}

uint8_t W25X0XGV_read(uint8_t *buf, uint32_t data_len) {
    if (!buf) return W25X0XGV_ERROR;
    if (data_len > W25X_MAX_COLUMN) return W25X0XGV_ERROR;
    uint8_t res = HAL_OK;

    /* 03h + CA[15:8] + CA[7:0] + dummy (BUF=1 mode)
       Here: start at column 0.
    */
    uint8_t tx[4] = { W25X_CMD_READ_DATA, 0x00, 0x00, 0x00 };

    if (wait_ready() != W25X0XGV_OK) return W25X0XGV_ERROR;

    CS_LOW();
    (void)HAL_SPI_Transmit(&W25N_SPI, tx, 4, 50);

    res = HAL_SPI_Receive(&W25N_SPI, buf, data_len, 5000);
    CS_HIGH();

    if (res != HAL_OK) return W25X0XGV_ERROR;

    return W25X0XGV_OK;
}




/* Test */
uint8_t flash_self_test(uint32_t test_page)
{
	uint8_t tx_buf[2048];
	uint8_t rx_buf[2048];

    //  Testpattern
    for (uint32_t i = 0; i < 2048; i++)
        tx_buf[i] = i & 0xFF;

    //  delete block
    if (W25X0XGV_block_erase(test_page) != W25X0XGV_OK)
        return 1;

    //  load data into buffer
    if (W25X0XGV_load_prog_data(tx_buf, 2048) != W25X0XGV_OK)
        return 2;

    //  Program Execute
    if (W25X0XGV_program_execute(test_page) != W25X0XGV_OK)
        return 3;

    // intern Buffer
    if (W25X0XGV_page_data_read(test_page) != W25X0XGV_OK)
        return 4;

    // Read
    if (W25X0XGV_read(rx_buf, 2048) != W25X0XGV_OK)
        return 5;

    // Compare
    for (uint32_t i = 0; i < 2048; i++)
    {
        if (rx_buf[i] != tx_buf[i])
            return 6;  // Error
    }

    return 0;  // Everything okay
}
