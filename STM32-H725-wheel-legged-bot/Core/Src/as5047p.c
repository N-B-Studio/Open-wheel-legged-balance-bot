#include "as5047p.h"

#define AS5047P_CMD_READ       0x4000u
#define AS5047P_REG_ANGLECOM   0x3FFFu
#define AS5047P_DATA_MASK      0x3FFFu
#define AS5047P_ERR_MASK       0x4000u

static uint16_t g_last_rx2 = 0;

static inline void CS_Low(AS5047P *e)  { HAL_GPIO_WritePin(e->cs_port, e->cs_pin, GPIO_PIN_RESET); }
static inline void CS_High(AS5047P *e) { HAL_GPIO_WritePin(e->cs_port, e->cs_pin, GPIO_PIN_SET); }

static HAL_StatusTypeDef spi_txrx_u16(AS5047P *e, uint16_t tx, uint16_t *rx)
{
    uint8_t txb[2] = {(uint8_t)(tx >> 8), (uint8_t)(tx & 0xFF)};
    uint8_t rxb[2] = {0, 0};
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(e->hspi, txb, rxb, 2, 2);
    if (st == HAL_OK && rx) *rx = ((uint16_t)rxb[0] << 8) | (uint16_t)rxb[1];
    return st;
}

// odd parity bit at bit15
static inline uint16_t apply_parity(uint16_t v)
{
    uint16_t x = v;
    x ^= x >> 8;
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    uint16_t odd = x & 1u;
    if (odd) v |= 0x8000u; else v &= 0x7FFFu;
    return v;
}

void as5047p_init(AS5047P *enc)
{
    CS_High(enc);
}

static uint16_t read_reg14(AS5047P *enc, uint16_t addr, uint8_t *err)
{
    if (err) *err = 0;

    uint16_t cmd = AS5047P_CMD_READ | (addr & 0x3FFFu);
    cmd = apply_parity(cmd);

    uint16_t rx1 = 0, rx2 = 0;

    CS_Low(enc);
    (void)spi_txrx_u16(enc, cmd, &rx1);
    CS_High(enc);

    __NOP(); __NOP(); __NOP();

    CS_Low(enc);
    (void)spi_txrx_u16(enc, 0x0000u, &rx2);
    CS_High(enc);

    g_last_rx2 = rx2;

    if ((rx2 & AS5047P_ERR_MASK) != 0u) {
        if (err) *err = 1;
    }

    return (rx2 & AS5047P_DATA_MASK);
}

uint16_t as5047p_read_angle_raw14(AS5047P *enc, uint8_t *err)
{
    return read_reg14(enc, AS5047P_REG_ANGLECOM, err);
}

uint16_t as5047p_last_rx2(void) { return g_last_rx2; }
