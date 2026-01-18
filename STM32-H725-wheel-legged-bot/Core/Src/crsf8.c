#include "crsf8.h"
#include <string.h>

#define CRSF_ADDR_RX          0xC8  // receiver
#define CRSF_ADDR_TX          0xEA  // transmitter (sometimes seen)
#define CRSF_TYPE_RC_CHANNELS 0x16

// CRSF CRC8: poly 0xD5, init 0x00
static uint8_t crc8_d5(const uint8_t *ptr, uint8_t len)
{
    uint8_t crc = 0;
    while (len--) {
        crc ^= *ptr++;
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0xD5);
            else           crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static void decode_rc_8ch(const uint8_t *p, uint16_t *ch8)
{
    // p points to RC payload (22 bytes, 16ch packed 11-bit). We only extract 8ch.
    ch8[0] = (uint16_t)(( p[0]        | (p[1] << 8)) & 0x07FF);
    ch8[1] = (uint16_t)(((p[1] >> 3)  | (p[2] << 5)) & 0x07FF);
    ch8[2] = (uint16_t)(((p[2] >> 6)  | (p[3] << 2) | (p[4] << 10)) & 0x07FF);
    ch8[3] = (uint16_t)(((p[4] >> 1)  | (p[5] << 7)) & 0x07FF);
    ch8[4] = (uint16_t)(((p[5] >> 4)  | (p[6] << 4)) & 0x07FF);
    ch8[5] = (uint16_t)(((p[6] >> 7)  | (p[7] << 1) | (p[8] << 9)) & 0x07FF);
    ch8[6] = (uint16_t)(((p[8] >> 2)  | (p[9] << 6)) & 0x07FF);
    ch8[7] = (uint16_t)(((p[9] >> 5)  | (p[10] << 3)) & 0x07FF);
}

void crsf8_init(Crsf8 *c, UART_HandleTypeDef *huart)
{
    memset(c, 0, sizeof(*c));
    c->huart = huart;
}

HAL_StatusTypeDef crsf8_start_rx_it(Crsf8 *c)
{
    c->idx  = 0;
    c->need = 0;
    return HAL_UART_Receive_IT(c->huart, &c->rx_byte, 1);
}

static void reset_frame(Crsf8 *c)
{
    c->idx  = 0;
    c->need = 0;
}

void crsf8_on_rx_byte(Crsf8 *c, uint8_t b)
{
    c->rx_bytes++;
    c->last_addr = b; // updated later; just for debug visibility

    // Frame format:
    // [0]=addr, [1]=len, then len bytes: [type + payload + crc]
    // total bytes = 2 + len
    if (c->idx == 0) {
        // sync on known addresses to avoid noise
        if (b == CRSF_ADDR_RX || b == CRSF_ADDR_TX) {
            c->buf[0] = b;
            c->idx = 1;
        }
        return;
    }

    c->buf[c->idx++] = b;

    if (c->idx == 2) {
        uint8_t len = c->buf[1];
        c->last_len = len;

        // Sanity: typical RC frame len is 24 (type+22payload+crc) => total 26
        // Accept reasonable bounds to keep resync stable
        if (len < 2 || len > 60) {
            c->bad++;
            reset_frame(c);
            return;
        }
        c->need = (uint8_t)(2 + len);
    }

    if (c->need && c->idx >= c->need) {
        // full frame received
        uint8_t addr = c->buf[0];
        uint8_t len  = c->buf[1];
        uint8_t type = c->buf[2];
        c->last_addr = addr;
        c->last_type = type;

        // CRC is last byte of the len block
        uint8_t crc_rx = c->buf[2 + len - 1];

        // CRC is computed over [type + payload] (i.e. len-1 bytes starting at buf[2])
        uint8_t crc_calc = crc8_d5(&c->buf[2], (uint8_t)(len - 1));

        if (crc_calc == crc_rx) {
            if (type == CRSF_TYPE_RC_CHANNELS && len >= (1 + 22 + 1)) {
                decode_rc_8ch(&c->buf[3], c->ch8);
                c->last_ms = HAL_GetTick();
                c->ok++;
            }
        } else {
            c->bad++;
        }

        reset_frame(c);
    }
}

void crsf8_on_uart_error(Crsf8 *c)
{
    UART_HandleTypeDef *hu = c->huart;

    __HAL_UART_CLEAR_OREFLAG(hu);
    __HAL_UART_CLEAR_FEFLAG(hu);
    __HAL_UART_CLEAR_NEFLAG(hu);

    HAL_UART_AbortReceive_IT(hu);
    reset_frame(c);

    // restart
    (void)HAL_UART_Receive_IT(hu, &c->rx_byte, 1);
}
