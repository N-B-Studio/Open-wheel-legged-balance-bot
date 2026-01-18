#pragma once
#include "stm32h7xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
    UART_HandleTypeDef *huart;

    // RX state
    uint8_t  rx_byte;
    uint32_t rx_bytes;

    uint8_t  buf[64];
    uint8_t  idx;
    uint8_t  need;        // total bytes needed for current frame (addr+len+type+payload+crc)

    // decoded
    uint16_t ch8[8];      // 0..2047 (CRSF packed 11-bit)
    uint32_t ok;
    uint32_t bad;
    uint32_t last_ms;

    // debug
    uint8_t  last_addr;
    uint8_t  last_type;
    uint8_t  last_len;
} Crsf8;

void     crsf8_init(Crsf8 *c, UART_HandleTypeDef *huart);
HAL_StatusTypeDef crsf8_start_rx_it(Crsf8 *c);

// feed bytes from IRQ
void     crsf8_on_rx_byte(Crsf8 *c, uint8_t b);
void     crsf8_on_uart_error(Crsf8 *c);

static inline uint32_t crsf8_age_ms(const Crsf8 *c) {
    return HAL_GetTick() - c->last_ms;
}

#ifdef __cplusplus
}
#endif
