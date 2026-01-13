#pragma once
#include "spi.h"
#include "gpio.h"
#include <stdint.h>

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
} AS5047P;

void as5047p_init(AS5047P *enc);
uint16_t as5047p_read_angle_raw14(AS5047P *enc, uint8_t *err);

// debug optional:
uint16_t as5047p_last_rx2(void);
