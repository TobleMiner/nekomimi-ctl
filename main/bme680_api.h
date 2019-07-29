#pragma once

#include "i2c_bus.h"

#define BME680_API_TAG "bme680_api"

void bme_api_delay_ms(uint32_t period, void* priv);
int8_t bme_api_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len, void* priv);
int8_t bme_api_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len, void* priv);
