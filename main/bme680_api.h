#pragma once

#include "i2c_bus.h"

#define BME680_API_TAG "bme680_api"

extern struct i2c_bus* bme680_api_i2c_bus;

void bme_api_delay_ms(uint32_t period);
int8_t bme_api_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t bme_api_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t bme_api_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t bme_api_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
