#pragma once

#include <stdint.h>

#include "i2c_bus.h"

#include "BME680_driver/bme680.h"


#define BME680_TAG "bme680"

struct bme680 {
  struct i2c_bus*   i2c_bus;
  uint8_t           i2c_addr;
  struct bme680_dev bme;
};

esp_err_t bme_init(struct bme680* bme, struct i2c_bus* i2c_bus, uint8_t i2c_addr);
