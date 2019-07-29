#pragma once

#include <stdint.h>

#include "BME680_driver/bme680.h"

#include "bme680.h"


#define BME680_SERVICE_TAG "bme680_srv"

#define BME680_SERVICE_STACK 4096

struct bme680_service {
  struct bme680 bme;
  SemaphoreHandle_t lock;

  struct bme680_field_data res;
};

esp_err_t bme680_service_init(struct bme680_service* service, struct i2c_bus* bus, uint8_t i2c_addr);

void bme680_service_measure(struct bme680_service* service, struct bme680_field_data* meas);
