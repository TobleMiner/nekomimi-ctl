#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "../BME680_driver/bme680.h"

#include "../i2c_bus.h"
#include "../bme680.h"


#define BME680_SERVICE_TAG "bme680_srv"

#define BME680_SERVICE_STACK 4096

#define BME680_SERVICE_MEASURE_INTERVAL 500

typedef void (*bme680_service_cb)(void* priv);

struct bme680_service_data {
  float temperature;
  float humidity;
  float pressure;
};

struct bme680_service {
  struct bme680 bme;
  SemaphoreHandle_t lock;

  struct bme680_service_data res;

  bme680_service_cb cb;
  void* cb_priv;
};

esp_err_t bme680_service_init(struct bme680_service* service, struct i2c_bus* bus, uint8_t i2c_addr);

void bme680_service_measure(struct bme680_service* service, struct bme680_service_data* meas);

void bme680_service_set_cb(struct bme680_service* service, bme680_service_cb cb, void* priv);
