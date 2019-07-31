#pragma once

#include <freertos/semphr.h>

#include "bh1750.h"

#define BH1750_SERVICE_TAG   "bh1750_service"

#define BH1750_SERVICE_STACK 4096

typedef void (*bh1750_service_cb)(void* priv);

struct bh1750_service {
  struct bh1750 bh;
  float illuminance;
  SemaphoreHandle_t lock;
  bh1750_service_cb cb;
  void* cb_priv;
};

esp_err_t bh1750_service_init(struct bh1750_service* service, struct i2c_bus* bus, uint8_t i2c_addr);

float bh1750_service_get_illuminance(struct bh1750_service* service);

void bh1750_service_set_cb(struct bh1750_service* service, bh1750_service_cb cb, void* priv);
