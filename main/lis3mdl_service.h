#pragma once

#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <esp_err.h>

#include "lis3mdl.h"

#define LIS3MDL_SERVICE_TAG "lis3mdl_service"

#define LIS3MDL_SERVICE_STACK 4096

struct lis3mdl_service {
  struct lis3mdl lis;
  struct lis3mdl_result res;
  int drdy_gpio;
  SemaphoreHandle_t lock;
  TaskHandle_t task;
};

esp_err_t lis3mdl_service_init(struct lis3mdl_service* service, struct i2c_bus* bus, uint8_t i2c_addr, int drdy_gpio);

void lis3mdl_service_measure_raw(struct lis3mdl_service* service, struct lis3mdl_result* res);