#pragma once

#include <stdint.h>

#include <esp_err.h>
#include <esp_timer.h>

#include "../BME680_driver/bme680.h"
#include <bsec_interface.h>

#include "../i2c_bus.h"
#include "../bme680.h"

#define BME680_SERVICE_TAG "bme680_srv non-free"

#define BME680_SERVICE_STACK 4096

#define BME680_SERVICE_MEASURE_INTERVAL 500

typedef void (*bme680_service_cb)(void* priv);

enum BME680_SERVICE_STATE {
  BME680_SERVICE_STATE_IDLE = 0,
  BME680_SERVICE_STATE_MEASURE,
};

struct bme680_service {
  struct bme680 bme;
  SemaphoreHandle_t lock;
  esp_timer_handle_t timer;
  enum BME680_SERVICE_STATE state;
  int64_t mesaurement_start;

  struct bme680_field_data res;

  bme680_service_cb cb;
  void* cb_priv;

  uint8_t work_buffer[BSEC_MAX_PROPERTY_BLOB_SIZE];
  bsec_sensor_configuration_t required_sensor_settings[BSEC_MAX_PHYSICAL_SENSOR];
  uint8_t num_required_sensor_settings;
};

struct bme680_service_rate {
  float temperature;
  float humidity;
  float pressure;
  float iaq;
};

esp_err_t bme680_service_init(struct bme680_service* service, struct i2c_bus* bus, uint8_t i2c_addr);

void bme680_service_measure(struct bme680_service* service, struct bme680_field_data* meas);

void bme680_service_set_cb(struct bme680_service* service, bme680_service_cb cb, void* priv);
