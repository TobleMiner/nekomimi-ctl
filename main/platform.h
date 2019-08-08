#pragma once

#include <esp_err.h>

#include "config.h"

#include "ear.h"
#include "sensor.h"
#include "bh1750_sensor.h"
#include "bme680_sensor.h"
#include "lis3mdl_sensor.h"

#define PLATFORM_TAG "nekomimi_plat"

extern struct sensor_manager sensors;
extern struct tlc_chain ears;
extern struct i2c_bus i2c_sensors;

esp_err_t platform_init();

static inline esp_err_t platform_set_led_rgb(unsigned int ear, unsigned int led, uint16_t red, uint16_t green, uint16_t blue) {
  return ear_set_led_rgb(&ears, ear, led, red, green, blue);
}

static inline esp_err_t platform_set_led_rgb888(unsigned int ear, unsigned int led, uint8_t red, uint8_t green, uint8_t blue) {
  return ear_set_led_rgb888(&ears, ear, led, red, green, blue);
}

static inline esp_err_t platform_set_led_uv(unsigned int ear, unsigned int led, uint16_t level) {
  return ear_set_led_uv(&ears, ear, led, level);
}

static inline esp_err_t platform_set_led_uv8(unsigned int ear, unsigned int led, uint8_t level) {
  return ear_set_led_uv8(&ears, ear, led, level);
}

static inline esp_err_t platform_subscribe_sensor(sensor_param_t param, sensor_subscriber_cb cb, void* priv) {
  return sensors_subscribe(&sensors, param, cb, priv);
}

static inline bool platform_has_sensor(struct sensor_manager* mgr, sensor_param_t param) {
  return sensors_has_sensor(&sensors, param);
}

static inline esp_err_t platform_get_sensor_result(sensor_param_t param, sensor_result_t* res, size_t len) {
  return sensors_get_result(&sensors, param, res, len);
}

static inline unsigned int platform_get_num_ears() {
  return ears.chain_len;
}
