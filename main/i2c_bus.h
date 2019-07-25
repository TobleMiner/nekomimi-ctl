#pragma once

#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <esp_err.h>
#include <driver/i2c.h>

struct i2c_bus {
  i2c_port_t i2c_port;
  SemaphoreHandle_t lock;
};

esp_err_t i2c_bus_init(struct i2c_bus* bus, i2c_port_t i2c_port, uint8_t gpio_sda, uint8_t gpio_scl, uint32_t speed);
esp_err_t i2c_bus_cmd_begin(struct i2c_bus* bus, i2c_cmd_handle_t handle, TickType_t timeout);

void i2c_detect(struct i2c_bus* i2c_bus);
