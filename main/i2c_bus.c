#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <esp_log.h>

#include "i2c_bus.h"

esp_err_t i2c_bus_init(struct i2c_bus* bus, i2c_port_t i2c_port, uint8_t gpio_sda, uint8_t gpio_scl, uint32_t speed) {
  esp_err_t err;
  i2c_config_t i2c_config = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = gpio_sda,
    .scl_io_num = gpio_scl,
    .master.clk_speed = speed,
  };

  err = i2c_param_config(i2c_port, &i2c_config);
  if(err) {
    return err;
  }
  err = i2c_driver_install(i2c_port, I2C_MODE_MASTER, 0, 0, 0);
  if(err) {
    return err;
  }

  memset(bus, 0, sizeof(*bus));
  bus->i2c_port = i2c_port;
  bus->lock = xSemaphoreCreateMutex();
  if(!bus->lock) {
    err = ESP_ERR_NO_MEM;
  }
  return err;
}

esp_err_t i2c_bus_cmd_begin(struct i2c_bus* bus, i2c_cmd_handle_t handle, TickType_t timeout) {
  esp_err_t err;
  xSemaphoreTake(bus->lock, portMAX_DELAY);
  err = i2c_master_cmd_begin(bus->i2c_port, handle, timeout);
  xSemaphoreGive(bus->lock);
  return err;
}

#define TAG_I2CDETECT "i2c_detect"

void i2c_detect(struct i2c_bus* bus) {
  ESP_LOGI(TAG_I2CDETECT, "Scanning i2c bus %d for devices", bus->i2c_port);
  for(uint8_t i = 0; i < 127; i++) {
  	esp_err_t err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if(!cmd) {
      err = ESP_ERR_NO_MEM;
      goto fail;
    }
    if((err = i2c_master_start(cmd))) {
      goto fail_link;
    }
    if((err = i2c_master_write_byte(cmd, (i << 1), 1))) {
      goto fail_link;
    }
    if((err = i2c_master_stop(cmd))) {
      goto fail_link;
    }
    err = i2c_bus_cmd_begin(bus, cmd, 100 / portTICK_RATE_MS);
    if(!err) {
      ESP_LOGI(TAG_I2CDETECT, "Device %02x is present", i);
    }
    i2c_cmd_link_delete(cmd);
    continue;

  fail_link:
    i2c_cmd_link_delete(cmd);
  fail:
    ESP_LOGI(TAG_I2CDETECT, "Failed to probe for %02x", i);
  }
}