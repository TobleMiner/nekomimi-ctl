#include <stdint.h>

#include <freertos/FreeRTOS.h>

#include <esp_log.h>

#include "i2c_bus.h"
#include "bme680_api.h"

void bme_api_delay_ms(uint32_t period, void* priv) {
  vTaskDelay(period / portTICK_RATE_MS);
}

int8_t bme_api_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len, void* priv) {
	esp_err_t err;
  struct i2c_bus* bus = priv;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  if(!cmd) {
    err = ESP_ERR_NO_MEM;
    goto fail;
  }
  if((err = i2c_master_start(cmd))) {
    goto fail_link;
  }
  if((err = i2c_master_write_byte(cmd, (dev_id << 1), 1))) {
    goto fail_link;
  }
  if((err = i2c_master_write_byte(cmd, reg_addr, 1))) {
    goto fail_link;
  }
  if((err = i2c_master_start(cmd))) {
    goto fail_link;
  }
  if((err = i2c_master_write_byte(cmd, (dev_id << 1) | 1, 1))) {
    goto fail_link;
  }
  if((err = i2c_master_read(cmd, reg_data, len, I2C_MASTER_LAST_NACK))) {
    goto fail_link;
  }
  if((err = i2c_master_stop(cmd))) {
    goto fail_link;
  }
  err = i2c_bus_cmd_begin(bus, cmd, 100 / portTICK_RATE_MS);
fail_link:
  i2c_cmd_link_delete(cmd);
fail:
  return err;
}

int8_t bme_api_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len, void* priv) {
	esp_err_t err;
  struct i2c_bus* bus = priv;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  if(!cmd) {
    err = ESP_ERR_NO_MEM;
    goto fail;
  }
  if((err = i2c_master_start(cmd))) {
    goto fail_link;
  }
  if((err = i2c_master_write_byte(cmd, dev_id << 1, 1))) {
    goto fail_link;
  }
  if((err = i2c_master_write_byte(cmd, reg_addr, 1))) {
    goto fail_link;
  }
  if((err = i2c_master_write(cmd, reg_data, len, 1))) {
    goto fail_link;
  }
  if((err = i2c_master_stop(cmd))) {
    goto fail_link;
  }
  err = i2c_bus_cmd_begin(bus, cmd, 100 / portTICK_RATE_MS);
fail_link:
  i2c_cmd_link_delete(cmd);
fail:
  return err;
}