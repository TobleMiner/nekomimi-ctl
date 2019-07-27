#include <stdint.h>
#include <string.h>

#include <esp_err.h>
#include <esp_log.h>

#include "lis3mdl.h"

// Private API
static esp_err_t lis3mdl_write_reg(struct lis3mdl* lis, uint8_t reg, uint8_t val) {
	esp_err_t err;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  if(!cmd) {
    err = ESP_ERR_NO_MEM;
    goto fail;
  }
  if((err = i2c_master_start(cmd))) {
    goto fail_link;
  }
  if((err = i2c_master_write_byte(cmd, lis->i2c_addr << 1, 1))) {
    goto fail_link;
  }
  if((err = i2c_master_write_byte(cmd, reg, 1))) {
    goto fail_link;
  }
  if((err = i2c_master_write_byte(cmd, val, 1))) {
    goto fail_link;
  }
  if((err = i2c_master_stop(cmd))) {
    goto fail_link;
  }
  err = i2c_bus_cmd_begin(lis->i2c_bus, cmd, 100 / portTICK_RATE_MS);
fail_link:
  i2c_cmd_link_delete(cmd);
fail:
  return err;
}

static esp_err_t lis3mdl_read_reg(struct lis3mdl* lis, uint8_t reg, uint8_t* val) {
	esp_err_t err;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  if(!cmd) {
    err = ESP_ERR_NO_MEM;
    goto fail;
  }
  if((err = i2c_master_start(cmd))) {
    goto fail_link;
  }
  if((err = i2c_master_write_byte(cmd, (lis->i2c_addr << 1), 1))) {
    goto fail_link;
  }
  if((err = i2c_master_write_byte(cmd, reg, 1))) {
    goto fail_link;
  }
  if((err = i2c_master_start(cmd))) {
    goto fail_link;
  }
  if((err = i2c_master_write_byte(cmd, (lis->i2c_addr << 1) | 1, 1))) {
    goto fail_link;
  }
  if((err = i2c_master_read_byte(cmd, val, 1))) {
    goto fail_link;
  }
  if((err = i2c_master_stop(cmd))) {
    goto fail_link;
  }
  err = i2c_bus_cmd_begin(lis->i2c_bus, cmd, 100 / portTICK_RATE_MS);
fail_link:
  i2c_cmd_link_delete(cmd);
fail:
  return err;
}

static esp_err_t lis3mdl_read_result(struct lis3mdl* lis, struct lis3mdl_result* res) {
	esp_err_t err;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  if(!cmd) {
    err = ESP_ERR_NO_MEM;
    goto fail;
  }
  if((err = i2c_master_start(cmd))) {
    goto fail_link;
  }
  if((err = i2c_master_write_byte(cmd, (lis->i2c_addr << 1), 1))) {
    goto fail_link;
  }
  if((err = i2c_master_write_byte(cmd, LIS3MDL_REG_OUT_X_L | LIS3MDL_AUTO_INCREMENT, 1))) {
    goto fail_link;
  }
  if((err = i2c_master_start(cmd))) {
    goto fail_link;
  }
  if((err = i2c_master_write_byte(cmd, (lis->i2c_addr << 1) | 1, 1))) {
    goto fail_link;
  }
  if((err = i2c_master_read(cmd, (uint8_t*)res, sizeof(*res), I2C_MASTER_LAST_NACK))) {
    goto fail_link;
  }
  if((err = i2c_master_stop(cmd))) {
    goto fail_link;
  }
  err = i2c_bus_cmd_begin(lis->i2c_bus, cmd, 100 / portTICK_RATE_MS);
fail_link:
  i2c_cmd_link_delete(cmd);
fail:
  return err;
}

// Public API
esp_err_t lis3mdl_reset(struct lis3mdl* lis) {
  // Perform register reset
  esp_err_t err = lis3mdl_write_reg(lis, LIS3MDL_REG_CTRL_REG2, 0b00001100);
  if(err) {
    ESP_LOGE(LIS3MDL_TAG, "Failed to reset magnetometer");
    return err;
  }
  vTaskDelay(500 / portTICK_PERIOD_MS);
  lis->range = LIS3MDL_RANGE_4_GAUSS;

  // Enable temperature sensor, fast measurement and high performance mode
  err = lis3mdl_write_reg(lis, LIS3MDL_REG_CTRL_REG1, 0b11100010);
  if(err) {
    ESP_LOGE(LIS3MDL_TAG, "Failed to setup control register 1");
    return err;
  }

  // Enable continous conversion mode
  err = lis3mdl_write_reg(lis, LIS3MDL_REG_CTRL_REG3, 0b00000000);
  if(err) {
    ESP_LOGE(LIS3MDL_TAG, "Failed to setup control register 3");
    return err;
  }

  // Enter continous conversion mode
  err = lis3mdl_write_reg(lis, LIS3MDL_REG_CTRL_REG4, 0b00001100);
  if(err) {
    ESP_LOGE(LIS3MDL_TAG, "Failed to setup control register 4");
    return err;
  }

  // Enable block data update
  err = lis3mdl_write_reg(lis, LIS3MDL_REG_CTRL_REG5, 0b01000000);
  if(err) {
    ESP_LOGE(LIS3MDL_TAG, "Failed to setup control register 5");
    return err;
  }
  return err;
}

esp_err_t lis3mdl_init(struct lis3mdl* lis, struct i2c_bus* i2c_bus, uint8_t i2c_addr) {
  esp_err_t err;
  uint8_t id;
  memset(lis, 0, sizeof(*lis));
  lis->i2c_bus = i2c_bus;
  lis->i2c_addr = i2c_addr;
  err = lis3mdl_reset(lis);
  if(err) {
    ESP_LOGE(LIS3MDL_TAG, "Failed to reset lis3mdl");
    return err;
  }
  
  err = lis3mdl_read_reg(lis, LIS3MDL_REG_WHO_AM_I, &id);
  if(err) {
    ESP_LOGE(LIS3MDL_TAG, "Failed to read lis3mdl id");
    return err;
  }

  if(id != LIS3MDL_ID) {
    ESP_LOGE(LIS3MDL_TAG, "Unexpected id %02x, should be %02x", id, LIS3MDL_ID);
    return ESP_ERR_INVALID_RESPONSE;
  }

  return ESP_OK;
}

esp_err_t lis3mdl_set_range(struct lis3mdl* lis, enum LIS3MDL_RANGE range) {
  esp_err_t err;
  if(range < LIS3MDL_RANGE_4_GAUSS || range >= LIS3MDL_RANGE_LIMIT_) {
    return ESP_ERR_INVALID_ARG;
  }

  err = lis3mdl_write_reg(lis, LIS3MDL_REG_CTRL_REG2, range << 5);
  if(err) {
    return err;
  }
  lis->range = range;

  return ESP_OK;
}

esp_err_t lis3mdl_measure_raw(struct lis3mdl* lis, struct lis3mdl_result* res) {
  return lis3mdl_read_result(lis, res);
}