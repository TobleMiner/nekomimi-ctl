#include <stdint.h>
#include <string.h>

#include <freertos/FreeRTOS.h>

#include <esp_err.h>
#include <esp_log.h>

#include "bme680.h"
#include "i2c_bus.h"

// Private API
static esp_err_t bme680_write_reg(struct bme680* bme, uint8_t reg, uint8_t* val, size_t len) {
  esp_err_t err;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  if(!cmd) {
    err = ESP_ERR_NO_MEM;
    goto fail;
  }
  if((err = i2c_master_start(cmd))) {
    goto fail_link;
  }
  if((err = i2c_master_write_byte(cmd, bme->i2c_addr << 1, 1))) {
    goto fail_link;
  }
  if((err = i2c_master_write_byte(cmd, reg, 1))) {
    goto fail_link;
  }
  if((err = i2c_master_write(cmd, val, len, 1))) {
    goto fail_link;
  }
  if((err = i2c_master_stop(cmd))) {
    goto fail_link;
  }
  err = i2c_bus_cmd_begin(bme->i2c_bus, cmd, 100 / portTICK_RATE_MS);
fail_link:
  i2c_cmd_link_delete(cmd);
fail:
  return err;
}

static esp_err_t bme680_read_reg(struct bme680* bme, uint8_t reg, uint8_t* val, size_t len) {
  esp_err_t err;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  if(!cmd) {
    err = ESP_ERR_NO_MEM;
    goto fail;
  }
  if((err = i2c_master_start(cmd))) {
    goto fail_link;
  }
  if((err = i2c_master_write_byte(cmd, (bme->i2c_addr << 1), 1))) {
    goto fail_link;
  }
  if((err = i2c_master_write_byte(cmd, reg, 1))) {
    goto fail_link;
  }
  if((err = i2c_master_start(cmd))) {
    goto fail_link;
  }
  if((err = i2c_master_write_byte(cmd, (bme->i2c_addr << 1) | 1, 1))) {
    goto fail_link;
  }
  if((err = i2c_master_read(cmd, val, len, I2C_MASTER_LAST_NACK))) {
    goto fail_link;
  }
  if((err = i2c_master_stop(cmd))) {
    goto fail_link;
  }
  err = i2c_bus_cmd_begin(bme->i2c_bus, cmd, 100 / portTICK_RATE_MS);
fail_link:
  i2c_cmd_link_delete(cmd);
fail:
  return err;
}

// Bosch sensor lib interface
static void bme680_api_delay_ms(uint32_t period, void* priv) {
  vTaskDelay(period / portTICK_RATE_MS);
}

static int8_t bme680_api_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len, void* priv) {
  struct bme680* bme = priv;
  return bme680_read_reg(bme, reg_addr, reg_data, len);
}

static int8_t bme680_api_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len, void* priv) {
  struct bme680* bme = priv;
  return bme680_write_reg(bme, reg_addr, reg_data, len);
}

// Public API
esp_err_t bme680_reset(struct bme680* bme) {
  esp_err_t err;
  bme->bme.tph_sett.os_hum = BME680_OS_2X;
  bme->bme.tph_sett.os_pres = BME680_OS_4X;
  bme->bme.tph_sett.os_temp = BME680_OS_8X;
  bme->bme.tph_sett.filter = BME680_FILTER_SIZE_3;

//  bme->bme.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
//  bme->bme.gas_sett.heatr_temp = 320;
//  bme->bme.gas_sett.heatr_dur = 150;

  bme->bme.power_mode = BME680_FORCED_MODE;

  err = bme680_set_sensor_settings(BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL | BME680_GAS_SENSOR_SEL, &bme->bme);
  if(err) {
    ESP_LOGE(BME680_TAG, "Failed to set BME680 settings");
  }

  return err;
}

esp_err_t bme_init(struct bme680* bme, struct i2c_bus* i2c_bus, uint8_t i2c_addr) {
  esp_err_t err;
  memset(bme, 0, sizeof(*bme));

  bme->i2c_bus = i2c_bus;
  bme->i2c_addr = i2c_addr;

  bme->bme.dev_id = i2c_addr;
  bme->bme.intf = BME680_I2C_INTF;
  bme->bme.read = bme680_api_i2c_read;
  bme->bme.write = bme680_api_i2c_write;
  bme->bme.delay_ms = bme680_api_delay_ms;
  bme->bme.fp_priv = bme;

  err = bme680_init(&bme->bme);
  if(err) {
    ESP_LOGE(BME680_TAG, "Failed to initialize BME680@%02x", i2c_addr);
    return err;
  }

  err = bme680_reset(bme);
  if(err) {
    ESP_LOGE(BME680_TAG, "Failed to reset BME680");
    return err;
  }

  return err;
}
