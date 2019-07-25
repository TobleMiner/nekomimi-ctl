#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include <freertos/FreeRTOS.h>

#include "bh1750.h"

// Private API
static esp_err_t bh1750_cmd(struct bh1750* bh, uint8_t bhcmd) {
	esp_err_t err;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  if(!cmd) {
    err = ESP_ERR_NO_MEM;
    goto fail;
  }
  if((err = i2c_master_start(cmd))) {
    goto fail_link;
  }
  if((err = i2c_master_write_byte(cmd, bh->i2c_addr << 1, 1))) {
    goto fail_link;
  }
  if((err = i2c_master_write_byte(cmd, bhcmd, 1))) {
    goto fail_link;
  }
    if((err = i2c_master_stop(cmd))) {
    goto fail_link;
  }
  err = i2c_bus_cmd_begin(bh->i2c_bus, cmd, 100 / portTICK_RATE_MS);
fail_link:
  i2c_cmd_link_delete(cmd);
fail:
  return err;
}

static esp_err_t bh1750_reset(struct bh1750* bh) {
  esp_err_t err = bh1750_cmd(bh, BH1750_CMD_POWER_ON);
  if(err) {
    return err;
  }
  vTaskDelay(100 / portTICK_PERIOD_MS);
  err = bh1750_cmd(bh, BH1750_CMD_RESET);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  if(err) {
    return err;
  }
  bh->mode = BH1750_MODE_POWER_ON;

  return bh1750_set_mt(bh, BH1750_MT_DEFAULT);
}

static esp_err_t bh1750_read_result(struct bh1750* bh, uint16_t* res) {
  uint8_t tmp[2];
	esp_err_t err;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  if(!cmd) {
    err = ESP_ERR_NO_MEM;
    goto fail;
  }
  if((err = i2c_master_start(cmd))) {
    goto fail_link;
  }
  if((err = i2c_master_write_byte(cmd, (bh->i2c_addr << 1) | 1, 1))) {
    goto fail_link;
  }
  if((err = i2c_master_read(cmd, tmp, sizeof(tmp), 0))) {
    goto fail_link;
  }
  if((err = i2c_master_stop(cmd))) {
    goto fail_link;
  }
  err = i2c_bus_cmd_begin(bh->i2c_bus, cmd, 100 / portTICK_RATE_MS);
  if(!err) {
    *res = tmp[0] << 8 | tmp[1];
  }
fail_link:
  i2c_cmd_link_delete(cmd);
fail:
  return err;
}


// Public API
esp_err_t bh1750_init(struct bh1750* bh, struct i2c_bus* i2c_bus, uint8_t i2c_addr) {
  memset(bh, 0, sizeof(*bh));
  bh->i2c_bus = i2c_bus;
  bh->i2c_addr = i2c_addr;
  return bh1750_reset(bh);
}

esp_err_t bh1750_cont_hires(struct bh1750* bh) {
  esp_err_t err = bh1750_cmd(bh, BH1750_CMD_CONT_HIRES);
  if(!err) {
    bh->mode = BH1750_MODE_CONT_HIRES;
  }
  return err;
}

esp_err_t bh1750_cont_hires2(struct bh1750* bh) {
  esp_err_t err = bh1750_cmd(bh, BH1750_CMD_CONT_HIRES2);
  if(!err) {
    bh->mode = BH1750_MODE_CONT_HIRES2;
  }
  return err;
}

esp_err_t bh1750_single_hires(struct bh1750* bh) {
  esp_err_t err = bh1750_cmd(bh, BH1750_CMD_SINGLE_HIRES);
  if(!err) {
    bh->mode = BH1750_MODE_SINGLE_HIRES;
  }
  return err;
}

esp_err_t bh1750_single_hires2(struct bh1750* bh) {
  esp_err_t err = bh1750_cmd(bh, BH1750_CMD_SINGLE_HIRES2);
  if(!err) {
    bh->mode = BH1750_MODE_SINGLE_HIRES2;
  }
  return err;
}


esp_err_t bh1750_set_mt(struct bh1750* bh, uint8_t val) {
  esp_err_t err;

  if(val < BH1750_MT_MIN || val > BH1750_MT_MAX) {
    return ESP_ERR_INVALID_ARG;
  }

  err = bh1750_cmd(bh, BH1750_CMD_CHANGE_MT_H | ((val & BH1750_MT_MASK_H) >> 5));
  if(err) {
    return err;
  }

  bh->mtreg &= BH1750_MT_MASK_L;
  bh->mtreg |= BH1750_MT_MASK_H & val;
  err = bh1750_cmd(bh, BH1750_CMD_CHANGE_MT_L | (val & BH1750_MT_MASK_L));
  if(err) {
    return err;
  }

  bh->mtreg = val;
  return err;
}


uint16_t bh1750_get_mt_ms(struct bh1750* bh) {
  uint16_t time_ms = BH1750_MT_LORES_MS;
  if(BH1750_MODE_IS_HIRES(bh->mode)) {
    time_ms = BH1750_MT_HIRES_MS;
  }

  time_ms *= (uint16_t)bh->mtreg;
  time_ms /= BH1750_MT_DEFAULT;
  return time_ms;
}


esp_err_t bh1750_measure_raw(struct bh1750* bh, uint16_t* res) {
  esp_err_t err;
  if(!BH1750_MODE_IS_CONT(bh->mode)) {
    err = bh1750_single_hires(bh);
    if(err) {
      return err;
    }
    vTaskDelay(bh1750_get_mt_ms(bh) / portTICK_PERIOD_MS);
  }
  err = bh1750_read_result(bh, res);
  if(!BH1750_MODE_IS_SINGLE(bh->mode)) {
    bh->mode = BH1750_MODE_POWER_DOWN;
  }
  return err;
}


esp_err_t bh1750_measure(struct bh1750* bh, float* res) {
  uint16_t raw;
  esp_err_t err = bh1750_measure_raw(bh, &raw);
  if(err) {
    return err;
  }
  *res = raw;
  *res /= 1.2;
  *res *= (float)BH1750_MT_DEFAULT;
  *res /= (float)bh->mtreg;
  if(BH1750_MODE_IS_HIRES2(bh->mode)) {
    *res /= 2;
  }
  return err;
}