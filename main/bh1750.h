#pragma once

#include <stdint.h>

#include <driver/i2c.h>
#include <esp_err.h>

#include "i2c_bus.h"

#define BH1750_ADDR_L            0b0100011
#define BH1750_ADDR_H            0b1011100

#define BH1750_CMD_POWER_DOWN    0b00000000
#define BH1750_CMD_POWER_ON      0b00000001
#define BH1750_CMD_RESET         0b00000111
#define BH1750_CMD_CONT_HIRES    0b00010000
#define BH1750_CMD_CONT_HIRES2   0b00010001
#define BH1750_CMD_CONT_LORES    0b00010011
#define BH1750_CMD_SINGLE_HIRES  0b00100000
#define BH1750_CMD_SINGLE_HIRES2 0b00100001
#define BH1750_CMD_SINGLE_LORES  0b00100011
#define BH1750_CMD_CHANGE_MT_H   0b01000000
#define BH1750_CMD_CHANGE_MT_L   0b01100000

#define BH1750_MT_DEFAULT        69
#define BH1750_MT_MIN            31
#define BH1750_MT_MAX            254

#define BH1750_MT_MASK_H         0b11100000
#define BH1750_MT_MASK_L         0b00011111
#define BH1750_MT_DEFAULT        69

#define BH1750_MT_HIRES_MS       180
#define BH1750_MT_LORES_MS       24


enum bh1750_mode {
  BH1750_MODE_UNKNOWN       = 0b00000,
  BH1750_MODE_POWER_DOWN    = 0b00001,
  BH1750_MODE_POWER_ON      = 0b00010,
  BH1750_MODE_CONT_LORES    = 0b01000,
  BH1750_MODE_CONT_HIRES    = 0b01100,
  BH1750_MODE_CONT_HIRES2   = 0b01110,
  BH1750_MODE_SINGLE_LORES  = 0b10000,
  BH1750_MODE_SINGLE_HIRES  = 0b10100,
  BH1750_MODE_SINGLE_HIRES2 = 0b10110,
};

#define BH1750_MODE_MASK_CONT   0b01000
#define BH1750_MODE_MASK_SINGLE 0b10000
#define BH1750_MODE_MASK_HIRES  0b00100
#define BH1750_MODE_MASK_HIRES2 0b00110

#define BH1750_MODE_IS_CONT(mode) ((mode) & BH1750_MODE_MASK_CONT)
#define BH1750_MODE_IS_SINGLE(mode) ((mode) & BH1750_MODE_MASK_SINGLE)
#define BH1750_MODE_IS_HIRES(mode) ((mode) & BH1750_MODE_MASK_HIRES)
#define BH1750_MODE_IS_HIRES2(mode) \
  (((mode) & BH1750_MODE_MASK_HIRES2) == BH1750_MODE_MASK_HIRES2)

struct bh1750 {
  struct i2c_bus*  i2c_bus;
  uint8_t          i2c_addr;
  uint8_t          mtreg;
  enum bh1750_mode mode;
};

esp_err_t bh1750_init(struct bh1750* bh, struct i2c_bus* i2c_bus, uint8_t i2c_addr);

esp_err_t bh1750_cont_hires(struct bh1750* bh);
esp_err_t bh1750_cont_hires2(struct bh1750* bh);

esp_err_t bh1750_single_hires(struct bh1750* bh);
esp_err_t bh1750_single_hires2(struct bh1750* bh);

esp_err_t bh1750_set_mt(struct bh1750* bh, uint8_t val);
uint16_t bh1750_get_mt_ms(struct bh1750* bh);

esp_err_t bh1750_measure_raw(struct bh1750* bh, uint16_t* val);
esp_err_t bh1750_measure(struct bh1750* bh, float* res);
