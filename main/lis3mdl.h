#pragma once

#include <stdint.h>

#include <esp_err.h>

#include "i2c_bus.h"

#define LIS3MDL_TAG            "lis3mdl"

#define LIS3MDL_ID             0b00111101

#define LIS3MDL_ADDR_L     0b0011100
#define LIS3MDL_ADDR_H     0b0011110

#define LIS3MDL_REG_WHO_AM_I   0x0F
#define LIS3MDL_REG_CTRL_REG1  0x20
#define LIS3MDL_REG_CTRL_REG2  0x21
#define LIS3MDL_REG_CTRL_REG3  0x22
#define LIS3MDL_REG_CTRL_REG4  0x23
#define LIS3MDL_REG_CTRL_REG5  0x24
#define LIS3MDL_REG_STATUS_REG 0x27
#define LIS3MDL_REG_OUT_X_L    0x28
#define LIS3MDL_REG_OUT_X_H    0x29
#define LIS3MDL_REG_OUT_Y_L    0x2A
#define LIS3MDL_REG_OUT_Y_H    0x2B
#define LIS3MDL_REG_OUT_Z_L    0x2C
#define LIS3MDL_REG_OUT_Z_H    0x2D
#define LIS3MDL_REG_TEMP_OUT_L 0x2E
#define LIS3MDL_REG_TEMP_OUT_H 0x2F
#define LIS3MDL_REG_INT_CFG    0x30
#define LIS3MDL_REG_INT_SRC    0x31
#define LIS3MDL_REG_INT_THS_L  0x32
#define LIS3MDL_REG_INT_THS_H  0x33

#define LIS3MDL_AUTO_INCREMENT 0b10000000

enum LIS3MDL_RANGE {
  LIS3MDL_RANGE_4_GAUSS  = 0b00,
  LIS3MDL_RANGE_8_GAUSS  = 0b01,
  LIS3MDL_RANGE_12_GAUSS = 0b10,
  LIS3MDL_RANGE_16_GAUSS = 0b11,
  LIS3MDL_RANGE_LIMIT_
};

struct lis3mdl {
  struct i2c_bus*    i2c_bus;
  uint8_t            i2c_addr;
  enum LIS3MDL_RANGE range;
};

struct lis3mdl_result {
  int16_t x;
  int16_t y;
  int16_t z;
  int16_t temp;
};

esp_err_t lis3mdl_init(struct lis3mdl* lis, struct i2c_bus* i2c_bus, uint8_t i2c_addr);
esp_err_t lis3mdl_reset(struct lis3mdl* lis);

esp_err_t lis3mdl_set_range(struct lis3mdl* lis, enum LIS3MDL_RANGE range);

esp_err_t lis3mdl_measure_raw(struct lis3mdl* lis, struct lis3mdl_result* res);
