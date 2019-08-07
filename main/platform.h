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
