#pragma once

#include <esp_err.h>

#include "sensor.h"
#include "bh1750_sensor.h"
#include "bme680_sensor.h"

extern struct sensor_manager sensors;

esp_err_t platform_init();
