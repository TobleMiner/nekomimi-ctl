#pragma once

#include "sdkconfig.h"

#include "sensor.h"
#include "bme680_service.h"

struct bme680_sensor {
  struct sensor sensor;
  struct bme680_service bme;
};

extern struct sensor_def bme680_sensor_def;
