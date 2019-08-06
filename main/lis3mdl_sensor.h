#pragma once

#include "sensor.h"
#include "lis3mdl_service.h"

struct lis3mdl_sensor {
  struct sensor sensor;
  struct lis3mdl_service lis;
};

struct lis3mdl_sensor_args {
  int gpio_drdy;
  int gpio_int;
};

extern struct sensor_def lis3mdl_sensor_def;
