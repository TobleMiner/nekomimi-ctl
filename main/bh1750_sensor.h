#pragma once

#include "sensor.h"
#include "bh1750_service.h"

struct bh1750_sensor {
  struct sensor sensor;
  struct bh1750_service bh;
};

extern struct sensor_def bh1750_sensor_def;