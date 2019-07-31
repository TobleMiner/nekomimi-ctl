#include <stdlib.h>

#include "i2c_bus.h"
#include "bh1750_sensor.h"


#define BH1750_SENSOR_FROM_SENSOR(sensor) (container_of((sensor), struct bh1750_sensor, sensor))

static esp_err_t bh1750_sensor_alloc(struct sensor** sensor) {
  struct bh1750_sensor* bh_sensor = malloc(sizeof(struct bh1750_sensor));
  if(!bh_sensor) {
    return ESP_ERR_NO_MEM;
  }
  *sensor = &bh_sensor->sensor;
  return ESP_OK;
}

static void bh1750_sensor_free(struct sensor* sensor) {
  free(BH1750_SENSOR_FROM_SENSOR(sensor));
}

static esp_err_t bh1750_sensor_init(struct sensor* sensor, struct i2c_bus* bus, uint8_t i2c_addr) {
  return bh1750_service_init(&BH1750_SENSOR_FROM_SENSOR(sensor)->bh, bus, i2c_addr);
}

static esp_err_t bh1750_sensor_get(struct sensor* sensor, sensor_param_t param, sensor_result_t* res, size_t len) {
  *res = bh1750_service_get_illuminance(&BH1750_SENSOR_FROM_SENSOR(sensor)->bh);
  return ESP_OK;
}

struct sensor_def bh1750_sensor_def = {
  .name = "BH1750",
  .type = SENSOR_PARAM_ILLUMINANCE,
  .ops = {
    .alloc = bh1750_sensor_alloc,
    .free = bh1750_sensor_free,
    .init = bh1750_sensor_init,
    .get = bh1750_sensor_get,
  },
};