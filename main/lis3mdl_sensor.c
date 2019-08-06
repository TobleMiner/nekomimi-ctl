#include <stdlib.h>

#include "i2c_bus.h"
#include "lis3mdl_sensor.h"


#define LIS3MDL_SENSOR_FROM_SENSOR(sensor) (container_of((sensor), struct lis3mdl_sensor, sensor))

static esp_err_t lis3mdl_sensor_alloc(struct sensor** sensor) {
  struct lis3mdl_sensor* lis_sensor = malloc(sizeof(struct lis3mdl_sensor));
  if(!lis_sensor) {
    return ESP_ERR_NO_MEM;
  }
  *sensor = &lis_sensor->sensor;
  return ESP_OK;
}

static void lis3mdl_sensor_free(struct sensor* sensor) {
  free(LIS3MDL_SENSOR_FROM_SENSOR(sensor));
}

static esp_err_t lis3mdl_sensor_get(struct sensor* sensor, sensor_param_t param, sensor_result_t* res, size_t len) {
  struct lis3mdl_result lisres;
  lis3mdl_service_measure_raw(&LIS3MDL_SENSOR_FROM_SENSOR(sensor)->lis, &lisres);
  *res++ = lisres.x;
  *res++ = lisres.y;
  *res = lisres.z;
  return ESP_OK;
}

static void lis3mdl_sensor_cb(void* priv) {
  struct sensor* sensor = priv;
  sensor_result_t res[3];
  lis3mdl_sensor_get(sensor, SENSOR_PARAM_BFIELD, res, sizeof(res));
  sensors_report_result(sensor, SENSOR_PARAM_BFIELD, res, sizeof(res));
}

static esp_err_t lis3mdl_sensor_init(struct sensor* sensor, struct i2c_bus* bus, uint8_t i2c_addr, void* args) {
  esp_err_t err;
  struct lis3mdl_sensor_args* lisargs = args;
  struct lis3mdl_service* lis = &LIS3MDL_SENSOR_FROM_SENSOR(sensor)->lis;
  err = lis3mdl_service_init(lis, bus, i2c_addr, lisargs->gpio_drdy);
  if(err) {
    return err;
  }

  lis3mdl_service_set_cb(lis, lis3mdl_sensor_cb, sensor);
  return ESP_OK;
}

struct sensor_def lis3mdl_sensor_def = {
  .name = "LIS3MDL",
  .type = SENSOR_PARAM_BFIELD,
  .ops = {
    .alloc = lis3mdl_sensor_alloc,
    .free = lis3mdl_sensor_free,
    .init = lis3mdl_sensor_init,
    .get = lis3mdl_sensor_get,
  },
};
