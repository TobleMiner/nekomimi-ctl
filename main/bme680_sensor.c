#include <stdlib.h>

#include "i2c_bus.h"
#include "bme680_sensor.h"


#define BME680_SENSOR_FROM_SENSOR(sensor) (container_of((sensor), struct bme680_sensor, sensor))

static esp_err_t bme680_sensor_alloc(struct sensor** sensor) {
  struct bme680_sensor* bme_sensor = malloc(sizeof(struct bme680_sensor));
  if(!bme_sensor) {
    return ESP_ERR_NO_MEM;
  }
  *sensor = &bme_sensor->sensor;
  return ESP_OK;
}

static void bme680_sensor_free(struct sensor* sensor) {
  free(BME680_SENSOR_FROM_SENSOR(sensor));
}

static esp_err_t bme680_sensor_get(struct sensor* sensor, sensor_param_t param, sensor_result_t* res, size_t len) {
  struct bme680_service_data meas;
  bme680_service_measure(&BME680_SENSOR_FROM_SENSOR(sensor)->bme, &meas);
  switch(param) {
    case SENSOR_PARAM_TEMPERATURE:
      *res = meas.temperature;
      break;
    case SENSOR_PARAM_HUMIDITY:
      *res = meas.humidity;
      break;
    case SENSOR_PARAM_PRESSURE:
      *res = meas.pressure;
      break;
#ifdef CONFIG_NEKOMIMI_BME680_ALGO_PROPRIETARY
    case SENSOR_PARAM_IAQ:
      *res = meas.iaq;
      break;
#endif
    default:
      return ESP_ERR_INVALID_ARG;
  }
  return ESP_OK;
}


#ifdef CONFIG_NEKOMIMI_BME680_ALGO_PROPRIETARY
static esp_err_t bme680_sensor_ready(struct sensor* sensor, sensor_param_t param, bool* ready) {
  struct bme680_service_data meas;
  bme680_service_measure(&BME680_SENSOR_FROM_SENSOR(sensor)->bme, &meas);
  switch(param) {
    case SENSOR_PARAM_TEMPERATURE:
      *ready = meas.temperature_ready;
      break;
    case SENSOR_PARAM_HUMIDITY:
      *ready = meas.humidity_ready;
      break;
    case SENSOR_PARAM_PRESSURE:
      *ready = meas.pressure_ready;
      break;
    case SENSOR_PARAM_IAQ:
      *ready = meas.iaq_ready;
      break;
    default:
      return ESP_ERR_INVALID_ARG;
  }
  return ESP_OK;
}
#else
static esp_err_t bme680_sensor_ready(struct sensor* sensor, sensor_param_t param, bool* ready) {
  *ready = true;
  return ESP_OK;
}
#endif

static void bme680_sensor_cb(void* priv) {
  struct sensor* sensor = priv;
  sensor_result_t res;
  bool ready;
  uint8_t i = 0;

#ifdef CONFIG_NEKOMIMI_BME680_ALGO_PROPRIETARY
  sensor_param_t params[] = { SENSOR_PARAM_TEMPERATURE, SENSOR_PARAM_HUMIDITY, SENSOR_PARAM_PRESSURE, SENSOR_PARAM_IAQ };
#else
  sensor_param_t params[] = { SENSOR_PARAM_TEMPERATURE, SENSOR_PARAM_HUMIDITY, SENSOR_PARAM_PRESSURE };
#endif
  
  for(i = 0; i < ARRAY_LEN(params); i++) { 
    bme680_sensor_ready(sensor, params[i], &ready);
    if(ready) {
      bme680_sensor_get(sensor, params[i], &res, sizeof(res));
    }
  }
}

static esp_err_t bme680_sensor_init(struct sensor* sensor, struct i2c_bus* bus, uint8_t i2c_addr, void* args) {
  esp_err_t err;
  struct bme680_service* bme = &BME680_SENSOR_FROM_SENSOR(sensor)->bme;
  err = bme680_service_init(bme, bus, i2c_addr);
  if(err) {
    return err;
  }

  bme680_service_set_cb(bme, bme680_sensor_cb, sensor);
  return ESP_OK;
}

struct sensor_def bme680_sensor_def = {
  .name = "BME680",
#ifdef CONFIG_NEKOMIMI_BME680_ALGO_PROPRIETARY
  .type = SENSOR_PARAM_TEMPERATURE | SENSOR_PARAM_HUMIDITY | SENSOR_PARAM_PRESSURE | SENSOR_PARAM_IAQ,
#else
  .type = SENSOR_PARAM_TEMPERATURE | SENSOR_PARAM_HUMIDITY | SENSOR_PARAM_PRESSURE,
#endif
  .ops = {
    .alloc = bme680_sensor_alloc,
    .free = bme680_sensor_free,
    .init = bme680_sensor_init,
    .get = bme680_sensor_get,
  },
};
