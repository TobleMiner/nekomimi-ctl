#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include <esp_log.h>

#include "sensor.h"

sensor_param_type_t sensors_param_get_type(sensor_param_t param) {
  switch(param) {
    case SENSOR_PARAM_TEMPERATURE:
    case SENSOR_PARAM_HUMIDITY:
    case SENSOR_PARAM_PRESSURE:
    case SENSOR_PARAM_ILLUMINANCE:
      return SENSOR_PARAM_TYPE_UNARY;
    case SENSOR_PARAM_HFIELD:
      return SENSOR_PARAM_TYPE_TERNARY;

    default:
      return 0;
  }
}

esp_err_t sensors_init(struct sensor_manager* mgr) {
  esp_err_t err;
  memset(mgr, 0, sizeof(*mgr));
  INIT_LIST_HEAD(mgr->sensors);
  INIT_LIST_HEAD(mgr->subscribers);

  mgr->lock = xSemaphoreCreateMutex();
  if(!mgr->lock) {
    err = ESP_ERR_NO_MEM;
    ESP_LOGE(SENSOR_TAG, "Failed to allocate lock");
    return err;
  }

  return ESP_OK;
}

esp_err_t sensors_add_sensor(struct sensor_manager* mgr, struct sensor_def* def, struct i2c_bus* bus, uint8_t i2c_addr) {
  esp_err_t err;
  struct sensor* sensor;

  err = def->ops.alloc(&sensor);
  if(err) {
    goto fail;
  }

  memset(sensor, 0, sizeof(*sensor));
  INIT_LIST_HEAD(sensor->list);
  sensor->mgr = mgr;
  sensor->def = def;

  err = def->ops.init(sensor, bus, i2c_addr);
  if(err) {
    goto fail_alloc;
  }

  xSemaphoreTake(mgr->lock, portMAX_DELAY);
  LIST_APPEND(&sensor->list, &mgr->sensors);
  xSemaphoreGive(mgr->lock);

  return ESP_OK;

fail_alloc:
  sensor->def->ops.free(sensor);
fail:
  return err;
}

esp_err_t sensors_subscribe(struct sensor_manager* mgr, sensor_param_t param, sensor_subscriber_cb cb, void* priv) {
  esp_err_t err;
  struct sensor_subscriber* sub = malloc(sizeof(struct sensor_subscriber));
  if(!sub) {
    err = ESP_ERR_NO_MEM;
    return err;
  }

  INIT_LIST_HEAD(sub->list);
  sub->param = param;
  sub->cb = cb;
  sub->priv = priv;

  xSemaphoreTake(mgr->lock, portMAX_DELAY);
  LIST_APPEND(&sub->list, &mgr->subscribers);
  xSemaphoreGive(mgr->lock);

  return ESP_OK;
}

esp_err_t sensors_get_result(struct sensor_manager* mgr, sensor_param_t param, sensor_result_t* res, size_t len) {
  esp_err_t err;
  struct list_head* cursor;
  if(sensors_param_get_type(param) * sizeof(sensor_result_t) != len) {
    err = ESP_ERR_INVALID_ARG;
    return err;
  }

  LIST_FOR_EACH(cursor, &mgr->sensors) {
    struct sensor* sensor = LIST_GET_ENTRY(cursor, struct sensor, list);
    if(sensor->def->type & param) {
      return sensor->def->ops.get(sensor, param, res, len);
    }
  }

  return ESP_ERR_NOT_FOUND;
}

esp_err_t sensors_report_result(struct sensor* sensor, sensor_param_t param, sensor_result_t* res, size_t len) {
  esp_err_t err;
  struct sensor_manager* mgr = sensor->mgr;
  struct list_head* cursor;
  if(sensors_param_get_type(param) * sizeof(sensor_result_t) != len) {
    err = ESP_ERR_INVALID_ARG;
    return err;
  }

  xSemaphoreTake(mgr->lock, portMAX_DELAY);
  LIST_FOR_EACH(cursor, &mgr->subscribers) {
    struct sensor_subscriber* sub = LIST_GET_ENTRY(cursor, struct sensor_subscriber, list);
    if(sub->param & param) {
      sub->cb(mgr, sensor, param, res, len);
    }
  }
  xSemaphoreGive(mgr->lock);
  
  return ESP_OK;
}