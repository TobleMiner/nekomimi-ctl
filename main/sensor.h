#pragma once

#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <esp_err.h>

#include "i2c_bus.h"
#include "util.h"
#include "list.h"

#define SENSOR_TAG "SENSOR MANAGER"

typedef uint8_t sensor_param_t;
typedef uint8_t sensor_param_type_t;
typedef float sensor_result_t;

#define SENSOR_PARAM_TEMPERATURE 0b00000001
#define SENSOR_PARAM_HUMIDITY    0b00000010
#define SENSOR_PARAM_PRESSURE    0b00000100
#define SENSOR_PARAM_ILLUMINANCE 0b00001000
#define SENSOR_PARAM_BFIELD      0b00010000
#define SENSOR_PARAM_IAQ         0b00100000

#define SENSOR_PARAM_TYPE_UNARY   1
#define SENSOR_PARAM_TYPE_BINARY  2
#define SENSOR_PARAM_TYPE_TERNARY 3

struct sensor_manager;

struct sensor;

// sensor callback type
typedef esp_err_t (*sensor_cb)(struct sensor* sensor, sensor_param_t param, sensor_result_t* res, size_t len, void* priv);

// Sensor ops
typedef esp_err_t (*sensor_alloc)(struct sensor** sensor);
typedef esp_err_t (*sensor_init)(struct sensor* sensor, struct i2c_bus* bus, uint8_t i2c_addr, void* args);
typedef void (*sensor_free)(struct sensor* sensor);
typedef esp_err_t (*sensor_get)(struct sensor* sensor, sensor_param_t param, sensor_result_t* res, size_t len);

// subscriber callback type
typedef void (*sensor_subscriber_cb)(struct sensor_manager* mgr, struct sensor* sensor, sensor_param_t param, sensor_result_t* res, size_t len, void* priv);

struct sensor_ops {
  sensor_alloc alloc;
  sensor_init init;
  sensor_free free;
  sensor_get get;
};

struct sensor_def {
  sensor_param_t type;
  const char* name;
  struct sensor_ops ops;
};

struct sensor_manager;

struct sensor {
  struct list_head list;
  struct sensor_def* def;
  struct sensor_manager* mgr;
};

struct sensor_subscriber {
  struct list_head list;
  sensor_param_t param;
  sensor_subscriber_cb cb;
  void* priv;
};

struct sensor_manager {
  struct list_head sensors;
  struct list_head subscribers;
  SemaphoreHandle_t lock;
};

sensor_param_type_t sensors_param_get_type(sensor_param_t param);
esp_err_t sensors_init(struct sensor_manager* mgr);
esp_err_t sensors_add_sensor(struct sensor_manager* mgr, struct sensor_def* def, struct i2c_bus* bus, uint8_t i2c_addr, void* args);
esp_err_t sensors_subscribe(struct sensor_manager* mgr, sensor_param_t param, sensor_subscriber_cb cb, void* priv);
esp_err_t sensors_get_result(struct sensor_manager* mgr, sensor_param_t param, sensor_result_t* res, size_t len);
bool sensors_has_sensor(struct sensor_manager* mgr, sensor_param_t param);

esp_err_t sensors_report_result(struct sensor* sensor, sensor_param_t param, sensor_result_t* res, size_t len);
