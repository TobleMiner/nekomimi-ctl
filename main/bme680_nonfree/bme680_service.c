#include <stdint.h>
#include <string.h>

#include <freertos/FreeRTOS.h>

#include <esp_err.h>
#include <esp_log.h>

#include "../util.h"
#include "bme680_service.h"

static void bme680_service_task(void* arg) {
  struct bme680_service* service = arg;
  uint16_t meas_dur_ms;

  service->bme.bme.power_mode = BME680_FORCED_MODE;
  bme680_set_sensor_mode(&service->bme.bme);

  bme680_get_profile_dur(&meas_dur_ms, &service->bme.bme);

  while(1) {
    struct bme680_field_data meas;
    esp_err_t err;
    uint64_t  
    bsec_sensor_control(service->required_sensor_settings)
    // Wait for result
    vTaskDelay(meas_dur_ms / portTICK_PERIOD_MS);

    // Get result
    err = bme680_get_sensor_data(&meas, &service->bme.bme);
    if(!err) {
      xSemaphoreTake(service->lock, portMAX_DELAY);
      service->res = meas;
      xSemaphoreGive(service->lock);
      if(service->cb) {
        service->cb(service->cb_priv);
      }
    }
    service->bme.bme.power_mode = BME680_FORCED_MODE;
    bme680_set_sensor_mode(&service->bme.bme);
    vTaskDelay(BME680_SERVICE_MEASURE_INTERVAL / portTICK_PERIOD_MS);
  }
}

static void bme680_service_timer_callback(void* arg) {
  struct bme680_service* service = arg;
  bsec_bme_settings_t bme_settings;
  int64_t now = esp_timer_get_time() * 1000LL;
  bsec_sensor_control(now, &bme_settings);
}

static esp_err_t bme680_service_bsec_update_subscription(struct bme680_service_rate* rate, bsec_sensor_configuration_t* required_config, uint8_t* num_required_config) {
  bsec_sensor_configuration_t requested_virtual_sensors[NUM_USED_OUTPUTS];

  requested_virtual_sensors[0].sensor_id = BSEC_OUTPUT_IAQ;
  requested_virtual_sensors[0].sample_rate = rate->iaq;
  requested_virtual_sensors[1].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE;
  requested_virtual_sensors[1].sample_rate = rate->temperature;
  requested_virtual_sensors[2].sensor_id = BSEC_OUTPUT_RAW_PRESSURE;
  requested_virtual_sensors[2].sample_rate = rate->pressure;
  requested_virtual_sensors[3].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY;
  requested_virtual_sensors[3].sample_rate = rate->humidity;
  requested_virtual_sensors[4].sensor_id = BSEC_OUTPUT_RAW_GAS;
  requested_virtual_sensors[4].sample_rate = rate->iaq;
  requested_virtual_sensors[5].sensor_id = BSEC_OUTPUT_RAW_TEMPERATURE;
  requested_virtual_sensors[5].sample_rate = rate->temperature;
  requested_virtual_sensors[6].sensor_id = BSEC_OUTPUT_RAW_HUMIDITY;
  requested_virtual_sensors[6].sample_rate = rate->humidity;
  requested_virtual_sensors[7].sensor_id = BSEC_OUTPUT_STATIC_IAQ;
  requested_virtual_sensors[7].sample_rate = rate->iaq;

  return bsec_update_subscription(requested_virtual_sensors, ARRAY_LEN(requested_virtual_sensors), required_config, num_required_config);
}

esp_err_t bme680_service_init(struct bme680_service* service, struct i2c_bus* bus, uint8_t i2c_addr) {
  esp_err_t err;
  struct bme680_service_rate rate = {
    temperature = BSEC_SAMPLE_RATE_LP,
    humidity = BSEC_SAMPLE_RATE_LP,
    pressure = BSEC_SAMPLE_RATE_LP,
    iaq = BSEC_SAMPLE_RATE_LP,
  };
  esp_timer_create_args_t timer_args = {
    callback = bme680_service_timer_callback,
    arg = service,
    dispatch_method = ESP_TIMER_TASK,
    name = "BME680_BSEC_TIMER",
  };
  memset(service, 0, sizeof(*service));

  err = bme_init(&service->bme, bus, i2c_addr);
  if(err) {
    ESP_LOGE(BME680_SERVICE_TAG, "Failed to initialize BME680");
    goto fail;
  }

  err = esp_timer_create(&timer_args, &service->timer);
  if(err) {
    ESP_LOGE(BME680_SERVICE_TAG, "Failed to initialize BME680 timer task");
    goto fail;
  }

  err = bsec_init();
  if(err) {
    ESP_LOGE(BME680_SERVICE_TAG, "Failed to initialize BME680 BSEC lib");
    goto fail;
  }

  err = bme680_service_bsec_update_subscription(&rate, service->required_sensor_settings, &service->num_required_sensor_settings);
  if(err) {
    ESP_LOGE(BME680_SERVICE_TAG, "Failed to set intial BME680 BSEC subscription");
    return goto fail;
  }

  service->lock = xSemaphoreCreateMutex();
  if(!service->lock) {
    ESP_LOGE(BME680_SERVICE_TAG, "Failed to allocate lock mutex");
    err = ESP_ERR_NO_MEM;
    goto fail;
  }

  err = xTaskCreate(bme680_service_task, "bme680_srv", BME680_SERVICE_STACK, service, 12, NULL);
  if(err != pdPASS) {
    ESP_LOGE(BME680_SERVICE_TAG, "Failed to initialize service task");
    err = ESP_ERR_NO_MEM;
    goto fail_lock;
  }

  return ESP_OK;

fail_lock:
  vSemaphoreDelete(service->lock);
fail:
  return err;
}

void bme680_service_measure(struct bme680_service* service, struct bme680_field_data* meas) {
  xSemaphoreTake(service->lock, portMAX_DELAY);
  *meas = service->res;
  xSemaphoreGive(service->lock);
}

void bme680_service_set_cb(struct bme680_service* service, bme680_service_cb cb, void* priv) {
  service->cb_priv = priv;
  __sync_synchronize();
  service->cb = cb;
}
