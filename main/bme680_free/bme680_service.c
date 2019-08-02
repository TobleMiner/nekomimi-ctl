#include <stdint.h>
#include <string.h>

#include <freertos/FreeRTOS.h>

#include <esp_err.h>
#include <esp_log.h>

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
    // Wait for result
    vTaskDelay(meas_dur_ms / portTICK_PERIOD_MS);

    // Get result
    err = bme680_get_sensor_data(&meas, &service->bme.bme);
    if(!err) {
      xSemaphoreTake(service->lock, portMAX_DELAY);
      service->res.temperature = meas.temperature;
      service->res.humidity = meas.humidity;
      service->res.pressure = meas.pressure;
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

esp_err_t bme680_service_init(struct bme680_service* service, struct i2c_bus* bus, uint8_t i2c_addr) {
  esp_err_t err;
  memset(service, 0, sizeof(*service));

  err = bme_init(&service->bme, bus, i2c_addr);
  if(err) {
    ESP_LOGE(BME680_SERVICE_TAG, "Failed to initialize BME680");
    goto fail;
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

void bme680_service_measure(struct bme680_service* service, struct bme680_service_data* meas) {
  xSemaphoreTake(service->lock, portMAX_DELAY);
  *meas = service->res;
  xSemaphoreGive(service->lock);
}

void bme680_service_set_cb(struct bme680_service* service, bme680_service_cb cb, void* priv) {
  service->cb_priv = priv;
  __sync_synchronize();
  service->cb = cb;
}
