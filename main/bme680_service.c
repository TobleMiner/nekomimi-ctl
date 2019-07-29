#include <stdint.h>
#include <string.h>

#include <freertos/FreeRTOS.h>

#include <esp_err.h>
#include <esp_log.h>

#include "i2c_bus.h"
#include "bme680_service.h"
#include "bme680_api.h"

static void bme680_service_task(void* arg) {
  struct bme680_service* service = arg;
  uint16_t meas_dur_ms;
  bme680_get_profile_dur(&meas_dur_ms, &service->bme);

  while(1) {
    struct bme680_field_data meas;
    esp_err_t err;
    // Wait for result
    vTaskDelay(meas_dur_ms / portTICK_PERIOD_MS);

    // Get result
    err = bme680_get_sensor_data(&meas, &service->bme);
    if(!err) {
      xSemaphoreTake(service->lock, portMAX_DELAY);
      service->res = meas;
      xSemaphoreGive(service->lock);
    }
    service->bme.power_mode = BME680_FORCED_MODE;
    bme680_set_sensor_mode(&service->bme);
  }
}

esp_err_t bme680_service_init(struct bme680_service* service, struct i2c_bus* bus, uint8_t i2c_addr) {
  esp_err_t err;
  memset(service, 0, sizeof(*service));

  service->bme.dev_id = i2c_addr;
  service->bme.intf = BME680_I2C_INTF;
  service->bme.read = bme_api_i2c_read;
  service->bme.write = bme_api_i2c_write;
  service->bme.delay_ms = bme_api_delay_ms;
  service->bme.fp_priv = bus;

  err = bme680_init(&service->bme);
  if(err) {
    ESP_LOGE(BME680_SERVICE_TAG, "Failed to initialize BME680");
    goto fail;
  }

  service->bme.tph_sett.os_hum = BME680_OS_2X;
  service->bme.tph_sett.os_pres = BME680_OS_4X;
  service->bme.tph_sett.os_temp = BME680_OS_8X;
  service->bme.tph_sett.filter = BME680_FILTER_SIZE_3;

//  service->bme.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
//  service->bme.gas_sett.heatr_temp = 320;
//  service->bme.gas_sett.heatr_dur = 150;

  service->bme.power_mode = BME680_FORCED_MODE;

  err = bme680_set_sensor_settings(BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL | BME680_GAS_SENSOR_SEL, &service->bme);
  if(err) {
    ESP_LOGE(BME680_SERVICE_TAG, "Failed to set BME680 settings");
    goto fail;
  }

  err = bme680_set_sensor_mode(&service->bme);
  if(err) {
    ESP_LOGE(BME680_SERVICE_TAG, "Failed to set BME680 mode");
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

void bme680_service_measure(struct bme680_service* service, struct bme680_field_data* meas) {
  xSemaphoreTake(service->lock, portMAX_DELAY);
  *meas = service->res;
  xSemaphoreGive(service->lock);
}
