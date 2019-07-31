#include <stdlib.h>
#include <string.h>

#include <esp_log.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "bh1750_service.h"

static void bh1750_service_task(void* arg) {
  struct bh1750_service* service = arg;
  bh1750_cont_hires2(&service->bh);

  while(1) {
    esp_err_t err;
    float res;

    // Wait for completion of measurement
    vTaskDelay(bh1750_get_mt_ms(&service->bh) / portTICK_PERIOD_MS);

    err = bh1750_measure(&service->bh, &res);
    if(!err) {
      xSemaphoreTake(service->lock, portMAX_DELAY);
      service->illuminance = res;
      xSemaphoreGive(service->lock);
    }
  }
}

esp_err_t bh1750_service_init(struct bh1750_service* service, struct i2c_bus* bus, uint8_t i2c_addr) {
  esp_err_t err;
  memset(service, 0, sizeof(*service));
  
  err = bh1750_init(&service->bh, bus, i2c_addr);
  if(err) {
    ESP_LOGE(BH1750_SERVICE_TAG, "Failed to initilalize bh1750");
    goto fail;
  }

  service->lock = xSemaphoreCreateMutex();
  if(!service->lock) {
    ESP_LOGE(BH1750_SERVICE_TAG, "Failed to allocate mutex");
    err = ESP_ERR_NO_MEM;
    goto fail;
  }

  err = xTaskCreate(bh1750_service_task, "illuminance_srv", BH1750_SERVICE_STACK, service, 12, NULL);
  if(err != pdPASS) {
    ESP_LOGE(BH1750_SERVICE_TAG, "Failed to allocate task");
    goto fail_mutex;
  }

  return ESP_OK;

fail_mutex:
  vSemaphoreDelete(service->lock);
fail:
  return err;  
}

float bh1750_service_get_illuminance(struct bh1750_service* service) {
  float res;
  xSemaphoreTake(service->lock, portMAX_DELAY);
  res = service->illuminance;
  xSemaphoreGive(service->lock);
  return res;
}