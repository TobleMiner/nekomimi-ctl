#include <stdint.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <esp_err.h>
#include <esp_log.h>

#include "lis3mdl_service.h"

static void IRAM_ATTR lis3mdl_drdy_isr(void* priv) {
  struct lis3mdl_service* service = priv;
  BaseType_t high_prio_task_woken = pdFALSE;
  vTaskNotifyGiveFromISR(service->task, &high_prio_task_woken);
  if(high_prio_task_woken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

static void lis3mdl_service_task(void* arg) {
  struct lis3mdl_service* service = arg;
  while(1) {
    struct lis3mdl_result res;
    esp_err_t err;
    ulTaskNotifyTake(pdPASS, portMAX_DELAY);
    err = lis3mdl_measure_raw(&service->lis, &res);
    if(!err) {
      xSemaphoreTake(service->lock, portMAX_DELAY);
      service->res = res;
      xSemaphoreGive(service->lock);
    }
  }
}


esp_err_t lis3mdl_service_init(struct lis3mdl_service* service, struct i2c_bus* bus, uint8_t i2c_addr, int drdy_gpio) {
  esp_err_t err;
  gpio_config_t gpio_conf = { 0 };

  memset(service, 0, sizeof(*service));

  service->drdy_gpio = drdy_gpio;
  gpio_conf.intr_type    = GPIO_PIN_INTR_POSEDGE;
  gpio_conf.pin_bit_mask = 1ULL << drdy_gpio;
  gpio_conf.mode         = GPIO_MODE_INPUT;
  err = gpio_config(&gpio_conf);
  if(err) {
    ESP_LOGE(LIS3MDL_SERVICE_TAG, "Failed to configure data ready irq gpio");
    goto fail;
  }
  err = gpio_install_isr_service(0);
  if(err && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(LIS3MDL_SERVICE_TAG, "Failed to set up gpio isr service");
    goto fail;
  }
  err = gpio_isr_handler_add(drdy_gpio, lis3mdl_drdy_isr, service);
  if(err) {
    ESP_LOGE(LIS3MDL_SERVICE_TAG, "Failed to attach isr handler");
    goto fail;
  }

  service->lock = xSemaphoreCreateMutex();
  if(!service->lock) {
    ESP_LOGE(LIS3MDL_SERVICE_TAG, "Failed to allocate lock mutex");
    err = ESP_ERR_NO_MEM;
    goto fail_isr;
  }

  err = lis3mdl_init(&service->lis, bus, i2c_addr);
  if(err) {
    ESP_LOGE(LIS3MDL_SERVICE_TAG, "Failed to initialize lis3mdl");
    goto fail_lock;
  }

  err = xTaskCreate(lis3mdl_service_task, "compass_srv", LIS3MDL_SERVICE_STACK, service, 12, &service->task);
  if(err != pdPASS) {
    ESP_LOGE(LIS3MDL_SERVICE_TAG, "Failed to initialze service task");
    err = ESP_ERR_NO_MEM;
    goto fail_lock;
  }

  xTaskNotifyGive(service->task);

  return ESP_OK;

fail_lock:
  vSemaphoreDelete(service->lock);
fail_isr:
  gpio_isr_handler_remove(drdy_gpio);
fail:
  return err;
}

void lis3mdl_service_measure_raw(struct lis3mdl_service* service, struct lis3mdl_result* res) {
  xSemaphoreTake(service->lock, portMAX_DELAY);
  *res = service->res;
  xSemaphoreGive(service->lock);
}
