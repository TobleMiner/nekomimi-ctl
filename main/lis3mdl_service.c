#include <stdint.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <esp_err.h>
#include <esp_log.h>

#include "util.h"
#include "lis3mdl_service.h"

static void IRAM_ATTR lis3mdl_drdy_isr(void* priv) {
  struct lis3mdl_service* service = priv;
  BaseType_t high_prio_task_woken = pdFALSE;
  vTaskNotifyGiveFromISR(service->task, &high_prio_task_woken);
  if(high_prio_task_woken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

static void lis3mdl_service_remove_offset(struct lis3mdl_service* service, struct lis3mdl_result* res) {
  uint8_t i;
  for(i = 0; i < 3; i++) {
    res->axes[i] -= (service->min.axes[i] + service->max.axes[i]) / 2;
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
      uint8_t i;
      xSemaphoreTake(service->lock, portMAX_DELAY);

      // Continous calibration
      for(i = 0; i < 3; i++) {
        service->min.axes[i] = min(service->min.axes[i], res.axes[i]);
        service->max.axes[i] = max(service->max.axes[i], res.axes[i]);
      }

      lis3mdl_service_remove_offset(service, &res);
      service->res = res;
      service->avg_buf[service->avg_buf_write_ptr++] = res;
      service->avg_buf_write_ptr %= ARRAY_LEN(service->avg_buf);
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

  err = xTaskCreate(lis3mdl_service_task, "compass_srv", LIS3MDL_SERVICE_STACK, service, 12, &service->task);
  if(err != pdPASS) {
    ESP_LOGE(LIS3MDL_SERVICE_TAG, "Failed to initialize service task");
    err = ESP_ERR_NO_MEM;
    goto fail;
  }

  err = gpio_isr_handler_add(drdy_gpio, lis3mdl_drdy_isr, service);
  if(err) {
    ESP_LOGE(LIS3MDL_SERVICE_TAG, "Failed to attach isr handler");
    goto fail_task;
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

  xTaskNotifyGive(service->task);

  return ESP_OK;

fail_lock:
  vSemaphoreDelete(service->lock);
fail_isr:
  gpio_isr_handler_remove(drdy_gpio);
fail_task:
  vTaskDelete(service->task);
fail:
  return err;
}

void lis3mdl_service_measure_raw(struct lis3mdl_service* service, struct lis3mdl_result* res) {
  int64_t x = 0;
  int64_t y = 0;
  int64_t z = 0;
  int64_t temp = 0;
  xSemaphoreTake(service->lock, portMAX_DELAY);
  for(int i = 0; i < ARRAY_LEN(service->avg_buf); i++) {
    x += service->avg_buf[i].x;
    y += service->avg_buf[i].y;
    z += service->avg_buf[i].z;
    temp += service->avg_buf[i].temp;
  }
  x /= (int64_t)ARRAY_LEN(service->avg_buf);
  y /= (int64_t)ARRAY_LEN(service->avg_buf);
  z /= (int64_t)ARRAY_LEN(service->avg_buf);
  temp /= LIS3MDL_SERVICE_AVG_LEN;
  res->x = x;
  res->y = y;
  res->z = z;
  res->temp = temp;
//  *res = service->res;
  xSemaphoreGive(service->lock);
}
