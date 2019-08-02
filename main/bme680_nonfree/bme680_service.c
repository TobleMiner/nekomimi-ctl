#include <stdint.h>
#include <string.h>

#include <freertos/FreeRTOS.h>

#include <esp_err.h>
#include <esp_log.h>

#include "../util.h"
#include "bme680_service.h"

static esp_err_t bme680_service_read_data(int64_t timestamp, struct bme680* bme, bsec_input_t* inputs, uint8_t* num_inputs, int32_t bsec_flags) {
  esp_err_t err;
  struct bme680_field_data data;
  unsigned int i = 0;

  *num_inputs = 0;

  err = bme680_get_sensor_data(&data, &bme->bme);
  if(err) {
    return err;
  }

  if(!(data.status & BME680_NEW_DATA_MSK)) {
    return ESP_OK;
  }

  if(bsec_flags & BSEC_PROCESS_TEMPERATURE) {
    inputs[i].sensor_id = BSEC_INPUT_TEMPERATURE;
    inputs[i].signal = data.temperature;
    inputs[i].time_stamp = timestamp;
    i++;
  }

  if(bsec_flags & BSEC_PROCESS_HUMIDITY) {
    inputs[i].sensor_id = BSEC_INPUT_HUMIDITY;
    inputs[i].signal = data.humidity;
    inputs[i].time_stamp = timestamp;
    i++;
  }

  if(bsec_flags & BSEC_PROCESS_PRESSURE) {
    inputs[i].sensor_id = BSEC_INPUT_PRESSURE;
    inputs[i].signal = data.pressure;
    inputs[i].time_stamp = timestamp;
    i++;
  }

  if(bsec_flags & BSEC_PROCESS_GAS) {
    inputs[i].sensor_id = BSEC_INPUT_GASRESISTOR;
    inputs[i].signal = data.gas_resistance;
    inputs[i].time_stamp = timestamp;
    i++;
  }

  *num_inputs = i;
  return ESP_OK;
}

esp_err_t bme680_service_process_data(bsec_input_t *inputs, uint8_t num_inputs, struct bme680_service_data* data) {
  esp_err_t err;
  bsec_output_t outputs[BSEC_NUMBER_OUTPUTS];
  uint8_t num_outputs = ARRAY_LEN(outputs);

  if(num_inputs == 0) {
    return ESP_OK;
  }

  err = bsec_do_steps(inputs, num_inputs, outputs, &num_outputs);
  if(err) {
    return err;
  }

  while(num_outputs--) {
    ESP_LOGI(BME680_SERVICE_TAG, "Raw value of output %u: %f", num_outputs, outputs[num_outputs].signal);
    switch(outputs[num_outputs].sensor_id) {
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
        data->temperature = outputs[num_outputs].signal;
        data->temperature_ready = true;
        break;
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
        data->humidity = outputs[num_outputs].signal;
        data->humidity_ready = true;
        break;
      case BSEC_OUTPUT_RAW_PRESSURE:
        data->pressure = outputs[num_outputs].signal;
        data->pressure_ready = true;
        break;
      case BSEC_OUTPUT_IAQ:
        data->iaq = outputs[num_outputs].signal;
        data->iaq_ready = true;
        break;
    }
  }

  return err;
}

static void bme680_service_task(void* arg) {
  struct bme680_service* service = arg;
  bsec_bme_settings_t bme_settings;
  bsec_input_t bsec_inputs[BSEC_MAX_PHYSICAL_SENSOR];

  while(1) {
    uint8_t num_inputs = 0;
    int64_t measure_begin = esp_timer_get_time() * 1000LL;
    struct bme680_service_data data;
    bool updated = false;
    bsec_sensor_control(measure_begin, &bme_settings);

    if(bme_settings.trigger_measurement) {
      uint16_t meas_dur_ms;
      ESP_LOGI(BME680_SERVICE_TAG, "Triggering measurement");

      service->bme.bme.power_mode = BME680_FORCED_MODE;

      service->bme.bme.tph_sett.os_temp = bme_settings.temperature_oversampling;
      service->bme.bme.tph_sett.os_hum = bme_settings.humidity_oversampling;
      service->bme.bme.tph_sett.os_pres = bme_settings.pressure_oversampling;

      service->bme.bme.gas_sett.run_gas = bme_settings.run_gas;
      service->bme.bme.gas_sett.heatr_temp = bme_settings.heater_temperature;
      service->bme.bme.gas_sett.heatr_dur = bme_settings.heating_duration;

      bme680_set_sensor_settings(BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_GAS_SENSOR_SEL, &service->bme.bme);
      bme680_set_sensor_mode(&service->bme.bme);
      service->state = BME680_SERVICE_STATE_MEASURE;

      bme680_get_profile_dur(&meas_dur_ms, &service->bme.bme);
      esp_timer_start_once(service->timer, (uint32_t)meas_dur_ms * 1000UL);
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

      bme680_get_sensor_mode(&service->bme.bme);
      while(service->bme.bme.power_mode == BME680_FORCED_MODE) {
        ESP_LOGI(BME680_SERVICE_TAG, "Waiting for measurement completion");
        esp_timer_start_once(service->timer, 5000UL);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        bme680_get_sensor_mode(&service->bme.bme);
      }
      ESP_LOGI(BME680_SERVICE_TAG, "Measurement done");
    }    

    // Measurement finished
    if(bme_settings.process_data) {
      ESP_LOGI(BME680_SERVICE_TAG, "Reading measured data");
      bme680_service_read_data(measure_begin, &service->bme, bsec_inputs, &num_inputs, bme_settings.process_data);
      ESP_LOGI(BME680_SERVICE_TAG, "Got %u data", num_inputs);
    }

    bme680_service_process_data(bsec_inputs, num_inputs, &data);


    xSemaphoreTake(service->lock, portMAX_DELAY);
    if(data.temperature_ready) {
      service->res.temperature = data.temperature;
      service->res.temperature_ready = true;
      updated = true;
    }
    if(data.humidity_ready) {
      service->res.humidity = data.humidity;
      service->res.humidity_ready = true;
      updated = true;
    }
    if(data.pressure_ready) {
      service->res.pressure = data.pressure;
      service->res.pressure_ready = true;
      updated = true;
    }
    if(data.iaq_ready) {
      service->res.iaq = data.iaq;
      service->res.iaq_ready = true;
      updated = true;
    }
    xSemaphoreGive(service->lock);  
    if(updated && service->cb) {
      service->cb(service->cb_priv);
    }

    int64_t measure_wait = (bme_settings.next_call - esp_timer_get_time() * 1000LL) / 1000LL;
    if(measure_wait > 0) {
        ESP_LOGI(BME680_SERVICE_TAG, "Sleeping for %lld us", measure_wait);
        esp_timer_start_once(service->timer, measure_wait);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);    
    }
  }
}

static void bme680_service_timer_callback(void* arg) {
  struct bme680_service* service = arg;
  xTaskNotifyGive(service->task);
}

static esp_err_t bme680_service_bsec_update_subscription(struct bme680_service_rate* rate, bsec_sensor_configuration_t* required_config, uint8_t* num_required_config) {
  bsec_sensor_configuration_t requested_virtual_sensors[BSEC_MAX_PHYSICAL_SENSOR];

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
    .temperature = BSEC_SAMPLE_RATE_LP,
    .humidity = BSEC_SAMPLE_RATE_LP,
    .pressure = BSEC_SAMPLE_RATE_LP,
    .iaq = BSEC_SAMPLE_RATE_LP,
  };
  esp_timer_create_args_t timer_args = {
    .callback = bme680_service_timer_callback,
    .arg = service,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "BME680_BSEC_TIMER",
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
    goto fail_timer;
  }

  service->num_required_sensor_settings = ARRAY_LEN(service->required_sensor_settings);
  err = bme680_service_bsec_update_subscription(&rate, service->required_sensor_settings, &service->num_required_sensor_settings);
  if(err) {
    ESP_LOGE(BME680_SERVICE_TAG, "Failed to set intial BME680 BSEC subscription");
    goto fail_timer;
  }

  service->lock = xSemaphoreCreateMutex();
  if(!service->lock) {
    ESP_LOGE(BME680_SERVICE_TAG, "Failed to allocate lock mutex");
    err = ESP_ERR_NO_MEM;
    goto fail_timer;
  }

  err = xTaskCreate(bme680_service_task, "bme680_srv", BME680_SERVICE_STACK, service, 12, &service->task);
  if(err != pdPASS) {
    ESP_LOGE(BME680_SERVICE_TAG, "Failed to initialize service task");
    err = ESP_ERR_NO_MEM;
    goto fail_lock;
  }

  return ESP_OK;

fail_lock:
  vSemaphoreDelete(service->lock);
fail_timer:
  esp_timer_delete(service->timer);
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
