#include <freertos/FreeRTOS.h>
#include <esp_system.h>
#include <esp_event.h>
#include <esp_event_loop.h>
#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <time.h>
#include <stdbool.h>
#include <math.h>
#include <driver/ledc.h>
#include <driver/spi_master.h>
#include <driver/i2c.h>
#include <string.h>

#include <math.h>

#include "ear.h"
#include "tlc.h"
#include "tlc_power_gov.h"
#include "util.h"
#include "i2c_bus.h"
#include "lis3mdl_service.h"
#include "bme680_sensor.h"
#include "bh1750_sensor.h"
#include "platform.h"
#include "wifi.h"
#include "random.h"
#include "flash.h"
#include "ip.h"
#include "httpd.h"
#include "pattern.h"
#include "pattern_color_wheel.h"

#include "fast_hsv2rgb.h"


//#define DIM 50
#define SELF_TEST
//#define UV
//#define UV_STROBE
//#define COLOR_STROBE
#define COLOR_WHEEL
//#define WHITE
//#define RED

struct tlc_chain tlc;

esp_err_t event_handler(void *ctx, system_event_t *event) {
    return ESP_OK;
}

void gpio_output(uint8_t gpio) {
  gpio_config_t conf = {
    .intr_type = GPIO_PIN_INTR_DISABLE,
    .mode = GPIO_MODE_OUTPUT,
    .pin_bit_mask = (1ull << gpio),
    .pull_down_en = 0,
    .pull_up_en = 0
  };
  gpio_config(&conf);
}

#define DOC_SET(doc, val)\
  do {\
    doc##_r = (val);\
    doc##_g = (val);\
    doc##_b = (val);\
  } while (0);


static void illuminance_cb(struct sensor_manager* mgr, struct sensor* sensor, sensor_param_t param, sensor_result_t* res, size_t len, void* priv) {
//  ESP_LOGI("ILLUMINANCE_CB", "Got illuminance CB: %f", *res);
}

static char* unknown_mode = "unknown mode";

static esp_err_t mode_template_cb(void* ctx, void* priv, struct templ_slice* slice) {
  return httpd_template_write(ctx, unknown_mode, strlen(unknown_mode));
}

struct pattern_def* patterns[] = {
  &pattern_cw_def,
  NULL,
};

static const char* option_start = "<option>";
static const char* option_end = "</option>\n\r";

static esp_err_t patterns_template_cb(void* ctx, void* priv, struct templ_slice* slice) {
  esp_err_t err = 0;
  int i = 0;
  while(patterns[i] != NULL) {
    struct pattern_def* pat = patterns[i];
    if((err = httpd_template_write(ctx, option_start, strlen(option_start)))) {
      goto fail;
    }
    if((err = httpd_template_write(ctx, pat->name, strlen(pat->name)))) {
      goto fail;
    }
    if((err = httpd_template_write(ctx, option_end, strlen(option_end)))) {
      goto fail;
    }
    i++;
  }
fail:
  return err;
}


void app_main(void) {
  ESP_LOGI("WIFI", "Initializing IP stack");
  ESP_ERROR_CHECK(ip_stack_init());
  ESP_LOGI("WIFI", "Initializing wifi stack");
  ESP_ERROR_CHECK(wifi_init());
  ESP_LOGI("NVS", "Initializing NVS");
  ESP_ERROR_CHECK(flash_nvs_init());
  struct fatfs* fs;
  ESP_LOGI("FATFS", "Initializing file system");
  ESP_ERROR_CHECK(flash_fatfs_alloc(&fs, "storage", "/flash"));

  ESP_LOGI("HTTPD", "Starting http server");
  struct httpd* httpd;
  ESP_ERROR_CHECK(httpd_alloc(&httpd, "/flash/srv/http", 256));
  ESP_ERROR_CHECK(httpd_add_template(httpd, "ears.patterns", patterns_template_cb, NULL));
  ESP_ERROR_CHECK(httpd_add_template(httpd, "mode", mode_template_cb, NULL));
  ESP_ERROR_CHECK(httpd_add_redirect(httpd, "/", "/index.html"));
  ESP_ERROR_CHECK(httpd_add_static_path(httpd, "/flash/srv/http"));

  ESP_LOGI("WIFI", "Starting AP");
  ESP_ERROR_CHECK(wifi_ap_start(WIFI_SSID, WIFI_PSK));

  ESP_ERROR_CHECK(platform_init());

  // Sensors
  ESP_ERROR_CHECK(platform_subscribe_sensor(SENSOR_PARAM_ILLUMINANCE, illuminance_cb, NULL));

  // EARS

  printf("Size of TLC GS is %zu bytes\n", sizeof(struct tlc_gs));
  printf("Size of TLC CTL is %zu bytes\n", sizeof(struct tlc_ctl));
  printf("Size of TLC CTL DC is %zu bytes\n", sizeof((((struct tlc_ctl*)(void*)0))->dc));


#ifdef DIM
  uint8_t dimval = (uint8_t)(CLAMP((DIM - 26), 0, 74) * 127 / 74);
  ESP_LOGI("DIMMING", "Dimming to %d %%, val = %u", DIM, dimval);

  for(int i = 0; i < NUM_TLCS; i++) {
    DOC_SET(ears.ctl_data[i].dc.doc_0, dimval)
    DOC_SET(ears.ctl_data[i].dc.doc_1, dimval)
    DOC_SET(ears.ctl_data[i].dc.doc_2, dimval)
    DOC_SET(ears.ctl_data[i].dc.doc_3, dimval)
    DOC_SET(ears.ctl_data[i].dc.doc_4, dimval)
    DOC_SET(ears.ctl_data[i].dc.doc_5, dimval)
    DOC_SET(ears.ctl_data[i].dc.doc_6, dimval)
    DOC_SET(ears.ctl_data[i].dc.doc_7, dimval)
    DOC_SET(ears.ctl_data[i].dc.doc_8, dimval)
    DOC_SET(ears.ctl_data[i].dc.doc_9, dimval)
    DOC_SET(ears.ctl_data[i].dc.doc_10, dimval)
    DOC_SET(ears.ctl_data[i].dc.doc_11, dimval)
    DOC_SET(ears.ctl_data[i].dc.doc_12, dimval)
    DOC_SET(ears.ctl_data[i].dc.doc_13, dimval)
    DOC_SET(ears.ctl_data[i].dc.doc_14, dimval)
    DOC_SET(ears.ctl_data[i].dc.doc_15, dimval)
  }
#endif

#define MOD(i, j, k) ((i % j == k) ? 1 : 0)

#ifdef SELF_TEST
  // Test pattern ~3.3 s per ear
  for(int k = 0; k < 3 * 3; k++) {
    for(int i = 0; i < ears.chain_len; i++) {
      for(int j = 0; j < NUM_LEDS_PER_EAR; j++) {
        platform_set_led_rgb888(i, j, 255 * MOD(k, 3, 0), 255 * MOD(k, 3, 1), 255 * MOD(k, 3, 2));
      }
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
#endif
  // Enable all UV LEDs
  for(int i = 0; i < ears.chain_len; i++) {
#ifdef UV
    for(int j = 0; j < NUM_UV_PER_EAR; j++) {
      printf("Enabling UV led %d %d\n", i, j);
      platform_set_led_uv8(i, j, 0xFF);
    }
#endif
    for(int j = 0; j < NUM_LEDS_PER_EAR; j++) {
      platform_set_led_rgb888(i, j, 0, 0, 0);
#ifdef WHITE
      platform_set_led_rgb888(i, j, 255, 255, 255);
#endif
#ifdef RED
      platform_set_led_rgb888(i, j, 255, 0, 0);
#endif
    }
  }

  int offset = 0;
  int steps_per_led = HSV_HUE_STEPS / (NUM_LEDS_PER_EAR * ears.chain_len);

  int count = 0;
  while(1) {
    if(!(count % 20)) {
      sensor_result_t magneto[3];
      sensor_result_t lux;
      sensor_result_t temperature;
      sensor_result_t humidity;
      sensor_result_t pressure;
      sensor_result_t iaq;
      ESP_ERROR_CHECK(platform_get_sensor_result(SENSOR_PARAM_ILLUMINANCE, &lux, sizeof(lux)));
      ESP_LOGI("BH1750", "%.4f Lux", lux);
      ESP_ERROR_CHECK(platform_get_sensor_result(SENSOR_PARAM_BFIELD, magneto, sizeof(magneto)));
      ESP_LOGI("LIS3MDL", "X: %f", magneto[0]);
      ESP_LOGI("LIS3MDL", "Y: %f", magneto[1]);
      ESP_LOGI("LIS3MDL", "Z: %f", magneto[2]);
      float x = magneto[0];
      float y = magneto[1];
      float angle = atanf(x / y) / M_PI * 180.0;
      ESP_LOGI("LIS3MDL", "Angle: %.3f\n", angle);

      ESP_ERROR_CHECK(platform_get_sensor_result(SENSOR_PARAM_TEMPERATURE, &temperature, sizeof(temperature)));
      ESP_ERROR_CHECK(platform_get_sensor_result(SENSOR_PARAM_HUMIDITY, &humidity, sizeof(humidity)));
      ESP_ERROR_CHECK(platform_get_sensor_result(SENSOR_PARAM_PRESSURE, &pressure, sizeof(pressure)));
      ESP_LOGI("BME680", "Temperature: %.2f °C", temperature);
      ESP_LOGI("BME680", "R. Humidity: %.2f %%", humidity);
      ESP_LOGI("BME680", "Pressure: %.2f hPa", pressure / 100.0);
      if(platform_has_sensor(&sensors, SENSOR_PARAM_IAQ)) {
        platform_get_sensor_result(SENSOR_PARAM_IAQ, &iaq, sizeof(iaq));
        ESP_LOGI("BME680", "IAQ: %.2f", iaq);
      }
      for(int i = 0; i < ears.chain_len; i++) {
        ESP_LOGI("power_governor", "Average power consumption: %u mW", ears.pwr_gov[i].power_avg_mw);
        ESP_LOGI("power_governor", "Current power consumption: %u mW", tlc_power_gov_current_power_mw(&ears.pwr_gov[i]));
      }
    }
#ifdef COLOR_WHEEL
    for(int i = 0; i < ears.chain_len; i++) {
      for(int j = 0; j < NUM_LEDS_PER_EAR; j++) {
        uint8_t r, g, b;
        fast_hsv2rgb_32bit((offset + (i * NUM_LEDS_PER_EAR * steps_per_led) + j * steps_per_led) % HSV_HUE_STEPS, HSV_SAT_MAX, HSV_VAL_MAX, &r, &g, &b);
        platform_set_led_rgb888(i, j, r, g, b);
      }
    }
    offset += 10;
    if(offset >= HSV_HUE_MAX) {
      offset = 0;
    }
#endif
#ifdef UV_STROBE
    for(int i = 0; i < ears.chain_len; i++) {
      for(int j = 0; j < NUM_UV_PER_EAR; j++) {
        if(count % 20 < 2) {
          platform_set_led_uv8(i, j, 0xFF);
        } else {
          platform_set_led_uv8(i, j, 0x00);
        }
      }
    }
#endif
#ifdef COLOR_STROBE
    for(int i = 0; i < ears.chain_len; i++) {
      for(int j = 0; j < NUM_LEDS_PER_EAR; j++) {
        if(count % 20 == 0) {
          platform_set_led_rgb888(i, j, 255, 0, 0);
        } else if(count % 20 == 10) {
          platform_set_led_rgb888(i, j, 0, 0, 255);
        }
      }
    }
#endif
    vTaskDelay(25 / portTICK_PERIOD_MS);
    count += 1;
    count %= 40;
  }
}
