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

#include "tlc.h"
#include "util.h"
#include "i2c_bus.h"
#include "bh1750_service.h"

#include "fast_hsv2rgb.h"

#define GPIO_PWM_CLK_OUT 32
#define GPIO_DATA_LATCH  33
#define GPIO_BLANK       25

#define NUM_EARS 2
#define NUM_TLCS NUM_EARS

#define NUM_LEDS_PER_EAR 13
#define NUM_UV_PER_EAR 8

#define DIM 50
#define SELF_TEST
//#define UV
#define COLOR_WHEEL
//#define WHITE
//#define RED

#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))

#define CLAMP(x, h, l) (max(min((x), (l)), (h)))


esp_err_t event_handler(void *ctx, system_event_t *event) {
    return ESP_OK;
}

struct color {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

#ifdef DIM
#define COL_TO_PWM(c) (((uint16_t)(c)) * (DIM > 26 ? 257U : 257U * DIM / 26)) // Not particularly exact but will do for now
#else
#define COL_TO_PWM(c) (((uint16_t)(c)) * 257U)
#endif

static uint8_t rgb_led_map[13] = {
  8,
  12,
  9,
  13,
  10,
  15,
  11,
  3,
  6,
  1,
  5,
  0,
  4
};

void ear_set_led_rgb(int ear, int led, uint8_t red, uint8_t green, uint8_t blue) {
  struct tlc_gs* chip = &tlc.gs_data[ear];
  struct tlc_gs_chan* chan = &chip->channels[rgb_led_map[led]];
  chan->r = COL_TO_PWM(red);
  chan->g = COL_TO_PWM(green);
  chan->b = COL_TO_PWM(blue);
//	printf("Setting LED %d:%d -> %u to (%u,%u,%u)\n", ear, led, rgb_led_map[led], red, green, blue);
}

static uint8_t uv_led_map[8] = {
  14,
  14,
  14,
  7,
  7,
  2,
  2,
  2,
};

static uint8_t uv_chan_map[8] = {
  2,
  0,
  1,
  1,
  0,
  1,
  0,
  2
};

void ear_set_led_uv(int ear, int led, uint8_t level) {
  struct tlc_gs* chip = &tlc.gs_data[ear];
  struct tlc_gs_chan* chan = &chip->channels[uv_led_map[led]];
  chan->channels[uv_chan_map[led]] = COL_TO_PWM(level);
}

void ear_set_led(int ear, int led, struct color col) {
  ear_set_led_rgb(ear, led, col.red, col.green, col.blue);
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
  

void app_main(void) {
  struct bh1750_service bh;
  struct i2c_bus i2c1;

  // I2C
  ESP_ERROR_CHECK(i2c_bus_init(&i2c1, I2C_NUM_1, 26, 25, 10000));
  i2c_detect(&i2c1);
  ESP_ERROR_CHECK(bh1750_service_init(&bh, &i2c1, BH1750_ADDR_L));

  // EARS

  printf("Size of TLC GS is %zu bytes\n", sizeof(struct tlc_gs));
  printf("Size of TLC CTL is %zu bytes\n", sizeof(struct tlc_ctl));
  printf("Size of TLC CTL DC is %zu bytes\n", sizeof((((struct tlc_ctl*)(void*)0))->dc));

  ESP_LOGI("TLC", "Initialitzing ...");
  ESP_ERROR_CHECK(tlc_init(NUM_TLCS, GPIO_PWM_CLK_OUT, GPIO_DATA_LATCH, HSPI_HOST));
  ESP_LOGI("TLC", "Initialized");

//  memset(tlc.gs_data, 0x00, sizeof(struct tlc_pwm) * tlc.chain_len);

/*
  tlc.gs_data[0].pwm_7_r = 0xFFF;
  tlc.gs_data[0].pwm_7_g = 0xFFF;
  tlc.gs_data[0].pwm_7_b = 0xFFF;

  tlc.gs_data[0].pwm_4_r = 0x0;
  tlc.gs_data[0].pwm_2_g = 0x0;
  tlc.gs_data[1].pwm_4_b = 0x0;
  tlc.gs_data[1].pwm_1_g = 0x0;
  tlc.gs_data[1].pwm_7_g = 0xFFF;
*/

#ifdef DIM
  uint8_t dimval = (uint8_t)(CLAMP((DIM - 26), 0, 74) * 127 / 74);
  ESP_LOGI("DIMMING", "Dimming to %d %%, val = %u", DIM, dimval);

  for(int i = 0; i < NUM_TLCS; i++) {
    DOC_SET(tlc.ctl_data[i].dc.doc_0, dimval)
    DOC_SET(tlc.ctl_data[i].dc.doc_1, dimval)
    DOC_SET(tlc.ctl_data[i].dc.doc_2, dimval)
    DOC_SET(tlc.ctl_data[i].dc.doc_3, dimval)
    DOC_SET(tlc.ctl_data[i].dc.doc_4, dimval)
    DOC_SET(tlc.ctl_data[i].dc.doc_5, dimval)
    DOC_SET(tlc.ctl_data[i].dc.doc_6, dimval)
    DOC_SET(tlc.ctl_data[i].dc.doc_7, dimval)
    DOC_SET(tlc.ctl_data[i].dc.doc_8, dimval)
    DOC_SET(tlc.ctl_data[i].dc.doc_9, dimval)
    DOC_SET(tlc.ctl_data[i].dc.doc_10, dimval)
    DOC_SET(tlc.ctl_data[i].dc.doc_11, dimval)
    DOC_SET(tlc.ctl_data[i].dc.doc_12, dimval)
    DOC_SET(tlc.ctl_data[i].dc.doc_13, dimval)
    DOC_SET(tlc.ctl_data[i].dc.doc_14, dimval)
    DOC_SET(tlc.ctl_data[i].dc.doc_15, dimval)
  }
#endif

  xTaskCreate(tlc_update_task, "tlc_task", 4096, NULL, 12, NULL);

#define MOD(i, j, k) ((i % j == k) ? 1 : 0)

#ifdef SELF_TEST
  // Test pattern ~3.3 s per ear
  for(int k = 0; k < 3 * 3; k++) {
    for(int i = 0; i < NUM_EARS; i++) {
      for(int j = 0; j < NUM_LEDS_PER_EAR; j++) {
        ear_set_led_rgb(i, j, 255 * MOD(k, 3, 0), 255 * MOD(k, 3, 1), 255 * MOD(k, 3, 2));
      }
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
#endif

  // Enable all UV LEDs
  for(int i = 0; i < NUM_EARS; i++) {
#ifdef UV
    for(int j = 0; j < NUM_UV_PER_EAR; j++) {
      printf("Enabling UV led %d %d\n", i, j);
      ear_set_led_uv(i, j, 0xFF);
    }
#endif
    for(int j = 0; j < NUM_LEDS_PER_EAR; j++) {
#ifdef WHITE
      ear_set_led_rgb(i, j, 255, 255, 255);      
#endif
#ifdef RED
      ear_set_led_rgb(i, j, 255, 0, 0);
#endif
    }
  }

  int offset = 0;
  int steps_per_led = HSV_HUE_STEPS / (NUM_LEDS_PER_EAR * NUM_EARS);

  int count = 0;
  while(1) {
    if(!(count % 20)) {
      float lux = bh1750_service_get_luminocity(&bh);
      ESP_LOGI("BH1750", "%.4f Lux", lux);
//      ESP_LOGI("BH1750", "Measurement took %u ms", bh1750_get_mt_ms(&bh));      
    }
#ifdef COLOR_WHEEL
    for(int i = 0; i < NUM_EARS; i++) {
      for(int j = 0; j < NUM_LEDS_PER_EAR; j++) {
        uint8_t r, g, b;
        fast_hsv2rgb_32bit((offset + (i * NUM_LEDS_PER_EAR * steps_per_led) + j * steps_per_led) % HSV_HUE_STEPS, HSV_SAT_MAX, HSV_VAL_MAX, &r, &g, &b);
        ear_set_led_rgb(i, j, r, g, b);
      }
    }
    offset += 10;
    if(offset >= HSV_HUE_MAX) {
      offset = 0;
    }
#endif
    vTaskDelay(25 / portTICK_PERIOD_MS);
    count += 1;
    count %= 40;
  }
}
