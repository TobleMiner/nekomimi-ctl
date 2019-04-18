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
#include <string.h>

#include <math.h>

#include "tlc.h"

#include "fast_hsv2rgb.h"

#define GPIO_PWM_CLK_OUT 32
#define GPIO_DATA_LATCH  33
#define GPIO_BLANK       25

#define NUM_EARS 2
#define NUM_TLCS (NUM_EARS * 2)

#define NUM_LEDS_PER_EAR 12

#define DIM 10

#define DIM_IS_LOW_RANGE ((DIM) < 66)

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

#define COL_TO_PWM(c) (((uint16_t)(c)) << 0) // Not particularly exact but will do for now

void ear_set_led_rgb(int ear, int led, uint8_t red, uint8_t green, uint8_t blue) {
  struct tlc_pwm* pwm0 = &tlc.gs_data[ear * 2];
  struct tlc_pwm* pwm1 = &tlc.gs_data[ear * 2 + 1];
  switch(led) {
    case 0:
      pwm0->pwm_7_r = COL_TO_PWM(red);
      pwm0->pwm_7_g = COL_TO_PWM(green);
      pwm0->pwm_7_b = COL_TO_PWM(blue);
      break;
    case 1:
      pwm0->pwm_6_r = COL_TO_PWM(red);
      pwm0->pwm_6_g = COL_TO_PWM(green);
      pwm0->pwm_6_b = COL_TO_PWM(blue);
      break;
    case 2:
      pwm0->pwm_5_r = COL_TO_PWM(red);
      pwm0->pwm_5_g = COL_TO_PWM(green);
      pwm0->pwm_5_b = COL_TO_PWM(blue);
      break;
    case 3:
      pwm0->pwm_3_r = COL_TO_PWM(red);
      pwm0->pwm_3_g = COL_TO_PWM(green);
      pwm0->pwm_3_b = COL_TO_PWM(blue);
      break;
    case 4:
      pwm0->pwm_1_r = COL_TO_PWM(red);
      pwm0->pwm_1_g = COL_TO_PWM(green);
      pwm0->pwm_1_b = COL_TO_PWM(blue);
      break;
    case 5:
      pwm0->pwm_0_r = COL_TO_PWM(red);
      pwm0->pwm_0_g = COL_TO_PWM(green);
      pwm0->pwm_0_b = COL_TO_PWM(blue);
      break;
    case 6:
      pwm1->pwm_0_r = COL_TO_PWM(red);
      pwm1->pwm_0_g = COL_TO_PWM(green);
      pwm1->pwm_0_b = COL_TO_PWM(blue);
      break;
    case 7:
      pwm1->pwm_2_r = COL_TO_PWM(red);
      pwm1->pwm_2_g = COL_TO_PWM(green);
      pwm1->pwm_2_b = COL_TO_PWM(blue);
      break;
    case 8:
      pwm1->pwm_3_r = COL_TO_PWM(red);
      pwm1->pwm_3_g = COL_TO_PWM(green);
      pwm1->pwm_3_b = COL_TO_PWM(blue);
      break;
    case 9:
      pwm1->pwm_5_r = COL_TO_PWM(red);
      pwm1->pwm_5_g = COL_TO_PWM(green);
      pwm1->pwm_5_b = COL_TO_PWM(blue);
      break;
    case 10:
      pwm1->pwm_6_r = COL_TO_PWM(red);
      pwm1->pwm_6_g = COL_TO_PWM(green);
      pwm1->pwm_6_b = COL_TO_PWM(blue);
      break;
    case 11:
      pwm1->pwm_7_r = COL_TO_PWM(red);
      pwm1->pwm_7_g = COL_TO_PWM(green);
      pwm1->pwm_7_b = COL_TO_PWM(blue);
      break;
  };
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

#define DOC_RANGE_LO(doc)\
  do {\
    (doc).doc_range_r = 0;\
    (doc).doc_range_g = 0;\
    (doc).doc_range_b = 0;\
  } while (0);
  

void app_main(void) {
  esp_err_t err;

  printf("Size of TLC PWM is %zu bytes\n", sizeof(struct tlc_pwm));
  printf("Size of TLC DC is %zu bytes\n", sizeof(struct tlc_dc));
  printf("Size of TLC DC DOC is %zu bytes\n", sizeof((((struct tlc_dc*)(void*)0))->doc));

  ESP_LOGI("TLC", "Initialitzing ...");
  ESP_ERROR_CHECK(tlc_init(NUM_TLCS, GPIO_PWM_CLK_OUT, GPIO_DATA_LATCH, GPIO_BLANK, HSPI_HOST, VSPI_HOST));
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
  uint8_t dimval = (uint8_t)CLAMP((DIM_IS_LOW_RANGE ? (((float)DIM) / 0.667) : ((((float)DIM) - 33.3) / 0.527559055)) / 100.0 * 127.0, 0, 127);
  ESP_LOGI("DIMMING", "Dimming to %d %%, val = %u", DIM, dimval);

  for(int i = 0; i < NUM_TLCS; i++) {
#if DIM_IS_LOW_RANGE
    ESP_LOGI("DIMMING", "Using low dimming range");
    DOC_RANGE_LO(tlc.dc_data[i]);
#endif
    DOC_SET(tlc.dc_data[i].doc.doc_0, dimval)
    DOC_SET(tlc.dc_data[i].doc.doc_1, dimval)
    DOC_SET(tlc.dc_data[i].doc.doc_2, dimval)
    DOC_SET(tlc.dc_data[i].doc.doc_3, dimval)
    DOC_SET(tlc.dc_data[i].doc.doc_4, dimval)
    DOC_SET(tlc.dc_data[i].doc.doc_5, dimval)
    DOC_SET(tlc.dc_data[i].doc.doc_6, dimval)
    DOC_SET(tlc.dc_data[i].doc.doc_7, dimval)
  }
#endif

  xTaskCreate(tlc_update_task, "tlc_task", 4096, NULL, 12, NULL);

  // Test pattern ~3.3 s per ear
  for(int i = 0; i < NUM_EARS; i++) {
    for(int j = 0; j < NUM_LEDS_PER_EAR; j++) {
      ear_set_led_rgb(i, j, 255, 0, 0);
      vTaskDelay(100 / portTICK_PERIOD_MS);
      ear_set_led_rgb(i, j, 0, 255, 0);
      vTaskDelay(100 / portTICK_PERIOD_MS);
      ear_set_led_rgb(i, j, 0, 0, 255);
      vTaskDelay(100 / portTICK_PERIOD_MS);
      ear_set_led_rgb(i, j, 0, 0, 0);
    }
  }

  int offset = 0;
  int steps_per_led = HSV_HUE_STEPS / (NUM_LEDS_PER_EAR * NUM_EARS);

  while(1) {
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
    
    vTaskDelay(25 / portTICK_PERIOD_MS);
  }
}
