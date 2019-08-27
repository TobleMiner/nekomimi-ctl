#include <stdlib.h>

#include "pattern_color_wheel.h"
#include "config.h"
#include "platform.h"

#include "fast_hsv2rgb.h"

#define PATTERN_TO_PATTERN_CW(pat) (container_of((pat), struct pattern_cw, pat))

static esp_err_t pat_alloc(struct pattern** retval) {
  struct pattern_cw* pat_cw = calloc(1, sizeof(struct pattern_cw));
  if(!pat_cw) {
    return ESP_ERR_NO_MEM;
  }
  pat_cw->steps_per_led = HSV_HUE_STEPS / (NUM_LEDS_PER_EAR * ears.chain_len);

  *retval = &pat_cw->pat;
  return ESP_OK;
}

static void pat_free(struct pattern* pat) {
  free(PATTERN_TO_PATTERN_CW(pat));
}

static esp_err_t pat_step(struct pattern* pat, uint32_t dt_ms) {
  struct pattern_cw* pat_cw = PATTERN_TO_PATTERN_CW(pat);
  int steps_per_led = pat_cw->steps_per_led;
  for(int i = 0; i < ears.chain_len; i++) {
    for(int j = 0; j < NUM_LEDS_PER_EAR; j++) {
      uint8_t r, g, b;
      fast_hsv2rgb_32bit((pat_cw->pos + (i * NUM_LEDS_PER_EAR * steps_per_led) + j * steps_per_led) % HSV_HUE_STEPS, HSV_SAT_MAX, HSV_VAL_MAX, &r, &g, &b);
      platform_set_led_rgb888(i, j, r, g, b);
    }
  }
  pat_cw->pos += 10;
  if(pat_cw->pos >= HSV_HUE_MAX) {
    pat_cw->pos = 0;
  }
  return ESP_OK;
}

struct pattern_def pattern_cw_def = {
  .name = "Color Wheel",
  .ops = {
    .alloc = pat_alloc,
    .free = pat_free,
    .step = pat_step
  }
};
