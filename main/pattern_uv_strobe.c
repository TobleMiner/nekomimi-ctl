#include <stdlib.h>

#include "pattern_uv_strobe.h"
#include "config.h"
#include "platform.h"

#define PATTERN_TO_PATTERN_STROBE(pat) (container_of((pat), struct pattern_uv_strobe, pat))

static esp_err_t pat_alloc(struct pattern** retval) {
  struct pattern_uv_strobe* pat_strobe = calloc(1, sizeof(struct pattern_uv_strobe));
  if(!pat_strobe) {
    return ESP_ERR_NO_MEM;
  }
  *retval = &pat_strobe->pat;
  return ESP_OK;
}

static void pat_free(struct pattern* pat) {
  free(PATTERN_TO_PATTERN_STROBE(pat));
}

#define STOBE_DURATION_MS 50
#define STOBE_INTERVAL_MS 1000

static esp_err_t pat_step(struct pattern* pat, uint32_t dt_ms) {
  struct pattern_uv_strobe* pat_strobe = PATTERN_TO_PATTERN_STROBE(pat);
  for(int i = 0; i < ears.chain_len; i++) {
    for(int j = 0; j < NUM_UV_PER_EAR; j++) {
      if(pat_strobe->count == 0) {
        platform_set_led_uv8(i, j, 0xFF);
      } else if(pat_strobe->count > STOBE_DURATION_MS) {
        platform_set_led_uv8(i, j, 0);
      }
    }
    for(int j = 0; j < NUM_LEDS_PER_EAR; j++) {
      platform_set_led_rgb888(i, j, 0, 0, 0);
    }
  }
  if(pat_strobe->count >= STOBE_INTERVAL_MS) {
    pat_strobe->count = 0;
  } else {
    pat_strobe->count += dt_ms;
  }
  return ESP_OK;
}

struct pattern_def pattern_uv_strobe_def = {
  .name = "UV Strobe",
  .ops = {
    .alloc = pat_alloc,
    .free = pat_free,
    .step = pat_step
  }
};
