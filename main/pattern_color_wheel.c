#include "pattern_color_wheel.h"

#include <stdlib.h>

#define PATTERN_TO_PATTERN_CW(pat) (container_of((pat), struct pattern_cw, pat))

static esp_err_t pat_alloc(struct pattern** retval) {
  struct pattern_cw* pat_cw = calloc(1, sizeof(struct pattern_cw));
  if(!pat_cw) {
    return ESP_ERR_NO_MEM;
  }
  *retval = &pat_cw->pat;
  return ESP_OK;
}

static void pat_free(struct pattern* pat) {
  free(PATTERN_TO_PATTERN_CW(pat));
}

static esp_err_t pat_step(struct pattern* pat, uint32_t dt_ms) {
  struct pattern_cw* pat_cw = PATTERN_TO_PATTERN_CW(pat);
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
