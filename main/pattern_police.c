#include <stdlib.h>

#include "pattern_police.h"
#include "config.h"
#include "platform.h"

#define PATTERN_TO_PATTERN_POLICE(pat) (container_of((pat), struct pattern_police, pat))

static esp_err_t pat_alloc(struct pattern** retval) {
  struct pattern_police* pat_police = calloc(1, sizeof(struct pattern_police));
  if(!pat_police) {
    return ESP_ERR_NO_MEM;
  }
  *retval = &pat_police->pat;
  return ESP_OK;
}

static void pat_free(struct pattern* pat) {
  free(PATTERN_TO_PATTERN_POLICE(pat));
}

static esp_err_t pat_step(struct pattern* pat, uint32_t dt_ms) {
  struct pattern_police* pat_police = PATTERN_TO_PATTERN_POLICE(pat);
  for(int i = 0; i < ears.chain_len; i++) {
    for(int j = 0; j < NUM_LEDS_PER_EAR; j++) {
      if(pat_police->count % 20 == 0) {
        platform_set_led_rgb888(i, j, 255, 0, 0);
        pat_police->count = 0;
      } else if(pat_police->count % 20 == 10) {
        platform_set_led_rgb888(i, j, 0, 0, 255);
      }
    }
  }
  pat_police->count++;
  return ESP_OK;
}

struct pattern_def pattern_police_def = {
  .name = "Police",
  .ops = {
    .alloc = pat_alloc,
    .free = pat_free,
    .step = pat_step
  }
};
