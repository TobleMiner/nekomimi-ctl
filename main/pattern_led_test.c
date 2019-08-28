#include <stdlib.h>

#include "pattern_led_test.h"
#include "config.h"
#include "platform.h"

#define PATTERN_TO_PATTERN_LED_TEST(pat) (container_of((pat), struct pattern_led_test, pat))

static esp_err_t pat_alloc(struct pattern** retval) {
  struct pattern_led_test* pat_test = calloc(1, sizeof(struct pattern_led_test));
  if(!pat_test) {
    return ESP_ERR_NO_MEM;
  }
  *retval = &pat_test->pat;
  return ESP_OK;
}

static void pat_free(struct pattern* pat) {
  free(PATTERN_TO_PATTERN_LED_TEST(pat));
}

#define LED_ON_TIME_MS 500

static esp_err_t pat_step(struct pattern* pat, uint32_t dt_ms) {
  struct pattern_led_test* pat_test = PATTERN_TO_PATTERN_LED_TEST(pat);
  pat_test->cnt += dt_ms;
  for(int i = 0; i < ears.chain_len; i++) {
    for(int j = 0; j < NUM_LEDS_PER_EAR; j++) {
      if(pat_test->cnt >= LED_ON_TIME_MS * 2) {
        platform_set_led_rgb888(i, j, 0, 0, 255);
      } else if(pat_test->cnt >= LED_ON_TIME_MS) {
        platform_set_led_rgb888(i, j, 0, 255, 0);
      } else {
        platform_set_led_rgb888(i, j, 255, 0, 0);
      }
    }
  }
  pat_test->cnt %= LED_ON_TIME_MS * 3;
  return ESP_OK;
}

struct pattern_def pattern_led_test_def = {
  .name = "LED test",
  .ops = {
    .alloc = pat_alloc,
    .free = pat_free,
    .step = pat_step
  }
};
