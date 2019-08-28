#pragma once

#include <stdint.h>

#include "pattern.h"
#include "util.h"

struct pattern_led_test {
  struct pattern pat;
  uint32_t cnt;
};

extern struct pattern_def pattern_led_test_def;
