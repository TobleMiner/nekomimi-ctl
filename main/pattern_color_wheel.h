#pragma once

#include <stdint.h>

#include "pattern.h"
#include "util.h"

struct pattern_cw {
  struct pattern pat;
  int pos;
  int steps_per_led;
};

extern struct pattern_def pattern_cw_def;
