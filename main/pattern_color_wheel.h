#pragma once

#include <stdint.h>

#include "pattern.h"
#include "util.h"

struct pattern_cw {
  struct pattern pat;
  uint32_t pos;
};

extern struct pattern_def pattern_cw_def;
