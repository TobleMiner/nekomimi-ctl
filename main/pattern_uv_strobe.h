#pragma once

#include <stdint.h>

#include "pattern.h"
#include "util.h"

struct pattern_uv_strobe {
  struct pattern pat;
  int count;
};

extern struct pattern_def pattern_uv_strobe_def;
