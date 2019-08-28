#pragma once

#include <stdint.h>

#include "pattern.h"
#include "util.h"

struct pattern_police {
  struct pattern pat;
  int count;
};

extern struct pattern_def pattern_police_def;
