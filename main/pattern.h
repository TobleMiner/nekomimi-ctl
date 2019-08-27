#pragma once

#include <stdint.h>

#include <esp_err.h>

struct pattern;

typedef esp_err_t (*pattern_alloc_f)(struct pattern** retval);
typedef void (*pattern_free_f)(struct pattern* retval);
typedef esp_err_t (*pattern_step_f)(struct pattern* retval, uint32_t dt_ms);

struct pattern_ops {
  pattern_alloc_f alloc;
  pattern_free_f free;
  pattern_step_f step;
};

struct pattern_def {
  char* name;
  struct pattern_ops ops;
};

struct pattern {
  struct pattern_def* def;
};

esp_err_t pattern_alloc(struct pattern** retval, struct pattern_def* def);
void pattern_free(struct pattern* pat);
esp_err_t pattern_step(struct pattern* pat, uint32_t dt_ms);
