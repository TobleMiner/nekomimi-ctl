#include "pattern.h"

esp_err_t pattern_alloc(struct pattern** retval, struct pattern_def* def) {
  esp_err_t err;
  err = def->ops.alloc(retval);
  if(err) {
    return err;
  }
  (*retval)->def = def;
  return ESP_OK;
}

void pattern_free(struct pattern* pat) {
  pat->def->ops.free(pat);
}

esp_err_t pattern_step(struct pattern* pat, uint32_t dt_ms) {
  return pat->def->ops.step(pat, dt_ms);
}
