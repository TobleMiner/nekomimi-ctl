#include <limits.h>

#include "timer.h"

esp_err_t system_timer_init(struct system_timer* timer, uint64_t interval_ns, timer_group_t timer_group, timer_idx_t timer_id) {
  esp_err_t err;
  uint64_t divider = TIMER_INTERVAL_NS_TO_DIVIDER(interval_ns);
  timer_config_t conf = {
    .counter_dir = TIMER_COUNT_UP,
    .divider     = divider,
    .counter_en  = TIMER_PAUSE,
    .auto_reload = TIMER_AUTORELOAD_EN,
  };

  if(interval_ns != TIMER_DIVIDER_TO_INTERVAL_NS(conf.divider)) {
    return ESP_ERR_INVALID_ARG;
  }
  
  timer->tick_interval_ns = interval_ns;
  timer->group = timer_group;
  timer->id = timer_id;
  err = timer_init(timer_group, timer_id, &conf);
  if(err) {
    return err;
  }
  err = timer_set_counter_value(timer_group, timer_id, 0);
  if(err) {
    return err;
  }
  err = timer_set_alarm_value(timer_group, timer_id, ULLONG_MAX / divider);
  if(err) {
    return err;
  }  
  return timer_start(timer_group, timer_id);
}

uint64_t system_timer_get_time_ns(struct system_timer* timer) {
  uint64_t val;
  timer_get_counter_value(timer->group, timer->id, &val);
  return val * timer->tick_interval_ns;
}
