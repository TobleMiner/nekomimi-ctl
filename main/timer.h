#pragma once

#include <stdint.h>

#include <esp_err.h>

#include <driver/timer.h>


#define TIMER_CLK ((uint64_t)(TIMER_BASE_CLK))
#define TIMER_NS_PER_SEC 1000000000ULL

#define TIMER_INTERVAL_NS_TO_DIVIDER(i) (((uint64_t)(i)) * TIMER_CLK / TIMER_NS_PER_SEC)
#define TIMER_DIVIDER_TO_INTERVAL_NS(d) (((uint64_t)(d)) * TIMER_NS_PER_SEC / TIMER_CLK)

struct system_timer {
  uint64_t      tick_interval_ns;
  timer_group_t group;
  timer_idx_t   id;
};

esp_err_t system_timer_init(struct system_timer* timer, uint64_t interval_ns, timer_group_t timer_group, timer_idx_t timer_id);
uint64_t system_timer_get_time_ns(struct system_timer* timer);
