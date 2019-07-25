#pragma once

#include <stdint.h>

#include "tlc.h"

struct tlc_led_spec {
  uint16_t u_forward;
  uint16_t i_cont;
  uint8_t  efficiency;
};

union tlc_chan_spec {
  struct {
    struct tlc_led_spec red;
    struct tlc_led_spec green;
    struct tlc_led_spec blue;
  };
  struct tlc_led_spec colors[3];
};

struct tlc_power_gov {
  uint16_t             voltage_mv;
  uint32_t             power_limit;
  union tlc_chan_spec  chan_specs[16];
  struct tlc_gs*       gs;
  struct tlc_ctl*      ctl;

  uint32_t             power_avg_mw;
};

void tlc_power_gov_init(struct tlc_power_gov* gov, uint16_t voltage_mv, uint16_t power_limit, struct tlc_gs* gs, struct tlc_ctl* ctl);
void tlc_power_gov_setup_led(struct tlc_power_gov* gov, uint8_t channel, uint8_t color, struct tlc_led_spec led_spec);
void tlc_power_gov_govern(struct tlc_power_gov* gov, uint32_t delta_t_us);