#pragma once

#include <stdint.h>

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
  struct tlc_gs*       gs_data;
  struct tlc_ctl*      ctl_data;

  uint32_t             power_avg_mw;
};
