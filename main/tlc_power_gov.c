#include <string.h>
#include <stdint.h>

#include "tlc_power_gov.h"

void tlc_power_gov_init(struct tlc_power_gov* gov, uint16_t voltage_mv, uint16_t power_limit) {
  memset(gov, 0, sizeof(*gov));
  gov->voltage_mv = voltage_mv;
  gov->power_limit = power_limit;
}

void tlc_power_gov_setup_led(struct tlc_power_gov* gov, uint8_t channel, uint8_t color, struct tlc_led_spec led_spec) {
  gov->chan_specs[channel].colors[color] = led_spec;
}

static uint16_t tlc_power_gov_calc_led(struct tlc_power_gov* gov, uint8_t channel, uint8_t color) {
  struct tlc_led_spec* spec = &gov->chan_specs[channel].colors[color];
  uint64_t i_out_ua = tlc_ctl_get_mcr_ua(gov->ctl, color);
  i_out_ua *= (262ULL + 738ULL * (uint64_t)tlc_ctl_get_doc(gov->ctl, channel, color) / 127ULL);
  i_out_ua *= (10ULL + 90ULL * (uint64_t)tlc_ctl_get_bc(gov->ctl, color) / 127ULL);
  i_out_ua /= 100000ULL;
  return (uint16_t)i_out_ua;
}

void tlc_power_gov_govern(struct tlc_power_gov* gov, struct tlc_gs* gs, struct tlc_ctl* ctl) {
  
}