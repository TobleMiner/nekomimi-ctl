#include <string.h>
#include <stdint.h>

#include <esp_log.h>

#include "tlc_accessors.h"
#include "tlc_power_gov.h"

#define TAG_POWER_GOV "power_gov"

void tlc_power_gov_init(struct tlc_power_gov* gov, uint16_t voltage_mv, uint16_t power_limit, struct tlc_gs* gs, struct tlc_ctl* ctl) {
  memset(gov, 0, sizeof(*gov));
  gov->voltage_mv = voltage_mv;
  gov->power_limit = power_limit;
  gov->gs = gs;
  gov->ctl = ctl;
}

void tlc_power_gov_setup_led(struct tlc_power_gov* gov, uint8_t channel, uint8_t color, struct tlc_led_spec led_spec) {
  gov->chan_specs[channel].colors[color] = led_spec;
}

uint16_t tlc_power_gov_current_power_mw(struct tlc_power_gov* gov) {
  uint64_t current_total_ua = 0;
  uint32_t power_mw;
  uint64_t power_avg_mw;

  // Iterate over colors
  for(int color = 0; color < 3; color++) {
    uint64_t current_color = 0;

    // Iterate over RGB channels
    for(int channel = 0; channel < 16; channel++) {
      // Calculate factor based on dot control values
      uint64_t current = (262ULL + 738ULL * (uint64_t)tlc_ctl_get_doc(gov->ctl, channel, color) / 127ULL);
      // Calculate factor based on PWM control values ("average" current)
      current *= (uint64_t)gov->gs->channels[channel].channels[color];
      current_color += current;
    }
    // Calculate mc only once per color
    current_color *= (uint64_t)tlc_ctl_get_mcr_ua(gov->ctl, color);
    // Calculate bc only once per color
    current_color *= (10ULL + 90ULL * (uint64_t)tlc_ctl_get_bc(gov->ctl, color) / 127ULL);
    current_total_ua += current_color; 
  }
  // Remove dot control and brightness control factor
  current_total_ua /= 100000ULL;
  // Remove PWM factor
  current_total_ua /= 65536ULL;
  
  return (uint16_t)((uint64_t)current_total_ua * (uint64_t)gov->voltage_mv / 1000000ULL);
}

uint8_t tlc_power_gov_govern(struct tlc_power_gov* gov, uint32_t delta_t_us) {
  uint32_t power_mw;
  uint64_t power_avg_mw;

  // Calculate average over 1s
  power_mw = tlc_power_gov_current_power_mw(gov);
  power_avg_mw = (uint64_t)gov->power_avg_mw * (1000000ULL - (uint64_t)delta_t_us);
  power_avg_mw += (uint64_t)delta_t_us * (uint64_t)power_mw;
  power_avg_mw /= 1000000ULL;
  gov->power_avg_mw = power_avg_mw;

  return gov->power_limit < gov->power_avg_mw ? (uint8_t)((uint32_t)gov->power_limit * 100UL / (uint32_t)gov->power_avg_mw) : 100;
}