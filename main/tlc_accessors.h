#pragma once

#include <stdint.h>

#include "tlc.h"

uint8_t tlc_ctl_get_bc(struct tlc_ctl* ctl, uint8_t color);

uint16_t tlc_ctl_get_mcr_ua(struct tlc_ctl* ctl, uint8_t color);

uint8_t tlc_ctl_get_doc(struct tlc_ctl* ctl, uint8_t channel, uint8_t color);
