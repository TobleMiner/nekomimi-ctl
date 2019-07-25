#include <stdint.h>

#include <esp_log.h>

#include "tlc_accessors.h"

uint8_t tlc_ctl_get_bc(struct tlc_ctl* ctl, uint8_t color) {
  if(color >= 3) {
    ESP_LOGE(TLC_TAG, "Color channel %u does not exist", color);
    return 0;
  }

  switch(color) {
    case 0:
      return ctl->bcr;
    case 1:
      return ctl->bcg;
    case 2:
      return ctl->bcb;
  }

  return 0;
}

uint16_t tlc_ctl_get_mcr_ua(struct tlc_ctl* ctl, uint8_t color) {
  uint8_t mcr = 0;

  if(color >= 3) {
    ESP_LOGE(TLC_TAG, "Color channel %u does not exist", color);
    return 0;
  }

  switch(color) {
    case 0:
      mcr = ctl->mcr;
      break;
    case 1:
      mcr = ctl->mcr;
      break;
    case 2:
      mcr = ctl->mcr;
      break;
  }

  switch(mcr) {
    case 0b000:
      return 3200;
    case 0b001:
      return 8000;
    case 0b010:
      return 11200;
    case 0b011:
      return 15900;
    case 0b100:
      return 19100;
    case 0b101:
      return 23900;
    case 0b110:
      return 27100;
    case 0b111:
      return 31900;
  }

  ESP_LOGE(TLC_TAG, "Invalid maximum current configuration on color %u: %u", color, mcr);
  return 0;
};

uint8_t tlc_ctl_get_doc(struct tlc_ctl* ctl, uint8_t channel, uint8_t color) {
  if(channel >= 16) {
    ESP_LOGE(TLC_TAG, "Channel %u does not exist", channel);
    return 0;
  }

  if(color >= 3) {
    ESP_LOGE(TLC_TAG, "Color channel %u does not exist", color);
    return 0;
  }

  switch(channel) {
    case 0:
      switch(color) {
        case 0:
          return ctl->dc.doc_0_r;
        case 1:
          return ctl->dc.doc_0_g;
        case 2:
          return ctl->dc.doc_0_b;
      }
    case 1:
      switch(color) {
        case 0:
          return ctl->dc.doc_1_r;
        case 1:
          return ctl->dc.doc_1_g;
        case 2:
          return ctl->dc.doc_1_b;
      }
    case 2:
      switch(color) {
        case 0:
          return ctl->dc.doc_2_r;
        case 1:
          return ctl->dc.doc_2_g;
        case 2:
          return ctl->dc.doc_2_b;
      }
    case 3:
      switch(color) {
        case 0:
          return ctl->dc.doc_3_r;
        case 1:
          return ctl->dc.doc_3_g;
        case 2:
          return ctl->dc.doc_3_b;
      }
    case 4:
      switch(color) {
        case 0:
          return ctl->dc.doc_4_r;
        case 1:
          return ctl->dc.doc_4_g;
        case 2:
          return ctl->dc.doc_4_b;
      }
    case 5:
      switch(color) {
        case 0:
          return ctl->dc.doc_5_r;
        case 1:
          return ctl->dc.doc_5_g;
        case 2:
          return ctl->dc.doc_5_b;
      }
    case 6:
      switch(color) {
        case 0:
          return ctl->dc.doc_6_r;
        case 1:
          return ctl->dc.doc_6_g;
        case 2:
          return ctl->dc.doc_6_b;
      }
    case 7:
      switch(color) {
        case 0:
          return ctl->dc.doc_7_r;
        case 1:
          return ctl->dc.doc_7_g;
        case 2:
          return ctl->dc.doc_7_b;
      }
    case 8:
      switch(color) {
        case 0:
          return ctl->dc.doc_8_r;
        case 1:
          return ctl->dc.doc_8_g;
        case 2:
          return ctl->dc.doc_8_b;
      }
    case 9:
      switch(color) {
        case 0:
          return ctl->dc.doc_9_r;
        case 1:
          return ctl->dc.doc_9_g;
        case 2:
          return ctl->dc.doc_9_b;
      }
    case 10:
      switch(color) {
        case 0:
          return ctl->dc.doc_10_r;
        case 1:
          return ctl->dc.doc_10_g;
        case 2:
          return ctl->dc.doc_10_b;
      }
    case 11:
      switch(color) {
        case 0:
          return ctl->dc.doc_11_r;
        case 1:
          return ctl->dc.doc_11_g;
        case 2:
          return ctl->dc.doc_11_b;
      }
    case 12:
      switch(color) {
        case 0:
          return ctl->dc.doc_12_r;
        case 1:
          return ctl->dc.doc_12_g;
        case 2:
          return ctl->dc.doc_12_b;
      }
    case 13:
      switch(color) {
        case 0:
          return ctl->dc.doc_13_r;
        case 1:
          return ctl->dc.doc_13_g;
        case 2:
          return ctl->dc.doc_13_b;
      }
    case 14:
      switch(color) {
        case 0:
          return ctl->dc.doc_14_r;
        case 1:
          return ctl->dc.doc_14_g;
        case 2:
          return ctl->dc.doc_14_b;
      }
    case 15:
      switch(color) {
        case 0:
          return ctl->dc.doc_15_r;
        case 1:
          return ctl->dc.doc_15_g;
        case 2:
          return ctl->dc.doc_15_b;
      }
  }
  return 0;
}