#pragma once

#include <esp_err.h>
#include <stdint.h>

#include <driver/spi_master.h>

#define TLC_TAG "TLC5955"

#define TLC_STACK 4096

#define SPI_MODE 0
//#define LSB_FIRST
#define REVERSE
//#define XMIT_DEBUG


#define PWM_CHAN(name)\
  uint16_t name##_r:16;\
  uint16_t name##_g:16;\
  uint16_t name##_b:16;

struct tlc_gs_chan {
  union {
    struct {
      uint16_t r;
      uint16_t g;
      uint16_t b;
    };
    uint16_t channels[3];
  };
} __attribute__((packed));

struct tlc_gs {
  struct tlc_gs_chan channels[16];

#ifdef LSB_FIRST
  uint8_t pad_id:7;
#endif
  uint8_t zero:1;
} __attribute__((packed));


#define DOC_CHAN(name)\
  uint8_t name##_r:7;\
  uint8_t name##_g:7;\
  uint8_t name##_b:7;

struct tlc_ctl {
  union {
    struct {
      DOC_CHAN(doc_0);
      DOC_CHAN(doc_1);
      DOC_CHAN(doc_2);
      DOC_CHAN(doc_3);
      DOC_CHAN(doc_4);
      DOC_CHAN(doc_5);
      DOC_CHAN(doc_6);
      DOC_CHAN(doc_7);
      DOC_CHAN(doc_8);
      DOC_CHAN(doc_9);
      DOC_CHAN(doc_10);
      DOC_CHAN(doc_11);
      DOC_CHAN(doc_12);
      DOC_CHAN(doc_13);
      DOC_CHAN(doc_14);
      DOC_CHAN(doc_15);
    } __attribute__((packed));
    uint8_t data[42];
  } __attribute__((packed)) dc;

  uint8_t mcr:3;
  uint8_t mcg:3;
  uint8_t mcb:3;

  uint8_t bcr:7;
  uint8_t bcg:7;
  uint8_t bcb:7;

  uint8_t dsprpt:1;
  uint8_t tmgrst:1;
  uint8_t refresh:1;
  uint8_t espwm:1;
  uint8_t lsdvlt:1;

  uint8_t bitpad:5;
  uint8_t pad[48];

  uint8_t ctl_cmd; // Must always be set to 0b10010110

#ifdef LSB_FIRST
  uint8_t pad_id:7;
#endif
  uint8_t one:1;
} __attribute__((packed));


struct tlc_chain {
  struct tlc_gs*        gs_data;
  struct tlc_ctl*       ctl_data;
  struct tlc_power_gov* pwr_gov;
  size_t                chain_len;
  uint8_t*              tlc_reverse_buffer;
  spi_device_handle_t   spi;
  struct {
    int latch;
  } gpio;
};

#define TLC_COLOR_CHANNEL_RED   0b001
#define TLC_COLOR_CHANNEL_GREEN 0b010
#define TLC_COLOR_CHANNEL_BLUE  0b100
#define TLC_COLOR_CHANNEL_ALL   (TLC_COLOR_CHANNEL_RED | TLC_COLOR_CHANNEL_GREEN | TLC_COLOR_CHANNEL_BLUE)

typedef uint8_t tlc_col_chan_t;


#define TLC_CURRENT_3_2_mA  0b000
#define TLC_CURRENT_8_0_mA  0b001
#define TLC_CURRENT_11_2_mA 0b010
#define TLC_CURRENT_15_9_mA 0b011
#define TLC_CURRENT_19_1_mA 0b100
#define TLC_CURRENT_23_9_mA 0b101
#define TLC_CURRENT_27_1_mA 0b110
#define TLC_CURRENT_31_9_mA 0b111
#define _TLC_CURRENT_LIMIT  TLC_CURRENT_31_9_mA

typedef uint8_t tlc_current_t;

esp_err_t tlc_init(struct tlc_chain* tlc, size_t len, int gpio_pwmclk, int gpio_latch, spi_host_device_t spi);

esp_err_t tlc_set_led_current(struct tlc_chain* tlc, unsigned int index, tlc_col_chan_t chan, tlc_current_t current);
esp_err_t tlc_set_led_current_all(struct tlc_chain* tlc, tlc_col_chan_t chan, tlc_current_t current);
