#pragma once

#include <esp_err.h>
#include <stdint.h>

#include <driver/spi_master.h>

/*
#include "loop.h"

union tlc_pwm {
  struct {
    PWM_CHAN(pwm_0);
    PWM_CHAN(pwm_1);
    PWM_CHAN(pwm_2);
    PWM_CHAN(pwm_3);
    PWM_CHAN(pwm_4);
    PWM_CHAN(pwm_5);
    PWM_CHAN(pwm_6);
    PWM_CHAN(pwm_7);
  } __attribute__((packed));
  uint8_t data[36];
} __attribute__((packed));

#define TLC_DECLARE_PWM_(i, _) union tlc_pwm tlc##i;
#define TLC_DECLARE_PWM_CHAIN(NAME, LENGTH) \
  union { \
    struct { \
      EVAL(REPEAT(LENGTH, TLC_DECLARE_PWM_, ~)) \
    } __attribute__((packed)); \
    uint8_t data[sizeof(union tlc_pwm) * LENGTH]; \
  }  __attribute__((packed)) NAME


struct doc_channel {
  uint8_t r:7;
  uint8_t g:7;
  uint8_t b:7;
} __attribute__((packed));

struct tlc_dc {
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
    } __attribute__((packed));
    uint8_t data[21];
  } __attribute__((packed)) doc;

  uint8_t gbc_r:8;
  uint8_t gbc_g:8;
  uint8_t gbc_b:8;

  uint8_t doc_range_r:1;
  uint8_t doc_range_g:1;
  uint8_t doc_range_b:1;
  uint8_t repeat:1;
  uint8_t timer_rst:1;
  uint8_t gs_cnt_mode:2;

  uint32_t user_data:17;
} __attribute__((packed));

#define TLC_DECLARE_DC_(i, _) struct tlc_dc tlc##i;
#define TLC_DECLARE_DC_CHAIN(NAME, LENGTH) \
  union { \
    struct { \
      EVAL(REPEAT(LENGTH, TLC_DECLARE_DC_, ~)) \
    } __attribute__((packed)); \
    uint8_t data[sizeof(struct tlc_dc) * LENGTH]; \
  }  __attribute__((packed)) NAME
*/


#define PWM_CHAN(name)\
  uint16_t name##_r:12;\
  uint16_t name##_g:12;\
  uint16_t name##_b:12;

// 8 * 36 bits => 36 bytes
struct tlc_pwm {
  PWM_CHAN(pwm_0);
  PWM_CHAN(pwm_1);
  PWM_CHAN(pwm_2);
  PWM_CHAN(pwm_3);
  PWM_CHAN(pwm_4);
  PWM_CHAN(pwm_5);
  PWM_CHAN(pwm_6);
  PWM_CHAN(pwm_7);
} __attribute__((packed));


#define DOC_CHAN(name)\
  uint8_t name##_r:7;\
  uint8_t name##_g:7;\
  uint8_t name##_b:7;

// 27 bytes
struct tlc_dc {
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
    } __attribute__((packed));
    uint8_t data[21];
  } __attribute__((packed)) doc;

  uint8_t gbc_r:8;
  uint8_t gbc_g:8;
  uint8_t gbc_b:8;

  uint8_t doc_range_r:1;
  uint8_t doc_range_g:1;
  uint8_t doc_range_b:1;
  uint8_t repeat:1;
  uint8_t timer_rst:1;
  uint8_t gs_cnt_mode:2;

  uint32_t user_data:17;

/* Virtual extension during shift-out
  uint8_t thermal_shdn:1;
  uint8_t short_r:8;
  uint8_t short_g:8;
  uint8_t short_b:8;
  uint8_t open_r:8;
  uint8_t open_g:8;
  uint8_t open_b:8;
*/
} __attribute__((packed));

struct tlc_chain {
  struct tlc_pwm* gs_data;
  struct tlc_dc*  dc_data;
  size_t          chain_len;
  struct {
    spi_device_handle_t gs;
    spi_device_handle_t dc;
  } spi;
  struct {
    int blank;
    int latch;
  } gpio;
};

esp_err_t tlc_init(size_t len, int gpio_pwmclk, int gpio_latch, int gpio_blank, spi_host_device_t spi_gs, spi_host_device_t spi_dc);
void tlc_update_task(void* args);

extern struct tlc_chain tlc;