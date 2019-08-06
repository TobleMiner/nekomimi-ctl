#include "ear.h"
#include "util.h"

#include "earmap.h"

esp_err_t ear_init(struct tlc_chain* tlc, size_t num_ears, int gpio_pwmclk, int gpio_latch, spi_host_device_t spi) {
  return tlc_init(tlc, num_ears, gpio_pwmclk, gpio_latch, spi);
}

esp_err_t ear_set_led_rgb(struct tlc_chain* ears, unsigned int ear, unsigned int led, uint16_t red, uint16_t green, uint16_t blue) {
  struct tlc_gs* chip;
  struct tlc_gs_chan* chan;

  if(ear >= ears->chain_len) {
    return ESP_ERR_INVALID_ARG;
  }

  if(led >= ARRAY_LEN(rgb_led_map)) {
    return ESP_ERR_INVALID_ARG;
  }

  chip = &ears->gs_data[ear];
  chan = &chip->channels[rgb_led_map[led]];
  chan->r = red;
  chan->g = green;
  chan->b = blue;
  return ESP_OK;
}

#define COL8_TO_PWM(c) (((uint16_t)(c)) * 257U)

esp_err_t ear_set_led_rgb888(struct tlc_chain* ears, unsigned int ear, unsigned int led, uint8_t red, uint8_t green, uint8_t blue) {
  return ear_set_led_rgb(ears, ear, led, COL8_TO_PWM(red), COL8_TO_PWM(green), COL8_TO_PWM(blue));
}

esp_err_t ear_set_led_uv(struct tlc_chain* ears, unsigned int ear, unsigned int led, uint16_t level) {
  struct tlc_gs* chip;
  struct tlc_gs_chan* chan;

  if(ear >= ears->chain_len) {
    return ESP_ERR_INVALID_ARG;
  }

  if(led >= ARRAY_LEN(uv_led_map) || led >= ARRAY_LEN(uv_chan_map)) {
    return ESP_ERR_INVALID_ARG;
  }

  chip = &ears->gs_data[ear];
  chan = &chip->channels[uv_led_map[led]];
  chan->channels[uv_chan_map[led]] = level;
  return ESP_OK;
}

esp_err_t ear_set_led_uv8(struct tlc_chain* ears, unsigned int ear, unsigned int led, uint8_t level) {
  return ear_set_led_uv(ears, ear, led, COL8_TO_PWM(level));
}
