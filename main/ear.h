#pragma once

#include <esp_err.h>

#include "tlc.h"

#define NUM_LEDS_PER_EAR 13
#define NUM_UV_PER_EAR    8

esp_err_t ear_init(struct tlc_chain* tlc, size_t num_ears, int gpio_pwmclk, int gpio_latch, spi_host_device_t spi);
esp_err_t ear_set_led_rgb(struct tlc_chain* ears, unsigned int ear, unsigned int led, uint16_t red, uint16_t green, uint16_t blue);
esp_err_t ear_set_led_rgb888(struct tlc_chain* ears, unsigned int ear, unsigned int led, uint8_t red, uint8_t green, uint8_t blue);
esp_err_t ear_set_led_uv(struct tlc_chain* ears, unsigned int ear, unsigned int led, uint16_t level);
esp_err_t ear_set_led_uv8(struct tlc_chain* ears, unsigned int ear, unsigned int led, uint8_t level);
