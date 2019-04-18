#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <stdlib.h>
#include <stdio.h>
#include <esp_log.h>
#include <string.h>

#include <driver/ledc.h>

#include "tlc.h"

#define SPI_MODE 0
#define LSB_FIRST
#define REVERSE
//#define XMIT_DEBUG

#define KHZ_TO_HZ(HZ) ((HZ) * 1000UL)

#define SPI_SPEED KHZ_TO_HZ(1000)
#define PWM_SPEED KHZ_TO_HZ(100)

#ifndef max
  #define max(a, b) ((a) > (b) ? (a) : (b))
#endif

#define HI(gpio) (gpio_set_level((gpio), 1))
#define LO(gpio) (gpio_set_level((gpio), 0))

#define TAG "TLC"

struct tlc_chain tlc = { 0 };
static uint8_t* tlc_reverse_buffer = NULL;

static void tlc_dc_init(struct tlc_dc* dc) {
  memset(dc, 0, sizeof(*dc));
  memset(dc->doc.data, 0xFF, sizeof(dc->doc.data));

  dc->gbc_r = 127;
  dc->gbc_g = 127;
  dc->gbc_b = 127;

  dc->doc_range_r = 1;
  dc->doc_range_g = 1;
  dc->doc_range_b = 1;
  dc->repeat = 1;
  dc->timer_rst = 0;
  dc->gs_cnt_mode = 3;
}

static esp_err_t tlc_pwm_init(int gpio) {
  esp_err_t err;
  // Initialize main PWM clock
  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .timer_num  = LEDC_TIMER_0,
    .bit_num    = 2,
    .freq_hz    = PWM_SPEED
  };
 
  ledc_channel_config_t ledc_channel = {
    .channel    = LEDC_CHANNEL_0,
    .gpio_num   = gpio,
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .timer_sel  = LEDC_TIMER_0,
    .duty       = 2
  };

  if((err = ledc_timer_config(&ledc_timer))) {
    goto fail;
  }
  if((err = ledc_channel_config(&ledc_channel))) {
    goto fail;
  }

fail:
  return err;
}

static esp_err_t tlc_gpio_init(int gpio_latch, int gpio_blank) {
  gpio_config_t conf = {
    .intr_type = GPIO_PIN_INTR_DISABLE,
    .mode = GPIO_MODE_OUTPUT,
    .pin_bit_mask = (1ull << gpio_latch) | (1ull << gpio_blank),
    .pull_down_en = 0,
    .pull_up_en = 0
  };
  return gpio_config(&conf);
}

static esp_err_t tlc_spi_init(spi_host_device_t spi_gs, spi_host_device_t spi_dc) {
  esp_err_t err;
  
  // Initialize GS SPI
  spi_bus_config_t buscfg_gs = {
    .miso_io_num = 12,
    .mosi_io_num = 13,
    .sclk_io_num = 14,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 1024
  };

  err = spi_bus_initialize(spi_gs, &buscfg_gs, 1);
  ESP_LOGI(TAG, "GS SPI init finish, %d\n", err);
  if(err) {
    goto fail;
  }

  spi_device_interface_config_t spi_devcfg_gs = {
    .clock_speed_hz = SPI_SPEED,
    .mode = SPI_MODE,
    .spics_io_num = -1,
    .queue_size = 8,
    .flags = SPI_DEVICE_HALFDUPLEX
  };

#ifdef LSBFIRST
  spi_devcfg_gs.flags |= SPI_DEVICE_TXBIT_LSBFIRST;
#endif

  err = spi_bus_add_device(spi_gs, &spi_devcfg_gs, &tlc.spi.gs);
  ESP_LOGI(TAG, "GS slave init finish, %d\n", err);
  if(err) {
    goto fail;
  }

  // Initialize DC SPI
  spi_bus_config_t buscfg_dc = {
    .miso_io_num = 19,
    .mosi_io_num = 23,
    .sclk_io_num = 18,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 1024
  };

  err = spi_bus_initialize(spi_dc, &buscfg_dc, 2);
  ESP_LOGI(TAG, "DC SPI init finish, %d\n", err);
  if(err) {
    goto fail;
  }

  spi_device_interface_config_t spi_devcfg_dc = {
    .clock_speed_hz = SPI_SPEED,
    .mode = SPI_MODE,
    .spics_io_num = -1,
    .queue_size = 8,
    .flags = SPI_DEVICE_HALFDUPLEX
  };

#ifdef LSBFIRST
  spi_devcfg_dc.flags |= SPI_DEVICE_TXBIT_LSBFIRST;
#endif

  err = spi_bus_add_device(spi_dc, &spi_devcfg_dc, &tlc.spi.dc);
  ESP_LOGI(TAG, "DC slave init finish, %d\n", err);
  if(err) {
    goto fail;
  }

fail:
  return err;
}

esp_err_t tlc_init(size_t len, int gpio_pwmclk, int gpio_latch, int gpio_blank, spi_host_device_t spi_gs, spi_host_device_t spi_dc) {
  esp_err_t err;
  int i;

  if(!(tlc.gs_data = calloc(len, sizeof(struct tlc_pwm)))) {
    err = ESP_ERR_NO_MEM;
    goto fail;
  }
  if(!(tlc.dc_data = calloc(len, sizeof(struct tlc_dc)))) {
    err = ESP_ERR_NO_MEM;
    goto fail_pwm;
  }

  for(i = 0; i < len; i++) {
    tlc_dc_init(&tlc.dc_data[i]);
  }

  if(!(tlc_reverse_buffer = calloc(len, max(sizeof(struct tlc_pwm), sizeof(struct tlc_dc))))) {
    err = ESP_ERR_NO_MEM;
    goto fail_dc;
  }
  
  tlc.chain_len = len;

  if((err = tlc_pwm_init(gpio_pwmclk))) {
    goto fail_buff;
  }

  tlc.gpio.blank = gpio_blank;
  tlc.gpio.latch = gpio_latch;

  if((err = tlc_gpio_init(gpio_latch, gpio_blank))) {
    goto fail_buff;
  }

  if((err = tlc_spi_init(spi_gs, spi_dc))) {
    goto fail_buff;
  }

  return ESP_OK;

fail_buff:
  free(tlc_reverse_buffer);
fail_dc:
  free(tlc.dc_data);
fail_pwm:
  free(tlc.gs_data);
fail:
  return err;
}

void hexdump(uint8_t* data, size_t len) {
  while(len-- > 0) {
    printf("%02X ", *data++);
  }
  printf("\n");
}

void tlc_xmit(spi_device_handle_t spi, void* data, size_t len) {
#ifdef REVERSE
#ifdef XMIT_DEBUG
  printf("Initial: ");
  hexdump((uint8_t*)data, len);
#endif
  uint8_t* ptr = tlc_reverse_buffer;
  size_t remain = len;
  while(remain-- > 0) {
    *ptr++ = ((uint8_t*)data)[remain];
  }
#ifdef XMIT_DEBUG
  printf("Reversed: ");
  hexdump((uint8_t*)tlc_reverse_buffer, len);
#endif
  struct spi_transaction_t trans = {
    .length = len * 8,
    .tx_buffer = tlc_reverse_buffer
  };
#else
  struct spi_transaction_t trans = {
    .length = len * 8,
    .tx_buffer = data
  };
#endif
  spi_device_transmit(spi, &trans);
}

void tlc_update_task(void* args) {
  HI(tlc.gpio.blank);

  while(1) {
    LO(tlc.gpio.blank);

    tlc_xmit(tlc.spi.gs, tlc.gs_data, sizeof(struct tlc_pwm) * tlc.chain_len);
    HI(tlc.gpio.blank);
    HI(tlc.gpio.latch);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    LO(tlc.gpio.latch);

//    ESP_LOGI(TAG, "Sending DC data\n");
    tlc_xmit(tlc.spi.dc, tlc.dc_data, sizeof(struct tlc_dc) * tlc.chain_len);


    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}