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
#define PWM_SPEED KHZ_TO_HZ(10000)

#ifndef max
  #define max(a, b) ((a) > (b) ? (a) : (b))
#endif

#define HI(gpio) (gpio_set_level((gpio), 1))
#define LO(gpio) (gpio_set_level((gpio), 0))

#define TAG "TLC"

struct tlc_chain tlc = { 0 };
static uint8_t* tlc_reverse_buffer = NULL;

static void tlc_ctl_init(struct tlc_ctl* ctl) {
  memset(ctl, 0, sizeof(*ctl));
  memset(ctl->dc.data, 0xFF, sizeof(ctl->dc.data));

  ctl->mcr = 0b101; // 19.1 mA
  ctl->mcg = 0b101; // 19.1 mA
  ctl->mcb = 0b101; // 19.1 mA

  ctl->bcr = 127;
  ctl->bcg = 127;
  ctl->bcb = 127;

	ctl->dsprpt = 1; // Auto-repeat enabled
	ctl->tmgrst = 1; // Disable outputs during data clock-in
	ctl->refresh = 1; // Refresh status data automatically
	ctl->espwm = 1; // Enable staggered PWM
	ctl->lsdvlt = 1; // Low short detect voltage

	ctl->ctl_cmd = 0b10010110;
}

static void tlc_gs_init(struct tlc_gs* gs) {
	(void)gs;
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

static esp_err_t tlc_gpio_init(int gpio_latch) {
  gpio_config_t conf = {
    .intr_type = GPIO_PIN_INTR_DISABLE,
    .mode = GPIO_MODE_OUTPUT,
    .pin_bit_mask = (1ull << gpio_latch),
    .pull_down_en = 0,
    .pull_up_en = 0
  };
  return gpio_config(&conf);
}

static esp_err_t tlc_spi_init(spi_host_device_t spi) {
  esp_err_t err;
  
  // Initialize SPI
  spi_bus_config_t buscfg = {
    .miso_io_num = 12,
    .mosi_io_num = 13,
    .sclk_io_num = 14,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 1024
  };

  err = spi_bus_initialize(spi, &buscfg, 1);
  ESP_LOGI(TAG, "GS SPI init finish, %d\n", err);
  if(err) {
    goto fail;
  }

  spi_device_interface_config_t spi_devcfg = {
    .clock_speed_hz = SPI_SPEED,
    .mode = SPI_MODE,
    .spics_io_num = -1,
    .queue_size = 8,
    .flags = SPI_DEVICE_HALFDUPLEX,
		.command_bits = 1
  };

#ifdef LSBFIRST
  spi_devcfg.flags |= SPI_DEVICE_TXBIT_LSBFIRST;
#endif

  err = spi_bus_add_device(spi, &spi_devcfg, &tlc.spi);
  ESP_LOGI(TAG, "GS slave init finish, %d\n", err);
  if(err) {
    goto fail;
  }

fail:
  return err;
}

esp_err_t tlc_init(size_t len, int gpio_pwmclk, int gpio_latch, spi_host_device_t spi) {
  esp_err_t err;
  int i;

  if(!(tlc.gs_data = calloc(len, sizeof(struct tlc_gs)))) {
    err = ESP_ERR_NO_MEM;
    goto fail;
  }
  if(!(tlc.ctl_data = calloc(len, sizeof(struct tlc_ctl)))) {
    err = ESP_ERR_NO_MEM;
    goto fail_pwm;
  }

  for(i = 0; i < len; i++) {
    tlc_gs_init(&tlc.gs_data[i]);
    tlc_ctl_init(&tlc.ctl_data[i]);
  }

  if(!(tlc_reverse_buffer = calloc(len, max(sizeof(struct tlc_gs), sizeof(struct tlc_ctl))))) {
    err = ESP_ERR_NO_MEM;
    goto fail_dc;
  }
  
  tlc.chain_len = len;

  if((err = tlc_pwm_init(gpio_pwmclk))) {
    goto fail_buff;
  }

  tlc.gpio.latch = gpio_latch;

  if((err = tlc_gpio_init(gpio_latch))) {
    goto fail_buff;
  }

  if((err = tlc_spi_init(spi))) {
    goto fail_buff;
  }

  return ESP_OK;

fail_buff:
  free(tlc_reverse_buffer);
fail_dc:
  free(tlc.ctl_data);
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

void tlc_xmit(spi_device_handle_t spi, uint8_t cmd, void* data, size_t len) {
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
    .tx_buffer = tlc_reverse_buffer,
		.cmd = cmd
  };
#else
  struct spi_transaction_t trans = {
    .length = len * 8,
    .tx_buffer = data,
		.cmd = cmd
  };
#endif
  spi_device_transmit(spi, &trans);
}

void tlc_update_task(void* args) {
  while(1) {
    tlc_xmit(tlc.spi, 0, tlc.gs_data, sizeof(struct tlc_gs) * tlc.chain_len);
    HI(tlc.gpio.latch);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    LO(tlc.gpio.latch);

//    ESP_LOGI(TAG, "Sending DC data\n");
//		memset(tlc.ctl_data, 0, sizeof(*tlc.ctl_data));
    tlc_xmit(tlc.spi, 1, tlc.ctl_data, sizeof(struct tlc_ctl) * tlc.chain_len);
    HI(tlc.gpio.latch);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    LO(tlc.gpio.latch);

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}