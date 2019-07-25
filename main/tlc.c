#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <stdlib.h>
#include <stdio.h>
#include <esp_log.h>
#include <string.h>

#include <driver/ledc.h>

#include "tlc.h"
#include "util.h"
#include "tlc_power_gov.h"

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

  ctl->mcr = 0b100; // 19.1 mA
  ctl->mcg = 0b100; // 19.1 mA
  ctl->mcb = 0b100; // 19.1 mA

  ctl->bcr = 127;
  ctl->bcg = 127;
  ctl->bcb = 127;

  ctl->dsprpt = 1; // Auto-repeat enabled
  ctl->tmgrst = 1; // Disable outputs during data clock-in
  ctl->refresh = 1; // Refresh status data automatically
  ctl->espwm = 1; // Enable staggered PWM
  ctl->lsdvlt = 1; // Low short detect voltage

  ctl->ctl_cmd = 0b10010110;
  ctl->one = 1;
}

static void tlc_gs_init(struct tlc_gs* gs) {
  gs->zero = 0;
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
  };

#ifdef LSB_FIRST
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

  ESP_LOGI(TAG, "Enabling VCC0\n");
  tlc_gpio_init(22);
  HI(22);	

  ESP_LOGI(TAG, "Enabling VCC1\n");
  tlc_gpio_init(27);
  HI(27);	



  if(!(tlc.gs_data = calloc(len, sizeof(struct tlc_gs)))) {
    err = ESP_ERR_NO_MEM;
    goto fail;
  }
  if(!(tlc.ctl_data = calloc(len, sizeof(struct tlc_ctl)))) {
    err = ESP_ERR_NO_MEM;
    goto fail_pwm;
  }
  if(!(tlc.pwr_gov = calloc(len, sizeof(struct tlc_power_gov)))) {
    err = ESP_ERR_NO_MEM;
    goto fail_dc;
  }

  struct tlc_led_spec led_spec = {
    .u_forward = 2500,
    .i_cont    = 25000,
  };

  for(i = 0; i < len; i++) {
    tlc_gs_init(&tlc.gs_data[i]);
    tlc_ctl_init(&tlc.ctl_data[i]);
    tlc_power_gov_init(&tlc.pwr_gov[i], 4000, 2000, &tlc.gs_data[i], &tlc.ctl_data[i]);
    for(int j = 0; j < 16; j++) {
      for(int k = 0; k < 3; k++) {
        tlc_power_gov_setup_led(&tlc.pwr_gov[i], j, k, led_spec);
      }
    }
  }

  if(!(tlc_reverse_buffer = calloc(len, max(sizeof(struct tlc_gs), sizeof(struct tlc_ctl))))) {
    err = ESP_ERR_NO_MEM;
    goto fail_pwr_gov;
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
fail_pwr_gov:
  free(tlc.pwr_gov);
fail_dc:
  free(tlc.ctl_data);
fail_pwm:
  free(tlc.gs_data);
fail:
  return err;
}

void tlc_xmit(spi_device_handle_t spi, void* data, size_t len) {
#ifdef REVERSE
#ifdef XMIT_DEBUG
  printf("Initial: ");
  hexdump((uint8_t*)data, len);
#endif
  // Major pain, have to perform a 7 bit bitshift on the whole buffer ...
  uint8_t* ptr = tlc_reverse_buffer;
  size_t remain = len;
  while(remain-- > 0) {
#ifndef LSB_FIRST
    *ptr++ = ((((uint8_t*)data)[remain] << 7)) | (remain > 0 ? (((uint8_t*)data)[remain - 1] >> 1) : 0);
#else
    *ptr++ = ((((uint8_t*)data)[remain] >> 7)) | (remain > 0 ? (((uint8_t*)data)[remain - 1] << 1) : 0);
#endif
  }
#ifdef XMIT_DEBUG
  printf("Reversed: ");
  hexdump((uint8_t*)tlc_reverse_buffer, len);
#endif
  struct spi_transaction_t trans = {
    .length = len * 8 - 7,
    .tx_buffer = tlc_reverse_buffer,
  };
#else
  struct spi_transaction_t trans = {
    .length = len * 8 - 7,
    .tx_buffer = data,
  };
#endif
  spi_device_transmit(spi, &trans);
}

void tlc_xmitn(spi_device_handle_t spi, void* data, size_t len, size_t num) {
  while(num-- > 0) {
    tlc_xmit(spi, data, len);
    data += len;
  }
}

void tlc_update_task(void* args) {
  while(1) {
    for(int i = 0; i < tlc.chain_len; i++) {
      tlc_power_gov_govern(&tlc.pwr_gov[i], 10000);
    }
    tlc_xmitn(tlc.spi, tlc.gs_data, sizeof(struct tlc_gs), tlc.chain_len);
    HI(tlc.gpio.latch);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    LO(tlc.gpio.latch);

//    ESP_LOGI(TAG, "Sending DC data\n");
    tlc_xmitn(tlc.spi, tlc.ctl_data, sizeof(struct tlc_ctl), tlc.chain_len);
    HI(tlc.gpio.latch);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    LO(tlc.gpio.latch);

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

uint8_t tlc_ctl_get_bc(struct tlc_ctl* ctl, uint8_t color) {
  if(color >= 3) {
    ESP_LOGE(TAG, "Color channel %u does not exist", color);
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
    ESP_LOGE(TAG, "Color channel %u does not exist", color);
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

  ESP_LOGE(TAG, "Invalid maximum current configuration on color %u: %u", color, mcr);
  return 0;
};

uint8_t tlc_ctl_get_doc(struct tlc_ctl* ctl, uint8_t channel, uint8_t color) {
  if(channel >= 16) {
    ESP_LOGE(TAG, "Channel %u does not exist", channel);
    return 0;
  }

  if(color >= 3) {
    ESP_LOGE(TAG, "Color channel %u does not exist", color);
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