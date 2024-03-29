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
#include "config.h"

#define KHZ_TO_HZ(HZ) ((HZ) * 1000UL)

#define SPI_SPEED KHZ_TO_HZ(1000)
#define PWM_SPEED KHZ_TO_HZ(10000)

#ifndef max
  #define max(a, b) ((a) > (b) ? (a) : (b))
#endif

#define HI(gpio) (gpio_set_level((gpio), 1))
#define LO(gpio) (gpio_set_level((gpio), 0))

static void tlc_ctl_init(struct tlc_ctl* ctl) {
  memset(ctl, 0, sizeof(*ctl));
  memset(ctl->dc.data, 0xFF, sizeof(ctl->dc.data));

  ctl->mcr = 0b000; // 3.2 mA
  ctl->mcg = 0b000; // 3.2 mA
  ctl->mcb = 0b000; // 3.2 mA

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

static esp_err_t tlc_spi_init(struct tlc_chain* tlc, spi_host_device_t spi) {
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
  ESP_LOGI(TLC_TAG, "GS SPI init finish, %d\n", err);
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

  err = spi_bus_add_device(spi, &spi_devcfg, &tlc->spi);
  ESP_LOGI(TLC_TAG, "GS slave init finish, %d\n", err);
  if(err) {
    goto fail;
  }

fail:
  return err;
}

static void tlc_xmit(struct tlc_chain* tlc, void* data, size_t len) {
#ifdef REVERSE
#ifdef XMIT_DEBUG
  printf("Initial: ");
  hexdump((uint8_t*)data, len);
#endif
  // Major pain, have to perform a 7 bit bitshift on the whole buffer ...
  uint8_t* ptr = tlc->tlc_reverse_buffer;
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
  hexdump((uint8_t*)tlc->tlc_reverse_buffer, len);
#endif
  struct spi_transaction_t trans = {
    .length = len * 8 - 7,
    .tx_buffer = tlc->tlc_reverse_buffer,
  };
#else
  struct spi_transaction_t trans = {
    .length = len * 8 - 7,
    .tx_buffer = data,
  };
#endif
  spi_device_transmit(tlc->spi, &trans);
}

static void tlc_xmitn(struct tlc_chain* tlc, void* data, size_t len, size_t num) {
  while(num-- > 0) {
    tlc_xmit(tlc, data, len);
    data += len;
  }
}

static void tlc_sleep(struct tlc_chain* tlc, uint64_t time_us) {
  esp_timer_start_once(tlc->timer, time_us);
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

static void tlc_update_task(void* arg) {
  struct tlc_chain* tlc = arg;
  while(1) {
    int64_t update_start = esp_timer_get_time();
    int64_t update_dt;
    int64_t sleep_time;
    for(int i = 0; i < tlc->chain_len; i++) {
      uint8_t dim;
      tlc->ctl_data[i].bcr = 127;
      tlc->ctl_data[i].bcg = 127;
      tlc->ctl_data[i].bcb = 127;
//      ESP_LOGI(TLC_TAG, "Pre dim power: %u mW", tlc_power_gov_current_power_mw(&tlc->pwr_gov[i]));
      dim = (uint8_t)(141U * (uint16_t)(CLAMP(tlc_power_gov_govern(&tlc->pwr_gov[i], 10000), 10, 100) - 10) / 100U);
//      ESP_LOGI(TLC_TAG, "Dimming to %u\n", dim);
      tlc->ctl_data[i].bcr = dim;
      tlc->ctl_data[i].bcg = dim;
      tlc->ctl_data[i].bcb = dim;
//      ESP_LOGI(TLC_TAG, "Post dim power: %u mW", tlc_power_gov_current_power_mw(&tlc->pwr_gov[i]));
    }
    tlc_xmitn(tlc, tlc->gs_data, sizeof(struct tlc_gs), tlc->chain_len);
    HI(tlc->gpio.latch);
    tlc_sleep(tlc, 1);
//    vTaskDelay(1 / portTICK_PERIOD_MS);
    LO(tlc->gpio.latch);

    tlc_xmitn(tlc, tlc->ctl_data, sizeof(struct tlc_ctl), tlc->chain_len);
    HI(tlc->gpio.latch);
    tlc_sleep(tlc, 1);
//    vTaskDelay(1 / portTICK_PERIOD_MS);
    LO(tlc->gpio.latch);

    update_dt = esp_timer_get_time() - update_start;
    sleep_time = 10000LL - update_dt;
    if(sleep_time > 0) {
      tlc_sleep(tlc, sleep_time);
    }
//    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}


static void tlc_timer_cb(void* arg) {
  struct tlc_chain* tlc = arg;
  xTaskNotifyGive(tlc->task);
};

// Public API
esp_err_t tlc_init(struct tlc_chain* tlc, size_t len, int gpio_pwmclk, int gpio_latch, spi_host_device_t spi) {
  esp_err_t err;
  int i;
  esp_timer_create_args_t timer_args = {
    .callback = tlc_timer_cb,
    .arg = tlc,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "tlc_timer",
  };

  memset(tlc, 0, sizeof(*tlc));

  ESP_LOGI(TLC_TAG, "Enabling VCC0\n");
  tlc_gpio_init(22);
  HI(22);

  ESP_LOGI(TLC_TAG, "Enabling VCC1\n");
  tlc_gpio_init(27);
  HI(27);

  if((err = esp_timer_create(&timer_args, &tlc->timer))) {
    goto fail;
  }


  if(!(tlc->gs_data = calloc(len, sizeof(struct tlc_gs)))) {
    err = ESP_ERR_NO_MEM;
    goto fail_timer;
  }
  if(!(tlc->ctl_data = calloc(len, sizeof(struct tlc_ctl)))) {
    err = ESP_ERR_NO_MEM;
    goto fail_pwm;
  }
  if(!(tlc->pwr_gov = calloc(len, sizeof(struct tlc_power_gov)))) {
    err = ESP_ERR_NO_MEM;
    goto fail_dc;
  }

  struct tlc_led_spec led_spec = {
    .u_forward = 2500,
    .i_cont    = 25000,
  };

  for(i = 0; i < len; i++) {
    tlc_gs_init(&tlc->gs_data[i]);
    tlc_ctl_init(&tlc->ctl_data[i]);
    tlc_power_gov_init(&tlc->pwr_gov[i], EAR_VOLTAGE_MV, EAR_TDP_MW, &tlc->gs_data[i], &tlc->ctl_data[i]);
    for(int j = 0; j < 16; j++) {
      for(int k = 0; k < 3; k++) {
        tlc_power_gov_setup_led(&tlc->pwr_gov[i], j, k, led_spec);
      }
    }
  }

  if(!(tlc->tlc_reverse_buffer = calloc(len, max(sizeof(struct tlc_gs), sizeof(struct tlc_ctl))))) {
    err = ESP_ERR_NO_MEM;
    goto fail_pwr_gov;
  }

  tlc->chain_len = len;

  if((err = tlc_pwm_init(gpio_pwmclk))) {
    goto fail_buff;
  }

  tlc->gpio.latch = gpio_latch;

  if((err = tlc_gpio_init(gpio_latch))) {
    goto fail_buff;
  }

  if((err = tlc_spi_init(tlc, spi))) {
    goto fail_buff;
  }

  if((err = xTaskCreate(tlc_update_task, "tlc_srv", TLC_STACK, tlc, 12, &tlc->task)) != pdPASS) {
    goto fail_buff;
  }

  return ESP_OK;

fail_buff:
  free(tlc->tlc_reverse_buffer);
fail_pwr_gov:
  free(tlc->pwr_gov);
fail_dc:
  free(tlc->ctl_data);
fail_pwm:
  free(tlc->gs_data);
fail_timer:
  esp_timer_delete(tlc->timer);
fail:
  return err;
}

esp_err_t tlc_set_led_current(struct tlc_chain* tlc, unsigned int index, tlc_col_chan_t chan, tlc_current_t current) {
  struct tlc_ctl* ctl;
  if(current > _TLC_CURRENT_LIMIT) {
    return ESP_ERR_INVALID_ARG;
  }

  if(index >= tlc->chain_len) {
    return ESP_ERR_INVALID_ARG;
  }

  ctl = &tlc->ctl_data[index];
  if(chan & TLC_COLOR_CHANNEL_RED) {
    ctl->mcr = current;
  }

  if(chan & TLC_COLOR_CHANNEL_GREEN) {
    ctl->mcg = current;
  }

  if(chan & TLC_COLOR_CHANNEL_BLUE) {
    ctl->mcb = current;
  }

  return ESP_OK;
}

esp_err_t tlc_set_led_current_all(struct tlc_chain* tlc, tlc_col_chan_t chan, tlc_current_t current) {
  esp_err_t err;
  unsigned int i;
  for(i = 0; i < tlc->chain_len; i++) {
    err = tlc_set_led_current(tlc, i, chan, current);
    if(err) {
      return err;
    }
  }

  return ESP_OK;
}
