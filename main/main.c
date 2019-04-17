#include <freertos/FreeRTOS.h>
#include <esp_system.h>
#include <esp_event.h>
#include <esp_event_loop.h>
#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <time.h>
#include <stdbool.h>
#include <math.h>
#include <driver/ledc.h> 
#include <driver/spi_master.h> 
#include <string.h>

#include "tlc.h"

#define GPIO_PWM_CLK_OUT 32
#define GPIO_DATA_LATCH  33
#define GPIO_BLANK       25

#define SPI_MODE 0

#define LSB_FIRST
#define REVERSE

esp_err_t event_handler(void *ctx, system_event_t *event) {
    return ESP_OK;
}

#define HI(gpio) (gpio_set_level((gpio), 1))
#define LO(gpio) (gpio_set_level((gpio), 0))

#define KHZ_TO_HZ(HZ) ((HZ) * 1000UL)

void gpio_output(uint8_t gpio) {
  gpio_config_t conf = {
    .intr_type = GPIO_PIN_INTR_DISABLE,
    .mode = GPIO_MODE_OUTPUT,
    .pin_bit_mask = (1ull << gpio),
    .pull_down_en = 0,
    .pull_up_en = 0
  };
  gpio_config(&conf);
}

TLC_DECLARE_PWM_CHAIN(tlc_pwm2, 2);
TLC_DECLARE_DC_CHAIN(tlc_dc2, 2);

void tlc_dc_init(struct tlc_dc* dc) {
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

void hexdump(uint8_t* data, size_t len) {
  while(len-- > 0) {
    printf("%02X ", *data++);
  }
  printf("\n");
}

uint8_t tlc_reverse_buffer[256];
void tlc_xmit(spi_device_handle_t spi, void* data, size_t len) {
#ifdef REVERSE
  printf("Initial: ");
  hexdump((uint8_t*)data, len);
  assert(len <= sizeof(tlc_reverse_buffer));
  uint8_t* ptr = tlc_reverse_buffer;
  size_t remain = len;
  while(remain-- > 0) {
    *ptr++ = ((uint8_t*)data)[remain];
  }
  printf("Reversed: ");
  hexdump((uint8_t*)tlc_reverse_buffer, len);
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

#define DOC_SET(doc, val)\
  do {\
    doc##_r = (val);\
    doc##_g = (val);\
    doc##_b = (val);\
  } while (0);

#define DOC_RANGE_LO(doc)\
  do {\
    (doc).doc_range_r = 0;\
    (doc).doc_range_g = 0;\
    (doc).doc_range_b = 0;\
  } while (0);
  

void app_main(void) {
  esp_err_t err;

  // Initialize main PWM clock
  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .timer_num  = LEDC_TIMER_0,
    .bit_num    = 2,
    .freq_hz    = 100000
  };
 
  ledc_channel_config_t ledc_channel = {
    .channel    = LEDC_CHANNEL_0,
    .gpio_num   = GPIO_PWM_CLK_OUT,
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .timer_sel  = LEDC_TIMER_0,
    .duty       = 2
  };
 
  ledc_timer_config(&ledc_timer);
  ledc_channel_config(&ledc_channel);

  // General IO setup
  gpio_output(GPIO_DATA_LATCH);
  gpio_output(GPIO_BLANK);

  // Initialize GS SPI
  spi_bus_config_t buscfg_hspi = {
    .miso_io_num = 12,
    .mosi_io_num = 13,
    .sclk_io_num = 14,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 1024
  };

  err = spi_bus_initialize(HSPI_HOST, &buscfg_hspi, 1);
  printf("HSPI init finish, %d\n", err);
  ESP_ERROR_CHECK(err);

  spi_device_interface_config_t spi_devcfg_gs = {
    .clock_speed_hz = KHZ_TO_HZ(100),
    .mode = SPI_MODE,
    .spics_io_num = (int)-1,
    .queue_size = 8,
    .flags = SPI_DEVICE_HALFDUPLEX
  };

#ifdef LSBFIRST
  spi_devcfg_gs.flags |= SPI_DEVICE_TXBIT_LSBFIRST;
#endif

  printf("SPI CS pin: %d\n", spi_devcfg_gs.spics_io_num);

  spi_device_handle_t spi_dev_gs;
  err = spi_bus_add_device(HSPI_HOST, &spi_devcfg_gs, &spi_dev_gs);
  printf("GS slave init finish, %d\n", err);
  ESP_ERROR_CHECK(err);

  // Initialize DC SPI
  spi_bus_config_t buscfg_vspi = {
    .miso_io_num = 19,
    .mosi_io_num = 23,
    .sclk_io_num = 18,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 1024
  };

  err = spi_bus_initialize(VSPI_HOST, &buscfg_vspi, 0);
  printf("VSPI init finish, %d\n", err);
  ESP_ERROR_CHECK(err);

  spi_device_interface_config_t spi_devcfg_dc = {
    .clock_speed_hz = KHZ_TO_HZ(100),
    .mode = SPI_MODE,
    .spics_io_num = (int)-1,
    .queue_size = 8,
    .flags = SPI_DEVICE_HALFDUPLEX
  };

#ifdef LSBFIRST
  spi_devcfg_dc.flags |= SPI_DEVICE_TXBIT_LSBFIRST;
#endif

  spi_device_handle_t spi_dev_dc;
  err = spi_bus_add_device(VSPI_HOST, &spi_devcfg_dc, &spi_dev_dc);
  printf("DC slave init finish, %d\n", err);
  ESP_ERROR_CHECK(err);

  printf("Size of TLC PWM is %zu bytes\n", sizeof(union tlc_pwm));
  printf("Size of TLC DC is %zu bytes\n", sizeof(struct tlc_dc));
  printf("Size of TLC DC DOC is %zu bytes\n", sizeof((((struct tlc_dc*)(void*)0))->doc));

  uint8_t gs_data[72];
  memset(gs_data, 0xFF, sizeof(gs_data));

  memset(tlc_pwm2.data, 0x00, sizeof(tlc_pwm2.data));

  printf("Transfer size will be %u\n", sizeof(tlc_pwm2.data) * 8);

  struct spi_transaction_t trans_gs = {
    .length = sizeof(gs_data) * 8,
    .tx_buffer = gs_data
  };

  HI(GPIO_BLANK);

/*
  tlc_pwm2.tlc0.pwm_7_r = 0xFFF;
  tlc_pwm2.tlc0.pwm_7_g = 0xFFF;
  tlc_pwm2.tlc0.pwm_7_b = 0xFFF;
*/
  tlc_pwm2.tlc0.pwm_4_r = 0x0;
  tlc_pwm2.tlc0.pwm_2_g = 0x0;
  tlc_pwm2.tlc1.pwm_4_b = 0x0;
  tlc_pwm2.tlc1.pwm_1_g = 0x0;
  tlc_pwm2.tlc1.pwm_7_g = 0xFFF;

  tlc_dc_init(&tlc_dc2.tlc0);
  tlc_dc_init(&tlc_dc2.tlc1);

  uint8_t dimval = 5;

/*
  DOC_RANGE_LO(tlc_dc2.tlc0);
  DOC_RANGE_LO(tlc_dc2.tlc1);
  DOC_SET(tlc_dc2.tlc0.doc.doc_0, dimval)
  DOC_SET(tlc_dc2.tlc0.doc.doc_1, dimval)
  DOC_SET(tlc_dc2.tlc0.doc.doc_2, dimval)
  DOC_SET(tlc_dc2.tlc0.doc.doc_3, dimval)
  DOC_SET(tlc_dc2.tlc0.doc.doc_4, dimval)
  DOC_SET(tlc_dc2.tlc0.doc.doc_5, dimval)
  DOC_SET(tlc_dc2.tlc0.doc.doc_6, dimval)
  DOC_SET(tlc_dc2.tlc0.doc.doc_7, dimval)
  DOC_SET(tlc_dc2.tlc1.doc.doc_0, dimval)
  DOC_SET(tlc_dc2.tlc1.doc.doc_1, dimval)
  DOC_SET(tlc_dc2.tlc1.doc.doc_2, dimval)
  DOC_SET(tlc_dc2.tlc1.doc.doc_3, dimval)
  DOC_SET(tlc_dc2.tlc1.doc.doc_4, dimval)
  DOC_SET(tlc_dc2.tlc1.doc.doc_5, dimval)
  DOC_SET(tlc_dc2.tlc1.doc.doc_6, dimval)
  DOC_SET(tlc_dc2.tlc1.doc.doc_7, dimval)
*/
  while(1) {  
    LO(GPIO_BLANK);
//    spi_device_transmit(spi_dev_gs, &trans_gs);

    tlc_xmit(spi_dev_gs, tlc_pwm2.data, sizeof(tlc_pwm2.data));
    HI(GPIO_BLANK);
    HI(GPIO_DATA_LATCH);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    LO(GPIO_DATA_LATCH);

    printf("Sending DC data\n");
    tlc_xmit(spi_dev_dc, tlc_dc2.data, sizeof(tlc_dc2.data));


    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
