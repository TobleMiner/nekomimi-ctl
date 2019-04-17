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

struct pwm {
  uint16_t val:12;
} __attribute__((packed));

#define PWM_CHAN(name)\
  uint16_t name##_r:12;\
  uint16_t name##_g:12;\
  uint16_t name##_b:12;

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

union {
  struct {
    union tlc_pwm tlc1;
    union tlc_pwm tlc2
  } __attribute__((packed)) tlcs;
  uint8_t data[72];
} tlc_pwm2;

struct doc_channel {
  uint8_t r:7;
  uint8_t g:7;
  uint8_t b:7;
} __attribute__((packed));

#pragma pack(1)

#define DOC_CHAN(name)\
  uint8_t name##_r:7;\
  uint8_t name##_g:7;\
  uint8_t name##_b:7;

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

union {
  struct {
    struct tlc_dc tlc1;
    struct tlc_dc tlc2;
  } __attribute__((packed)) tlcs;
  uint8_t data[54];
} tlc_dc2;

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

  uint8_t gs_data[36];
  uint8_t dc_data[54];

//  memset(gs_data, 0xFF, sizeof(gs_data));
//  memset(dc_data, 0xFF, sizeof(dc_data));
//  memset(gs_data, 0x42, sizeof(gs_data));
//  memset(dc_data, 0x42, sizeof(dc_data));

  for(int i = 0; i < sizeof(gs_data); i++) {
    gs_data[i] = i;
  }

  for(int i = 0; i < sizeof(dc_data); i++) {
    dc_data[i] = i;
  }

  memset(gs_data, 0b01010101, sizeof(gs_data));
  memset(dc_data, 0b01010101, sizeof(dc_data));
  memset(gs_data, 0xFF, sizeof(gs_data));
  memset(dc_data, 0xFF, sizeof(dc_data));

  memset(tlc_pwm2.data, 0x00, sizeof(tlc_pwm2.data));

  printf("Transfer size will be %u\n", sizeof(tlc_pwm2.data) * 8);

  struct spi_transaction_t trans_gs = {
    .length = sizeof(tlc_pwm2.data) * 8,
    .tx_buffer = tlc_pwm2.data
  };

  struct spi_transaction_t trans_dc = {
    .length = sizeof(tlc_dc2.data) * 8,
    .tx_buffer = tlc_dc2.data
  };

  HI(GPIO_BLANK);

//  dc_data[sizeof(dc_data) - 24 - 27 - 1] &= ~(1<<(8 - 4 - 1));
//  dc_data[sizeof(dc_data) - 24 - 1] &= ~(1<<(8 - 4 - 1));
//  dc_data[0] = 128;
//  spi_device_transmit(spi_dev_dc, &trans_dc);

  tlc_pwm2.tlcs.tlc1.pwm_0_r = 127;

  tlc_dc_init(&tlc_dc2.tlcs.tlc1);
  tlc_dc_init(&tlc_dc2.tlcs.tlc2);

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
//    spi_device_transmit(spi_dev_dc, &trans_dc);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
