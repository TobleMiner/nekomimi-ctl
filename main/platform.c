#include "platform.h"

#include <esp_log.h>

#include "i2c_bus.h"

struct sensor_manager sensors;
struct tlc_chain ears;
struct i2c_bus i2c_sensors;

static esp_err_t platform_bus_init() {
  esp_err_t err = i2c_bus_init(&i2c_sensors, SENSOR_I2C_BUS, SENSOR_I2C_BUS_GPIO_SDA, SENSOR_I2C_BUS_GPIO_SCL, SENSOR_I2C_BUS_FREQ);
  if(err) {
    ESP_LOGE(PLATFORM_TAG, "Failed to initialize sensor I2C bus");
    return err;
  }
  i2c_detect(&i2c_sensors);
  return ESP_OK;
}

static esp_err_t platform_sensor_init() {
#ifdef NEKOMIMI_SENSOR_USE_LIS3MDL
  struct lis3mdl_sensor_args lis_args = {
    .gpio_drdy = SENSOR_LIS3MDL_GPIO_DRDY,
    .gpio_int = SENSOR_LIS3MDL_GPIO_INT,
  };
#endif
  esp_err_t err = sensors_init(&sensors);
  if(err) {
    ESP_LOGE(PLATFORM_TAG, "Failed to initialize sensor layer");
    return err;
  }

#ifdef NEKOMIMI_SENSOR_USE_BME680
  err = sensors_add_sensor(&sensors, &bme680_sensor_def, &i2c_sensors, SENSOR_BME680_ADDR, NULL);
  if(err) {
    ESP_LOGE(PLATFORM_TAG, "Failed to initialize Bosch BME680");
    return err;
  }
#endif

#ifdef NEKOMIMI_SENSOR_USE_BH1750
  err = sensors_add_sensor(&sensors, &bh1750_sensor_def, &i2c_sensors, SENSOR_BH1750_ADDR, NULL);
  if(err) {
    ESP_LOGE(PLATFORM_TAG, "Failed to initialize BH1750");
    return err;
  }
#endif

#ifdef NEKOMIMI_SENSOR_USE_LIS3MDL
  err = sensors_add_sensor(&sensors, &lis3mdl_sensor_def, &i2c_sensors, SENSOR_LIS3MDL_ADDR, &lis_args);
  if(err) {
    ESP_LOGE(PLATFORM_TAG, "Failed to initialize LIS3MDL");
    return err;
  }
#endif

  return ESP_OK;
}

static esp_err_t platform_ear_init() {
  esp_err_t err;
  if((err = ear_init(&ears, EAR_CNT, EAR_GPIO_PWM_CLK, EAR_GPIO_LATCH, EAR_SPI))) {
    return err;
  }

  if((err = tlc_set_led_current_all(&ears, TLC_COLOR_CHANNEL_ALL, EAR_DEFAULT_CURRENT))) {
    return err;
  }

  return ESP_OK;
}

esp_err_t platform_init() {
  esp_err_t err = platform_bus_init();
  if(err) {
    ESP_LOGE(PLATFORM_TAG, "Failed to initialize busses");
    return err;
  }
  err = platform_sensor_init();
  if(err) {
    ESP_LOGE(PLATFORM_TAG, "Failed to initialize sensors");
    return err;
  }
  err = platform_ear_init();
  if(err) {
    ESP_LOGE(PLATFORM_TAG, "Failed to initialize ears");
    return err;
  }
  return ESP_OK;
}
