#CFLAGS += -fpack-struct

COMPONENT_SRCDIRS = . BME680_driver

ifeq ($(CONFIG_NEKOMIMI_BME680_ALGO_PROPRIETARY),y)
  COMPONENT_PRIV_INCLUDEDIRS += bme680_nonfree bme680_nonfree/BSEC/algo/lite_version/inc/
  COMPONENT_ADD_LINKER_DEPS += bme680_nonfree/BSEC/algo/lite_version/bin/esp32/libalgobsec.a
  COMPONENT_SRCDIRS += bme680_nonfree
endif

ifeq ($(CONFIG_NEKOMIMI_BME680_ALGO_FREE),y)
  COMPONENT_PRIV_INCLUDEDIRS += bme680_free
  COMPONENT_SRCDIRS += bme680_free
endif
