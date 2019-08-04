#CFLAGS += -fpack-struct

COMPONENT_SRCDIRS = . BME680_driver

BOSCH_BSEC := "https://ae-bst.resource.bosch.com/media/_tech/media/bsec/BSEC_1.4.7.4_Generic_Release.zip"
BME_NONFREE := bme680_nonfree
BSEC_PATH := $(BME_NONFREE)/BSEC.zip
BSEC_DIR := $(BME_NONFREE)/BSEC
BUILD_RELPATH := ../build/$(COMPONENT_NAME)/

ifeq ($(CONFIG_NEKOMIMI_BME680_ALGO_PROPRIETARY),y)
	bsec=$(shell mkdir -p $(BME_NONFREE); unzip -t "$(BSEC_PATH)" 2> /dev/null || wget "$(BOSCH_BSEC)" -O "$(BSEC_PATH)"; [ -e $(BSEC_DIR) ] || unzip "$(BSEC_PATH)" -d "$(BSEC_DIR)")
	bsec_algo=$(shell find "$(BSEC_DIR)" -name lite_version -type d \# $(bsec))
	bsec_esp32=$(shell find "$(bsec_algo)" -name esp32 -type d)
	bsec_lib=$(shell find "$(bsec_esp32)" -name libalgobsec.a -type f)
	COMPONENT_PRIV_INCLUDEDIRS += bme680_nonfree $(BUILD_RELPATH)/$(bsec_algo)/inc/
	COMPONENT_ADD_LINKER_DEPS += $(BUILD_RELPATH)/$(bsec_lib)
	COMPONENT_ADD_LDFLAGS += $(abspath $(bsec_lib))
	COMPONENT_SRCDIRS += bme680_nonfree
	COMPONENT_OWNCLEANTARGET := 1
endif

ifeq ($(CONFIG_NEKOMIMI_BME680_ALGO_FREE),y)
	COMPONENT_PRIV_INCLUDEDIRS += bme680_free
	COMPONENT_SRCDIRS += bme680_free
endif


clean: clean_bsec

clean_bsec:
	rm -f $(BSEC_PATH)
	rm -rf $(BSEC_DIR)

.PHONY: clean_bsec
