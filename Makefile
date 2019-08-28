#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := nekomimi

include $(IDF_PATH)/make/project.mk

flash: flash_fatfs

undefine ESPTOOL_ARGS
ifeq ($(CONFIG_ESPTOOLPY_FLASHSIZE_DETECT),y)
	ESPTOOL_ARGS += --flash_size detect
endif

flash_fatfs:
	python $(IDF_PATH)/components/esptool_py/esptool/esptool.py --chip esp32 --port $(CONFIG_ESPTOOLPY_PORT) --baud $(CONFIG_ESPTOOLPY_BAUD) \
	--before $(CONFIG_ESPTOOLPY_BEFORE) --after $(CONFIG_ESPTOOLPY_AFTER) write_flash -z --flash_mode $(CONFIG_ESPTOOLPY_FLASHMODE) \
	--flash_freq $(CONFIG_ESPTOOLPY_FLASHFREQ) $(ESPTOOL_ARGS) 0x130000 build/main/fatfs.img

.PHONY: flash_fatfs
