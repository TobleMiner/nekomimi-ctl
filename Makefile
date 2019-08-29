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

MKFATFS_SUBMODULE := mk_esp32fat
MKFATFS := $(MKFATFS_SUBMODULE)/mkfatfs
FATFS_ROOT := fat_root
BUILD_DIR := build
FATFS_IMG := $(BUILD_DIR)/fatfs.img
FATFS_OFFSET := 0x130000
PARTITION_TABLE := $(BUILD_DIR)/partitions.bin

$(MKFATFS): $(MKFATFS_SUBMODULE)
	LDFLAGS='' AR='' CFLAGS='' CXXFLAGS='' CPPFLAGS='' CC='' CXX='' CPP='' make -C "$(MKFATFS_SUBMODULE)"

build_fatfs: $(MKFATFS)
	"$(MKFATFS)" -c "$(FATFS_ROOT)" -t "$(PARTITION_TABLE)" "$(FATFS_IMG)"

flash_fatfs: build_fatfs
	python $(IDF_PATH)/components/esptool_py/esptool/esptool.py --chip esp32 --port $(CONFIG_ESPTOOLPY_PORT) --baud $(CONFIG_ESPTOOLPY_BAUD) \
	--before $(CONFIG_ESPTOOLPY_BEFORE) --after $(CONFIG_ESPTOOLPY_AFTER) write_flash -z --flash_mode $(CONFIG_ESPTOOLPY_FLASHMODE) \
	--flash_freq $(CONFIG_ESPTOOLPY_FLASHFREQ) $(ESPTOOL_ARGS) $(FATFS_OFFSET) $(FATFS_IMG)

.PHONY: flash_fatfs build_fatfs
