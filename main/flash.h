#ifndef _FLASH_H_
#define _FLASH_H_

#include <stdbool.h>

#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_system.h"

struct flash {
	bool nvs_initialized;
};

struct fatfs {
	char* base_path;
	wl_handle_t wl_handle;
};

esp_err_t flash_nvs_init();
esp_err_t flash_fatfs_alloc(struct fatfs** retval, const char* label, const char* base_path);

#endif
