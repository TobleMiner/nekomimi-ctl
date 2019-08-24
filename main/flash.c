#include <string.h>

#include "nvs_flash.h"

#include "flash.h"

struct flash flash = { 0 };

esp_vfs_fat_mount_config_t flash_fat_mount_cfg = {
	.format_if_mount_failed = false,
	.max_files = 16,
	.allocation_unit_size = CONFIG_WL_SECTOR_SIZE,
};

esp_err_t flash_nvs_init() {
	esp_err_t err = nvs_flash_init();
	if(err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		if((err = nvs_flash_erase())) {
			return err;
		}
		err = nvs_flash_init();
	}
	if(!err) {
		flash.nvs_initialized = true;
	}
	return err;
}

esp_err_t flash_fatfs_alloc(struct fatfs** retval, const char* label, const char* base_path) {
	esp_err_t err;
	struct fatfs* fs = calloc(1, sizeof(struct fatfs));
	if(!fs) {
		err = ESP_ERR_NO_MEM;
		goto fail;
	}

	fs->wl_handle = WL_INVALID_HANDLE;

	fs->base_path = strdup(base_path);
	if(!fs->base_path) {
		err = ESP_ERR_NO_MEM;
		goto fail_fs_alloc;
	}

	if((err = esp_vfs_fat_spiflash_mount(base_path, label, &flash_fat_mount_cfg, &fs->wl_handle))) {
		goto fail_path_alloc;
	}

	*retval = fs;
	return ESP_OK;

fail_path_alloc:
	free(fs->base_path);
fail_fs_alloc:
	free(fs);
fail:
	return err;
}
