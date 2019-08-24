#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "esp_system.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"

#include "wifi.h"
#include "random.h"

extern struct random random;

// There can only be one wifi instance, use statically allocated structs
static struct wifi wifi = { 0 };

static esp_err_t wifi_event_handler(void* priv, system_event_t* event) {
	return ESP_OK;
}

bool wifi_enabled() {
	return !!wifi.enabled;
}

esp_err_t wifi_init(void) {
	esp_err_t err;
	
	wifi.event_group = xEventGroupCreate();
	if(!wifi.event_group) {
		err = ESP_ERR_NO_MEM;
		goto fail;
	}

	if((err = esp_event_loop_init(wifi_event_handler, NULL))) {
		goto fail_event_group;
	}

	return ESP_OK;

fail_event_group:
	vEventGroupDelete(wifi.event_group);
fail:
	return err;
}

esp_err_t wifi_ap_start(char* ssid, char* passwd) {
	esp_err_t err;
	size_t ssid_len = strlen(ssid);
	size_t passwd_len = strlen(passwd);
	wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();


	wifi_config_t cfg = {
		.ap = {
			.ssid_len = ssid_len,
			.max_connection = WIFI_MAX_STATIONS,
			.authmode = WIFI_AUTH_WPA_WPA2_PSK,
		},
	};

	if(ssid_len > sizeof(cfg.ap.ssid)) {
		err = ESP_ERR_INVALID_ARG;
		goto fail;
	}

	if(passwd_len > sizeof(cfg.ap.password) - 1) {
		err = ESP_ERR_INVALID_ARG;
		goto fail;
	}

	memcpy((char*)cfg.ap.ssid, ssid, ssid_len);
	strcpy((char*)cfg.ap.password, passwd);

	random_disable();

	if((err = esp_wifi_init(&init_cfg))) {
		goto fail;
	}

	if((err = esp_wifi_set_mode(WIFI_MODE_AP))) {
		goto fail;
	}

	if((err = esp_wifi_set_config(ESP_IF_WIFI_AP, &cfg))) {
		goto fail;
	}

	if((err = esp_wifi_start())) {
		goto fail;
	}

	wifi.enabled = true;
	return ESP_OK;

fail:
	return err;
}

esp_err_t wifi_ap_stop() {
	esp_err_t err = esp_wifi_stop();
	wifi.enabled = false;
	return err;
}

void wifi_generate_password(char* buf, size_t len) {
	size_t i, num_passwd_chars = strlen(WIFI_PASSWD_CHARS);

	for(i = 0; i < len; i++) {
		buf[i] = WIFI_PASSWD_CHARS[random_uint32() % num_passwd_chars];
	}
}

char* wifi_alloc_password(size_t len) {
	char* passwd = calloc(1, len + 1);
	if(!passwd) {
		return NULL;
	}

	wifi_generate_password(passwd, len);

	return passwd;
}
