#ifndef _WIFI_H_
#define _WIFI_H_

#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#define WIFI_PASSWD_LEN 12

#define WIFI_MAX_STATIONS 4

#define WIFI_PASSWD_CHARS "AFHL073E2CWIUT4"

struct wifi {
	bool enabled;
	EventGroupHandle_t event_group;
};

bool wifi_enabled();
esp_err_t wifi_init(void);
esp_err_t wifi_ap_start(char* ssid, char* password);
esp_err_t wifi_ap_stop();

void wifi_generate_password(char* buf, size_t len);
char* wifi_alloc_password(size_t len);

#endif
