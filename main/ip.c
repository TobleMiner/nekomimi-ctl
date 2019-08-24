#include <tcpip_adapter.h>

#include "ip.h"

static struct ip_stack ip_stack = { 0 };

esp_err_t ip_stack_init() {
	esp_err_t err;
	if(!(err = tcpip_adapter_init())) {
		ip_stack.initialized = true;
	}
	return err;
}

bool ip_stack_initialized() {
	return ip_stack.initialized;
}
