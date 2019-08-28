#include <tcpip_adapter.h>

#include "ip.h"

static struct ip_stack ip_stack = { 0 };

esp_err_t ip_stack_init() {
  esp_err_t err = 0;
#ifdef ESP_IDF_SAFE_TCPIP_INIT
  if(!(err = tcpip_adapter_init())) {
    ip_stack.initialized = true;
  }
#else
  tcpip_adapter_init();
  ip_stack.initialized = true;
#endif
  return err;
}

bool ip_stack_initialized() {
  return ip_stack.initialized;
}
