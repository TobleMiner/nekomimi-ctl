#ifndef _IP_H_
#define _IP_H_

#include <stdbool.h>

struct ip_stack {
	bool initialized;
};

esp_err_t ip_stack_init();
bool ip_stack_initialized();

#endif
