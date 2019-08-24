#ifndef _RANDOM_H_
#define _RANDOM_H_

#include <stdbool.h>

#define RANDOM_HEADERS_AVAIL 1

struct random {
	bool initialized;
};

bool random_initialized();
void random_enable();
void random_disable();

uint32_t random_uint32();

#endif
