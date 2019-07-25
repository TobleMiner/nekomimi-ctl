#pragma once

#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif
#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

#define CLAMP(x, h, l) (max(min((x), (l)), (h)))

void hexdump(uint8_t* data, size_t len);
