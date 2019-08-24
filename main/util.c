#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include "util.h"

void strntr(char* str, size_t len, char a, char b) {
	while(len-- > 0) {
		if(*str == a) {
			*str = b;
		}
		str++;
	}
}

void hexdump(uint8_t* data, size_t len) {
  while(len-- > 0) {
    printf("%02X ", *data++);
  }
  printf("\n");
}
