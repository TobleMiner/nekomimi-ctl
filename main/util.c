#include <string.h>
#include <stdio.h>
#include <stdint.h>

void hexdump(uint8_t* data, size_t len) {
  while(len-- > 0) {
    printf("%02X ", *data++);
  }
  printf("\n");
}
