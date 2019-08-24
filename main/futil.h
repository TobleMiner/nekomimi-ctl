#ifndef _FUTIL_H_
#define _FUTIL_H_

#include <stdbool.h>

#include "esp_err.h"

#define FUTIL_CHUNK_SIZE 256

typedef esp_err_t (*futil_write_cb)(void* ctx, char* buff, size_t len);

void futil_normalize_path(char* path);
esp_err_t futil_relpath(char* path, char* basepath);
char* futil_get_fext(char* path);
esp_err_t futil_get_bytes(void* dst, size_t len, char* path);
esp_err_t futil_read_file(void* ctx, char* path, futil_write_cb cb);
bool futil_is_path_relative(char* path);
char* futil_path_concat(char* path, char* basepath);

#endif
