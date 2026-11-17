#pragma once
#include <stddef.h>
extern "C" {
void* malloc(size_t);
void* realloc(void*, size_t);
void free(void*);
void abort(void);
double strtod(const char*, char**);
}
