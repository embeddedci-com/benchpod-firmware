/* string.h for the freestanding WASM build of the sensor definition engine (libc/shim.c). */
#pragma once
#include <stddef.h>

void  *memcpy(void *restrict d, const void *restrict s, size_t n);
void  *memmove(void *d, const void *s, size_t n);
void  *memset(void *d, int c, size_t n);
int    memcmp(const void *a, const void *b, size_t n);
void  *memchr(const void *s, int c, size_t n);
size_t strlen(const char *s);
char  *strchr(const char *s, int c);
int    strcmp(const char *a, const char *b);
int    strncmp(const char *a, const char *b, size_t n);
