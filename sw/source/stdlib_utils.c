#include "stdlib_utils.h"

#include "stdint.h"

// weak memcpy instance becuase GCC places calls to memcpy as part of some of 
// its optimizations.
void* __attribute__((weak)) memcpy(void* dst, void const *src, size_t count) {
    char *d = dst;
    char const *s = src;
    for (size_t i = 0; i < count; i++, d++, s++) {
        *d = *s;
    }
    return dst;
}