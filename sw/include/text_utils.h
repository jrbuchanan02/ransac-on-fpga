#ifndef TEXT_UTILS_H
#define TEXT_UTILS_H

#include "stdint.h"

struct string_from_u32_hex {
    char string[9];
};

struct string_from_u32_hex u32_to_hex_string(uint32_t);

#endif // ifndef TEXT_UTILS_H