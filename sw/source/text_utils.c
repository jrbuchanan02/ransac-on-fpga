#include "text_utils.h"

#include "stdint.h"

struct string_from_u32_hex u32_to_hex_string(uint32_t u) {
    struct string_from_u32_hex str;
    for (int i = 0; i < sizeof str; i++) {
        str.string[i] = '\0';
    }
    for (int i = 7; i >= 0; i--) {
        uint32_t nybble = u & 0xF;
        u >>= 4;
        if (nybble < 10) {
            str.string[i] = '0' + nybble;
        } else {
            str.string[i] = 'A' + nybble - 10;
        }
    }
    return str;
}