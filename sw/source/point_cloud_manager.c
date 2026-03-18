
#include "point_cloud_manager.h"

#ifndef POINT_CLOUD_LENGTH
#define POINT_CLOUD_LENGTH 2048
#endif // ifndef 

#include "stdbool.h"

#include "uart.h"

// buffer length
// 8 chars per point (hex str) + 1 char (\n) per 3 points + 1 char per point (,)
// = (27 + 1) * POINT_CLOUD_LENGTH + 1 chars.

// this data should automatically place itself in ddr3mem

char point_cloud_text_buffer[28 * POINT_CLOUD_LENGTH + 1];

uint32_t point_cloud[3 * POINT_CLOUD_LENGTH + 1];

int hex_digit_to_nybble(char hex) {
    if (hex >= '0' && hex <= '9') {
        return hex - '0';
    } else if (hex >= 'a' && hex <= 'f') {
        return hex - 'a' + 10;
    } else if (hex >= 'A' && hex <= 'F') {
        return hex - 'A' + 10;
    } else {
        return -1;
    }
}

bool should_skip_csv_char(char c) {
    switch (c) {
    case ',': return true;
    case '\r': return true;
    case '\n': return true;
    default: return false;
    }
}


void copy_scalar(uint32_t **pscalar_iterator, char const **pstring_iterator) {
    // copy over each byte. Advance past the comma and then the \r, \n if
    // necesary.
    **pscalar_iterator = 0;
    for (int i = 0; i < 8; i++, (*pstring_iterator)++) {
        **pscalar_iterator <<= 4;
        int nybble = hex_digit_to_nybble(**pstring_iterator);
        if (nybble == -1) {
            **pscalar_iterator = 0;
            uart_puts("Found Invalid Scalar: has non-hex char ");
            uart_putc(**pstring_iterator);
            uart_putc('\n');
            break;
        }
        **pscalar_iterator |= (nybble & 0xF);
    }
    (*pscalar_iterator)++;
    while (should_skip_csv_char(**pstring_iterator)) {
        (*pstring_iterator)++;
    }
}

void point_cloud_manager_init(void) {
    for (char *p = point_cloud_text_buffer; p < point_cloud_text_buffer + sizeof point_cloud_text_buffer; p++) {
        *p = '\0';
    }
    for (uint32_t *p = point_cloud; p < point_cloud + POINT_CLOUD_LENGTH; p++) {
        *p = 0;
    }
}

void read_in_from_uart(void) {
    for (uintptr_t i = 0; i < sizeof point_cloud_text_buffer; i++) {
        point_cloud_text_buffer[i] = uart_getc();
    }
    uint32_t *scalar_iterator = point_cloud;
    char const *string_iterator = point_cloud_text_buffer;
    for (uintptr_t i = 0; i < POINT_CLOUD_LENGTH; i++) {
        copy_scalar(&scalar_iterator, &string_iterator);
    }
}

uint32_t const *get_cloud_base_address(void) {
    return point_cloud;
}

uintptr_t get_cloud_length(void) {
    return POINT_CLOUD_LENGTH;
}