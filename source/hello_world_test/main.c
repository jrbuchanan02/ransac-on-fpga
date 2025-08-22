/**
 * Hello, World test
 * This file tests that the built-in CPU can boot properly and that the 
 * processor can run C code to the point of outputting the string "Hello, World"
 */

#include <stdint.h>

typedef struct {
    uint32_t const volatile rx_fifo;
    uint32_t volatile tx_fifo;
    union {
        uint32_t const volatile status;
        struct {
            uint32_t const volatile rx_valid : 1;
            uint32_t const volatile rx_full : 1;
            uint32_t const volatile tx_empty : 1;
            uint32_t const volatile tx_full : 1;
            uint32_t const volatile intr_enabled : 1;
            uint32_t const volatile overrun : 1;
            uint32_t const volatile frame : 1;
            uint32_t const volatile parity : 1;
        };
    };
    union {
        uint32_t volatile control;
        struct {
            uint32_t volatile tx_reset : 1;
            uint32_t volatile rx_reset : 1;
            uint32_t volatile : 2;
            uint32_t volatile intr_enable : 1;
        };
    };
} uartlite_s;


void reset_uartlite(uartlite_s volatile *uart) {
    uart->tx_reset = 1;
    uart->rx_reset = 1;
    uart->intr_enable = 0;
}

void putc_uartlite(uartlite_s volatile *uart, char const c) {
    while (uart->tx_full);
    uart->tx_fifo = c;
}

char getc_uartlite(uartlite_s volatile *uart) {
    while (!uart->rx_valid);
    return (char)uart->rx_fifo;
}

void puts_uartlite(uartlite_s volatile *uart, char const *const str) {
    for (char const *pc = str; *pc; pc++) {
        putc_uartlite(uart, *pc);
    }
}

static uartlite_s volatile *const uartlite = (void *)0x40600000;

static char const hello_world[] = "Hello, World!\r\n";

int main() {
    reset_uartlite(uartlite);
    while(1) {
        puts_uartlite(uartlite, hello_world);
    }
}