#include "uart.h"

#include <stdbool.h>
#include <stdint.h>

uint32_t volatile *const uartlite_ctrl_reg = (uint32_t volatile *)0x4060000C;
uint32_t volatile const *const uartlite_stat_reg = (uint32_t volatile const *)0x40600008;
uint32_t volatile *const uartlite_txdata = (uint32_t volatile *)0x40600004;
uint32_t volatile const *const uartlite_rxdata = (uint32_t volatile const *)0x40600000;

bool fifo_is_filled(void) {
    uint32_t fifo_state = *uartlite_stat_reg;
    if (fifo_state & 8) {
        return true;
    } else {
        return false;
    }
}

bool rdata_waiting(void) {
    uint32_t fifo_state = *uartlite_stat_reg;
    if (fifo_state & 1) {
        return true;
    } else {
        return false;
    }
}

void uart_init(void) {
    *uartlite_ctrl_reg = 3;
}

void uart_putc(char c) {
    while (fifo_is_filled());
    *uartlite_txdata = c;
}

char uart_getc(void) {
    while (!rdata_waiting());
    return (char)*uartlite_rxdata;
}

void uart_puts(char const *s) {
    for(; *s; s++) {
        uart_putc(*s);
    }
}