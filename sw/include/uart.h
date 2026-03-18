#ifndef UART_H
#define UART_H

void uart_init(void);

inline void uart_reset(void) {
    uart_init();
}

void uart_putc(char);
char uart_getc(void);

void uart_puts(char const *s);

#endif // #ifndef UART_H