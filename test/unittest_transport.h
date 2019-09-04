#ifndef UNITEST_TRANSPORT_H
#define UNITEST_TRANSPORT_H


void unittest_uart_begin();
void unittest_uart_putchar(char c);
void unittest_uart_flush();
void unittest_uart_end();

#endif // UNITEST_TRANSPORT_H
