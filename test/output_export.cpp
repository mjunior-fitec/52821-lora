#include "unittest_transport.h"
#include <output_export.h>



void output_start(unsigned int baudrate)
{
    unittest_uart_begin();
}

void output_char(int c)
{
    unittest_uart_putchar(c);
}

void output_flush(void)
{
    unittest_uart_flush();
}

void output_complete(void)
{
   unittest_uart_end();
}