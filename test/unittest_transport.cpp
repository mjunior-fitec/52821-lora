
#include <Arduino.h>
#include "unittest_transport.h"
#include "wiring_private.h"
#include "uart.h"

Uart Serial3(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);

void SERCOM3_Handler()
{
    Serial3.IrqHandler();
}

void unittest_uart_begin()
{
    pinPeripheral(1, PIO_SERCOM);
    pinPeripheral(0, PIO_SERCOM);

    ///
    // USARTx_TX_GPIO_CLK_ENABLE();
    // USARTx_RX_GPIO_CLK_ENABLE();
    // USARTx_CLK_ENABLE();
    ///

    Serial3.begin(9600);
}

void unittest_uart_putchar(char c)
{
    Serial3.write(c);
}

void unittest_uart_flush()
{
    Serial3.flush();
}

void unittest_uart_end() {
  Serial3.end();
}
