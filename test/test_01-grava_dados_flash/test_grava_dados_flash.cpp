
#include "Arduino.h"
#include <unity.h>

#include <FlashStorage.h>
#include "enddevice.h"
#include "util.h"
#include "abnt.h"
#include "ansi_fitec.h"
#include "otica_ftdi.h"
#include "serial_nativa.h"
#include "wiring_private.h"

// void setUp(void)
// {
// }

// void tearDown(void)
// {
// }

typedef struct testData
{
    uint8_t     Id;
    uint32_t    testFloatQ14: 24;
    float       testFloat;
    uint16_t    testUint16;
    uint8_t     StrTest[6];
} __attribute__ ((packed)) testData_t;

testData_t valorTeste, valorLido;
FlashStorage(valorGravado, testData_t);

//Habilitação de uma nova UART, Serial3, nos pinos 0, PA22 (Tx) e 1, PA23 (Rx)
// Uart Serial3(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);
// void SERCOM3_Handler()
// {
//     Serial3.IrqHandler();
// }

//#define SerialDebug SerialUSB

void alwaysTrue(void)
{
    TEST_ASSERT(true);
}

void imprimeTeste(testData_t valor)
{
    SerialDebug.println("Id: 0x" + String(valor.Id, HEX));
    SerialDebug.println("Float: " + String(valor.testFloat));
    SerialDebug.println("Float Q.14: 0x" + String(valor.testFloatQ14, HEX) + \
                        ", sendo = " + String (((float)valor.testFloatQ14) / (1<<14)));
    SerialDebug.println("Valor 16 bits: 0x" + String(valor.testUint16, HEX));
    SerialDebug.println("Texto: " + String((char *)valor.StrTest) + "\r\n");
}

void testGravacaoFlash(void)
{
    SerialDebug.println("\r\nValor lido, antes da gravacao:");
    valorTeste = valorGravado.read();
    imprimeTeste(valorTeste);
    char valorStrTeste[6] = "FITec";

    SerialDebug.println("\r\nValor a ser gravado:");
    valorTeste.Id = 0xA5;
    valorTeste.testFloat = 123.45;
    valorTeste.testFloatQ14 = valorTeste.testFloat * (1<<14);
    valorTeste.testUint16 = 0xDEAD;
    memcpy (valorTeste.StrTest, valorStrTeste, 6);
    imprimeTeste(valorTeste);

    valorGravado.write(valorTeste);
    SerialDebug.println("\r\nValor lido, apos gravacao:");
    valorLido = valorGravado.read();
    imprimeTeste(valorTeste);
    TEST_ASSERT(valorTeste.Id == valorLido.Id);
    TEST_ASSERT(valorTeste.testFloat == valorLido.testFloat);
    TEST_ASSERT(valorTeste.testFloatQ14 == valorLido.testFloatQ14);
    TEST_ASSERT(valorTeste.testUint16 == valorLido.testUint16);
}

void setup()
{
    // pinPeripheral(1, PIO_SERCOM);
    // pinPeripheral(0, PIO_SERCOM);
    delay (2000);
    // SerialDebug.begin(9600);
    //while (!SerialUSB) ;

    UNITY_BEGIN();
    RUN_TEST(alwaysTrue);
    RUN_TEST(testGravacaoFlash);
    UNITY_END();
}

void loop()
{
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
}
