
#include "Arduino.h"
#include <unity.h>

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

#define T_MAX_WAIT_DOWNLINK (30000)

//Habilitação de uma nova UART, Serial3, nos pinos 0, PA22 (Tx) e 1, PA23 (Rx)
Uart Serial3(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM3_Handler()
{
    Serial3.IrqHandler();
}

bool recebeuAcertaRTC;
RTCZero rtc;

//#define SerialDebug SerialUSB

void alwaysTrue(void)
{
    TEST_ASSERT(true);
}

void recTabANSI10(void)
{
    if (modem.available())
    {
        char rcv[64];
        uint8_t i = 0;
        while (modem.available())
        {
            rcv[i++] = (char)modem.read();
        }
        SerialDebug.print("DL Recvd: ");
        for (uint8_t j = 0; j < i; j++)
        {
            SerialDebug.print(rcv[j] >> 4, HEX);
            SerialDebug.print(rcv[j] & 0xF, HEX);
            SerialDebug.print(" ");
        }
        SerialDebug.println();

        if ((rcv[0] == TAB_ANSI10) && (rcv[2] == SIZE_TABELA10_SO_RELOGIO))
        {
            struct tm *datahora_tm;
            time_t *horaCerta = (time_t *)(&(rcv[4]));

            rtc.begin();
            datahora_tm = localtime(horaCerta);
            //Ajusta o RTC com a programacao recebida
            rtc.setDate(datahora_tm->tm_mday, (datahora_tm->tm_mon + 1), (datahora_tm->tm_year - 100));
            rtc.setTime(datahora_tm->tm_hour, datahora_tm->tm_min, datahora_tm->tm_sec);
            recebeuAcertaRTC = true;
            SerialDebug.println("Hora lida do RTC apos programacao: ");
            SerialDebug.println("Dia: " + String(rtc.getDay()) + "/" + String(rtc.getMonth()) + "/" + String(rtc.getYear()));
            SerialDebug.println("Hora: "+ String(rtc.getHours()) + ":"+ String(rtc.getMinutes()) + ":" + String(rtc.getSeconds()));
        }
    }
}

void testaRecTabANSI10(void)
{
    recebeuAcertaRTC = false;
    if (!modem.begin(AU915))
    {
        SerialDebug.println("Erro ao iniciar radio LoRa");
        TEST_FAIL();
    }
    delay(500);
    if (!modem.publicNetwork(false))
    {
        SerialDebug.println("Erro ao configurar rede privada");
        TEST_FAIL();
    }
    if (!modem.configureClass(CLASS_C))
    {
        SerialDebug.println("Erro ao configurar para classe C");
        TEST_FAIL();
    }
    int connected = modem.joinABP(ABP_DEVADDR, ABP_NWKSKEY, ABP_APPSKEY);

    if (connected)
    {
        int err;
        uint8_t testLoRa[4] = {0xDE, 0xAD, 0xBE, 0xEF};
        modem.beginPacket();
        modem.write(testLoRa, 4);
        err = modem.endPacket(false);
        //SerialDebug.println("Send ret: " + String(err));
        if (err < 0)
        {
            SerialDebug.println("Erro ao enviar pacote LoRa!");
            TEST_FAIL();
        }
        delay(2000);
        SerialDebug.println("Aguardando downlink (30s) ...");
        uint32_t t_atual;

        t_atual = millis();
        while ((millis() - t_atual) < T_MAX_WAIT_DOWNLINK)
        {
            recTabANSI10();
        }
        SerialDebug.println("Tempo esgotado!");
    }
    else
        SerialDebug.println("Erro ao conectar LoRa");

    TEST_ASSERT(recebeuAcertaRTC);
}


void setup()
{
    pinPeripheral(1, PIO_SERCOM);
    pinPeripheral(0, PIO_SERCOM);
    delay (2000);
    SerialDebug.begin(9600);
    piscaLed(5, 200, 200);
    while (!SerialUSB) ;
    piscaLed(3, 500, 200);

    UNITY_BEGIN();
    RUN_TEST(alwaysTrue);
    RUN_TEST(testaRecTabANSI10);
    UNITY_END();
}

void loop()
{
    digitalWrite(LED_BUILTIN, HIGH);
    delay(800);
    digitalWrite(LED_BUILTIN, LOW);
    delay(800);
}
