
#include "Arduino.h"
#include <unity.h>


#define USB_DEBUG


#include <FlashStorage.h>
#include "enddevice.h"
#include "util.h"
#include "abnt.h"
#include "ansi_fitec.h"
#include "otica_ftdi.h"
#include "serial_nativa.h"
#include <cdcftdi.h>
#include <usbhub.h>
#include "wiring_private.h"

// void setUp(void)
// {
// }

// void tearDown(void)
// {
// }


//#define SerialDebug SerialUSB

#define TAM_LISTA_CMD_TESTE 16
#define MAX_T_UM_COMANDO 5000

#if 0
//Habilitação de uma nova UART, Serial3, nos pinos 0, PA22 (Tx) e 1, PA23 (Rx)
Uart Serial3(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM3_Handler()
{
    Serial3.IrqHandler();
}
#endif

//info salvas
feriado_t feriadosProgramados[TAM_LISTA_CMD_TESTE];
inicioPostosHorarios_t segmentosProgramados[4];

RTCZero rtc;

uint8_t i_test;
uint8_t listaCmdParaTestes[TAM_LISTA_CMD_TESTE]  = \
 { ID_CMD14, ID_CMD21, ID_CMD23, ID_CMD25, ID_CMD28, ID_CMD29, ID_CMD30, \
   ID_CMD32, ID_CMD35, ID_CMD64, ID_CMD80, ID_CMD87, ID_CMDE2, ID_CMDE2, \
   ID_CMDE2, ID_CMD37 };
uint8_t listaCmdEstendido[3] = {0x50, 0x90, 0x2B};
uint8_t i_Estendido = 0;

void alwaysTrue(void)
{
    TEST_ASSERT(true);
}

void  initTesteABNT(void)
{
    rtc.begin();
    rtc.setDate(04, 04, 19);
    rtc.setTime(10, 30, 10);

    abntProgFeriados.feriados[0].dia = 0x05;
    abntProgFeriados.feriados[0].mes = 0x03;
    abntProgFeriados.feriados[0].ano = 0x19;
    abntProgFeriados.feriados[1].dia = 0x19;
    abntProgFeriados.feriados[1].mes = 0x04;
    abntProgFeriados.feriados[1].ano = 0x19;
    abntProgFeriados.feriados[2].dia = 0x20;
    abntProgFeriados.feriados[2].mes = 0x06;
    abntProgFeriados.feriados[2].ano = 0x19;

    for (int fer = 3; fer < 15; fer++)
    {
        abntProgFeriados.feriados[fer].dia = 1;
        abntProgFeriados.feriados[fer].mes = 1;
        abntProgFeriados.feriados[fer].ano = 2;
    }

    abntProgSegmentosHorarios.progPostoHor[0].horaInicio1     = 0x08;
    abntProgSegmentosHorarios.progPostoHor[0].minutoInicio1   = 0x00;
    abntProgSegmentosHorarios.progPostoHor[0].horaInicio2     = 0x14;
    abntProgSegmentosHorarios.progPostoHor[0].minutoInicio2   = 0x30;
    abntProgSegmentosHorarios.progPostoHor[0].horaInicio3     = 0x18;
    abntProgSegmentosHorarios.progPostoHor[0].minutoInicio3   = 0x00;
    abntProgSegmentosHorarios.progPostoHor[0].horaInicio4     = 0x00;
    abntProgSegmentosHorarios.progPostoHor[0].minutoInicio4   = 0x00;
    abntProgSegmentosHorarios.progPostoHor[1].horaInicio1     = 0x10;
    abntProgSegmentosHorarios.progPostoHor[1].minutoInicio1   = 0x30;
    abntProgSegmentosHorarios.progPostoHor[1].horaInicio2     = 0x16;
    abntProgSegmentosHorarios.progPostoHor[1].minutoInicio2   = 0x30;
    abntProgSegmentosHorarios.progPostoHor[1].horaInicio3     = 0x20;
    abntProgSegmentosHorarios.progPostoHor[1].minutoInicio3   = 0x30;
    abntProgSegmentosHorarios.progPostoHor[1].horaInicio4     = 0x00;
    abntProgSegmentosHorarios.progPostoHor[1].minutoInicio4   = 0x00;
    abntProgSegmentosHorarios.numSegmentos = 2;

    abntAlteraHorVer.horarioVerao.ativar = 1;
    abntAlteraHorVer.horarioVerao.diaFimHorInverno = 0x20;
    abntAlteraHorVer.horarioVerao.mesFimHorInverno = 0x10;
    abntAlteraHorVer.horarioVerao.diaFimHorVerao = 0x17;
    abntAlteraHorVer.horarioVerao.mesFimHorVerao = 0x02;

    abntInit();
} //initTesteABNT(

void adicionaCmdABNTparaTeste(uint8_t comando)
{
    abntInit();
    insereCmdABNT(comando);
    if (comando == ID_CMDE2)
        cmdEstendido = listaCmdEstendido[i_Estendido++];
    SerialDebug.println("Prox cmd: " + String(comando, HEX));
    SerialDebug.println();
}

void proxComando(void)
{
    uint32_t t_comandoAtual = millis();

    while ((millis() - t_comandoAtual) < MAX_T_UM_COMANDO)
    {
        maqEstAbnt();
        if (respABNTRecebida)
        {
            SerialDebug.println("Comando enviado: ");
            for (uint8_t i = 0; i < 66; i++)
            {
                SerialDebug.print(String(pBuffABNTsend[i], HEX) + "\t");

            }
            SerialDebug.println("");
            SerialDebug.println("Resposta cmd: " + String(((abnt_resp_generic_t *)pBuffABNTrecv)->id, HEX));
            //SerialDebug.println("80 primeiros bytes abaixo:");
            for (uint16_t ii = 0; ii < 258; ++ii)
            {
                SerialDebug.print(String(pBuffABNTrecv[ii], HEX) + "\t");
            }
            SerialDebug.println();
            SerialDebug.flush();
            delay(200);
            piscaLed(2,200,100);
            TEST_PASS();
        }
        delay(50);
    }
    TEST_FAIL();
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(SERIAL_RTS, OUTPUT);
    digitalWrite(SERIAL_RTS, LOW); //Low TTL = +5,5V RS232
    digitalWrite(LED_BUILTIN, LOW);
    pinPeripheral(1, PIO_SERCOM);
    pinPeripheral(0, PIO_SERCOM);

    Serial1.begin(ABNT_BAUD);
    Serial1.setTimeout(250);
    SerialDebug.begin(9600);

    if (UsbH.Init())
    {
        delay(500);
        piscaLed(3, 350, 200);
    }

    delay (2000);
    SerialDebug.begin(9600);
    //while (!SerialUSB) ;
    SerialDebug.println("Init...");
    initTesteABNT();
    SerialDebug.println("Unit test - ABNT");
    abntOcorrencia = 0x00; //Preencher para limpar ocorrência

    UNITY_BEGIN();
    RUN_TEST(alwaysTrue);
    for (i_test = 0; i_test < TAM_LISTA_CMD_TESTE; ++i_test)
    {
        SerialDebug.println();
        SerialDebug.println("test: " + String(i_test));
        adicionaCmdABNTparaTeste(listaCmdParaTestes[i_test]);
        RUN_TEST(proxComando);
    }
    UNITY_END();
    SerialDebug.end();
}

void loop()
{
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
}
