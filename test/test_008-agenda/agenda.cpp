
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

itemAgenda_t * listaAgenda, *itemAtual;

void alwaysTrue(void)
{
    TEST_ASSERT(true);
}


void programaAlarmeAgenda(void)
{
    uint8_t horaAlarme, minAlarme, segAlarme;

    if (itemAtual->next)
        itemAtual = itemAtual->next;
    else
        itemAtual = listaAgenda; //volta ao inicio
    segundosParaHHMMSS(itemAtual->tAgenda, &horaAlarme, &minAlarme, &segAlarme);
    rtc.setAlarmTime(horaAlarme, minAlarme, segAlarme);
    rtc.attachInterrupt(itemAtual->fAgenda);
}

void iniciaAlarmes(void)
{
    uint32_t horaAtual = 0;
    uint8_t horaAlarme, minAlarme, segAlarme;
    horaAtual = hhmmssParaSegundos(rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
    itemAtual = listaAgenda;
    while (itemAtual->tAgenda < (horaAtual + 2))
    {
        if (itemAtual->next)
            itemAtual = itemAtual->next;
        else
        {
            //Se chegou no fim da lista, usa o primeiro valor do dia
            itemAtual = listaAgenda;
            break;
        }
    }
    segundosParaHHMMSS(itemAtual->tAgenda, &horaAlarme, &minAlarme, &segAlarme);
    rtc.setAlarmTime(horaAlarme, minAlarme, segAlarme);
    rtc.enableAlarm(rtc.MATCH_HHMMSS);
    rtc.attachInterrupt(itemAtual->fAgenda);
}

void transmiteLoRaFrequente()
{
    SerialDebug.println("Transmite LoRa frequente!\r\n Tabelas ANSI: 01 e 02");
    programaAlarmeAgenda();
}
void transmiteLoRaEventual()
{
    SerialDebug.println("Transmite LoRa eventual!\r\n Tabelas ANSI: 03 e 04");
    programaAlarmeAgenda();
}
void preparaABNTFrequente()
{
    SerialDebug.println("Envia cmds ABNT:\r\n 23, 21+23");
    programaAlarmeAgenda();
}
void preparaABNTEventual()
{
    SerialDebug.println("Envia cmds ABNT:\r\n 23, 14, E2-41, 25");
    programaAlarmeAgenda();
}

void montaAgenda(void)
{
    uint8_t nEventosFreqLoRa = (24 / intervaloLoRa);
    itemAgenda_t * novoItem;

    uint32_t intervaloLoRaSegundos = intervaloLoRa *  NUM_SEG_HORA;

    //Pacotes frequentes LoRa
    for (uint8_t n = 0; n < nEventosFreqLoRa; ++n)
    {
        novoItem = criaItemAgenda((t0_LoRa + (n * intervaloLoRaSegundos))%NUM_SEG_DIA, transmiteLoRaFrequente);
        insereAgenda(&listaAgenda, novoItem);
    }

    //Pacotes eventuais LoRa
    novoItem = criaItemAgenda((t0_LoRa + OFFSET_LORA_EVENTUAL)%NUM_SEG_DIA, transmiteLoRaEventual);
    insereAgenda(&listaAgenda, novoItem);

    //Envio de comandos ABNT frequentes
    for (uint8_t n = 0; n < 24; ++n)
    {
        novoItem = criaItemAgenda((t0_ABNT + (n * NUM_SEG_HORA))%NUM_SEG_DIA, preparaABNTFrequente);
        insereAgenda(&listaAgenda, novoItem);
    }

    //Envio de comandos ABNT diario
    novoItem = criaItemAgenda((t0_ABNT + OFFSET_ABNT_EVENTUAL)%NUM_SEG_DIA, preparaABNTEventual);
    insereAgenda(&listaAgenda, novoItem);
}
// void montaAgenda(


void imprimeAgenda()
{
    uint8_t contItem = 0;
    itemAgenda_t *pImprime = itemAtual;
    SerialDebug.print(" --- AGENDA Atual - rnd: " + String(t0_LoRa));
    uint8_t horaAgenda, minAgenda, segAgenda;

    segundosParaHHMMSS(t0_LoRa, &horaAgenda, &minAgenda, &segAgenda);
    SerialDebug.println("\t " + String(horaAgenda) + ":" + String(minAgenda) + ":" + String(segAgenda));
    while (pImprime->next)
    {
        segundosParaHHMMSS(pImprime->tAgenda, &horaAgenda, &minAgenda, &segAgenda);
        SerialDebug.println(String(contItem++) + ": " + String(pImprime->tAgenda) + "   \t" + String(horaAgenda) + ":" + String(minAgenda) + ":" + String(segAgenda));
        pImprime = pImprime->next;
    }
    segundosParaHHMMSS(pImprime->tAgenda, &horaAgenda, &minAgenda, &segAgenda);
    SerialDebug.println(String(contItem++) + ": " + String(pImprime->tAgenda) + "   \t" + String(horaAgenda) + ":" + String(minAgenda) + ":" + String(segAgenda));
    SerialDebug.println(" -- FIM -- \r\n");
    SerialDebug.flush();
    TEST_ASSERT_EQUAL(24+(24/intervaloLoRa)+2, contItem);
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    pinPeripheral(1, PIO_SERCOM);
    pinPeripheral(0, PIO_SERCOM);

    Serial1.begin(ABNT_BAUD);
    Serial1.setTimeout(250);
    SerialDebug.begin(9600);
    delay (2000);
    pinMode(PIN_A1, INPUT);
    randomSeed(analogRead(PIN_A1));

    SerialDebug.println("Init...");
    SerialDebug.println("Teste de agenda");

    t0_LoRa = random(NUM_SEG_DIA);
    t0_ABNT = t0_LoRa + OFFSET_LORA_ABNT;
    intervaloLoRa = 8;
    listaAgenda = NULL;
    montaAgenda();
    itemAtual = listaAgenda;

    UNITY_BEGIN();
    RUN_TEST(alwaysTrue);
    RUN_TEST(imprimeAgenda);
    UNITY_END();
}

void loop()
{
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
}
