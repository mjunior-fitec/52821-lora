#include "test.h"
#ifdef TESTE_ABNT

#include <cdcftdi.h>
#include <usbhub.h>
#include <RTCZero.h>
#include <FlashStorage.h>

#include "abnt.h"
#include "otica_tfdi.h"
#include "pgmstrings.h"
#include "ansi_fitec.h"

#include <MKRWAN.h>

// On SAMD boards where the native USB port is also the serial console, use
// Serial1 for the serial console. This applies to all SAMD boards except for
// Arduino Zero and M0 boards.
#if (USB_VID == 0x2341 && defined(ARDUINO_SAMD_ZERO)) || \
    (USB_VID == 0x2a03 && defined(ARDUINO_SAM_ZERO))
#define SerialDebug SERIAL_PORT_MONITOR
#else
#  define SerialDebug Serial
#  define USB_DEBUG
#endif

LoRaModem modem;
// static const String appEui = SECRET_APP_EUI;
// static const String appKey = SECRET_APP_KEY;
char appEui[17] = SECRET_APP_EUI;
char appKey[33] = SECRET_APP_KEY;

bool sendNow = false;
bool respOk = false;
uint8_t ContENQ = 0;

RTCZero rtc;

abnt_resp_generic_t bufABNTrecv;
volatile uint8_t *pbufrec = NULL; //pointer to receiving data
uint16_t bytesRecABNT;

uint32_t tLastABNTSend;

bool send = false;
uint8_t cont = 0;

bool LoRaOK = false;
bool waitResp = false;
uint32_t tLastSend;

uint8_t LoRaBuff[12]; //msg a ser enviada via LoRa
uint8_t *loRaBuffPt;
uint8_t loRaBuffSize;

//info salvas
feriado_t feriadosProgramados[15];
inicioPostosHorarios_t segmentosProgramados[4];

secret_keys_t localKeys;
FlashStorage(savedKeys, secret_keys_t);

uint8_t stringAtual[10];
uint8_t stringLeitura[10] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};

uint8_t nextCmd;

uint16_t calc_crc_16(uint8_t *buffer, uint16_t len);

int8_t loRaSetup(void);
void loRaLoop(uint8_t *msg, uint8_t tam);

void setup()
{
    SerialDebug.begin(9600);
    # ifdef USB_DEBUG
    while (!SerialDebug) ;
    # endif
    SerialDebug.println("Start");

    memcpy (localKeys.appeui, (uint8_t *)&appEui, sizeof(localKeys.appeui));
    memcpy (localKeys.appkey, (uint8_t *)&appKey, sizeof(localKeys.appkey));

    pinMode (2, INPUT);
    if (digitalRead(5) == HIGH)
        savedKeys.write(localKeys);
    else if (digitalRead(5) == LOW)
        SerialDebug.println("5 - Low");

    savedKeys.read();

    ////SerialDebug.println("Tabela 03: " + String(sizeof(tabela03_t)));

    rtc.begin();
    rtc.setDate(28, 1, 19);
    rtc.setTime(10, 30, 10);

    //As programacoes sera recebida da aplicacao, numa tabela ANSI-FITec
    feriadosProgramados[0].dia = 0x05;
    feriadosProgramados[0].mes = 0x03;
    feriadosProgramados[0].ano = 0x19;
    feriadosProgramados[1].dia = 0x19;
    feriadosProgramados[1].mes = 0x04;
    feriadosProgramados[1].ano = 0x19;
    feriadosProgramados[2].dia = 0x20;
    feriadosProgramados[2].mes = 0x06;
    feriadosProgramados[2].ano = 0x19;

    for (int fer = 3; fer < 15; fer++)
    {
        feriadosProgramados[fer].dia = 0;
        feriadosProgramados[fer].mes = 0;
        feriadosProgramados[fer].ano = 0;
    }

    segmentosProgramados[0].horaInicio1     = 0x08;
    segmentosProgramados[0].minutoInicio1   = 0x00;
    segmentosProgramados[0].horaInicio2     = 0x14;
    segmentosProgramados[0].minutoInicio1   = 0x30;
    segmentosProgramados[0].horaInicio3     = 0x18;
    segmentosProgramados[0].minutoInicio3   = 0x00;
    segmentosProgramados[0].horaInicio4     = 0x00;
    segmentosProgramados[0].minutoInicio4   = 0x00;
    segmentosProgramados[1].horaInicio1     = 0x10;
    segmentosProgramados[1].minutoInicio1   = 0x30;
    segmentosProgramados[1].horaInicio2     = 0x16;
    segmentosProgramados[1].minutoInicio1   = 0x30;
    segmentosProgramados[1].horaInicio3     = 0x20;
    segmentosProgramados[1].minutoInicio3   = 0x30;
    segmentosProgramados[1].horaInicio4     = 0x00;
    segmentosProgramados[1].minutoInicio4   = 0x00;

    nextCmd = 0;

#   ifndef SEMLORA
    if (loRaSetup() < 0)
    {
        SerialDebug.println("LoRa Error!");
        LoRaOK = false;
    }
    else
        LoRaOK = true;
#   else
    LoRaOK = false;
#   endif

#  ifndef USB_DEBUG
    if (UsbH.Init())
        SerialDebug.println("USB host did not start.");
#  endif

    abntInit();
    tLastABNTSend = millis();
}

void loop()
{
    float Va = 0.0;
    float Vb = 0.0;
    float Vc = 0.0;

    abnt_resp_le_grand_instant_t *respLeGrandInst;

    localKeys = savedKeys.read();
    // SerialDebug.println(String((char *)(localKeys.appeui)));
    // SerialDebug.println(String((char *)(localKeys.appkey)));
    // delay(1200);

    //SerialDebug.println("1.Bfr USB");
    # ifndef USB_DEBUG
    UsbH.Task(); //### Verificar FTDI no emuration!
    #  endif

    if (UsbH.getUsbTaskState() == USB_STATE_RUNNING)
    {
        maqEstAbnt();
        if ((millis() - tLastABNTSend) > MIN_ABNT_SEND_INTERVAL)
        {
            if (stateABNT == ABNT_STATE_IDLE)
            {
                tLastABNTSend = millis();
                if (nextCmd == 0)
                    nextCmd = ID_CMD14;
                sinaliza_cmd_abnt(nextCmd);
            }
        }
        if (respABNT_OK)
        {
            nextCmd = 0;
            if (((abnt_resp_generic_t *)pBuffABNTrecv)->id == ID_CMD13)
            {
                abnt_resp_pedido_string_t *respPedidoString = (abnt_resp_pedido_string_t *)pBuffABNTrecv;
                for (uint8_t i = 0; i < 10; i++)
                {
                    stringAtual[i] = respPedidoString->string_senha[i];
                }
                nextCmd = ID_CMD11;
            }

            if ( ((abnt_resp_generic_t *)pBuffABNTrecv)->id != ID_CMD14) {
                SerialDebug.println("Cmd OK!");
                delay(1500);
                abntInit();
                for (uint8_t i=0; i<30; i++)
                {
                    SerialDebug.print(String(pBuffABNTrecv[i],HEX) + "\t");
                }
                SerialDebug.println();

                if (((abnt_resp_generic_t *)pBuffABNTrecv)->id == ID_CMD25)
                {
                    for (uint16_t ii = 245; ii < 258; ii++)
                    {
                        SerialDebug.print(String(pBuffABNTrecv[ii], HEX) + "\t");
                    }
                    SerialDebug.println();
                }
                return;
            }

            respLeGrandInst = (abnt_resp_le_grand_instant_t *)pBuffABNTrecv;

            Va = respLeGrandInst->tensao_a;
            Vb = respLeGrandInst->tensao_b;
            Vc = respLeGrandInst->tensao_c;

            if (LoRaOK)
            {
                loRaBuffSize = 0;
                memcpy(loRaBuffPt + loRaBuffSize, (uint8_t *)&Va, sizeof(Va));
                loRaBuffSize += sizeof(Va);
                memcpy(loRaBuffPt + loRaBuffSize, (uint8_t *)&Vb, sizeof(Vb));
                loRaBuffSize += sizeof(Vb);
                memcpy(loRaBuffPt + loRaBuffSize, (uint8_t *)&Vc, sizeof(Vc));
                loRaBuffSize += sizeof(Vc);
            }

            SerialDebug.println("Dia: " + String(respLeGrandInst->dia, HEX) +
                                "/" + String(respLeGrandInst->mes, HEX) +
                                "/" + String(respLeGrandInst->ano, HEX) +
                                "\t\tHora: " + String(respLeGrandInst->hora, HEX) +
                                ":" + String(respLeGrandInst->min, HEX) +
                                ":" + String(respLeGrandInst->seg, HEX) +
                                "\t\tVa: " + String(respLeGrandInst->tensao_a) +
                                "\tVbc: " + String(respLeGrandInst->tensao_bc));

            abntInit();

            SerialDebug.println("RTC local:\nDia: " + String(rtc.getDay()) +
                                "/" + String(rtc.getMonth()) +
                                "/" + String(rtc.getYear()) +
                                "\t\tHora: " + String(rtc.getHours()) +
                                ":" + String(rtc.getMinutes()) +
                                ":" + String(rtc.getSeconds()) +
                                " Sem - " + String(CalcWeekDayNumFromDate(rtc.getYear(), rtc.getMonth(), rtc.getDay())) );
        }
    }
    else
    {
        SerialDebug.println("USB Not running...");
        delay(300);
    }
    if (LoRaOK)
        loRaLoop(loRaBuffPt, loRaBuffSize);
    SerialDebug.println("St = " + String(stateABNT));
}

int8_t loRaSetup(void)
{
    if (!modem.begin(AU915))
    {
        SerialDebug.println("Failed to start module");
        return -1;
    }
    SerialDebug.print("Your module version is: ");
    SerialDebug.println(modem.version());
    SerialDebug.print("Your device EUI is: ");
    SerialDebug.println(modem.deviceEUI());
    delay(500);

    //-------- DEBUG ----------
    SerialDebug.println("App EUI is: " + String(appEui));
    SerialDebug.println("App key is: " + String(appKey));
    //-------- DEBUG ----------

    SerialDebug.println("Before JOIN:");
    if (modem.publicNetwork(false))
        SerialDebug.println("Private mode!");

    if (modem.getDataRate())
        SerialDebug.println("Get DR OK!");
    else
        SerialDebug.println("Get DR ERR!");

    appEui[16] = appKey[32] = 0;
    //int connected = modem.joinOTAA(appEui, appKey);
    int connected = modem.joinOTAA(String(appEui), String(appKey));
    if (!connected)
    {
        SerialDebug.println("Something went wrong; are you indoor? Move "
                            "near a window and retry");
        return -1;
    }

    if (modem.dutyCycle(false))
        SerialDebug.println("Duty cycle desligado com sucesso!");

    if (modem.getDataRate())
        SerialDebug.println("Get DR OK!");
    else
        SerialDebug.println("Get DR ERR!");

    loRaBuffPt = LoRaBuff;
    loRaBuffSize = 0;

    tLastSend = millis();
    // Set poll interval to 60 secs.
    modem.minPollInterval(60);
    // NOTE: independently by this setting the modem will
    // not allow to send more than one message every 2 minutes,
    // this is enforced by firmware and can not be changed.

    return 0;
}

void loRaLoop(uint8_t *msg, uint8_t tam)
{
    int err;

    if ((millis() - tLastSend) > MIN_LORA_INTERVAL)
    {
        tLastSend = millis();
        waitResp = true;
        //SerialDebug.println("Sending "+String(tam)+" bytes\n [0] = "+String(msg[0]));
        modem.beginPacket();
        modem.write(msg, tam);
        err = modem.endPacket(true);

        if (err > 0)
        {
            SerialDebug.println("Message sent correctly!");
        }
        else
        {
            SerialDebug.println("Error sending message :(");
        }
    }

    if (waitResp && ((millis() - tLastSend) > LORA_RESP_INTERVAL))
    {
        waitResp = false;
        if (!modem.available())
        {
            SerialDebug.println("No downlink message received at this time.");
            return;
        }
        char rcv[64];
        unsigned int i = 0;
        while (modem.available())
        {
            rcv[i++] = (char)modem.read();
        }
        SerialDebug.print("DL Recvd: ");
        for (unsigned int j = 0; j < i; j++)
        {
            SerialDebug.print(rcv[j] >> 4, HEX);
            SerialDebug.print(rcv[j] & 0xF, HEX);
            SerialDebug.print(" ");
        }
        SerialDebug.println();
    }
}

#endif //TESTE_ABNT
