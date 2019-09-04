/** @file semRTOS.cpp
* @brief Teste do sistema base sem RTOS
*
* Teste do sistema completo, com comunicação ABNT via serial nativa
* e com conversor USB (FTDI), comunicação LoRa no protocolo ANSI-FITec
* e tratamento de agendas.
*
* @date Feb/2019
* @author FITec - Fundacao para Inovacoes Tecnologicas
*         Marcio J Teixeira Jr. <mjunior@fitec.org.br>
*
************************************************************************/


#include "test.h"
#include "util.h"
#include <Arduino.h>
#include <stdint.h>
#ifdef TEST_SEMRTOS

#include "abnt.h"
#include "otica_tfdi.h"
#include "serial_nativa.h"
#include "pgmstrings.h"
#include "ansi_fitec.h"

#include <cdcftdi.h>
#include <usbhub.h>
#include <RTCZero.h>
#include <FlashStorage.h>
#include <ArduinoLowPower.h>
#include <MKRWAN.h>

#if defined (USB_DEBUG)
# define SerialDebug SerialUSB
#elif defined (DEBUG_SERIAL3)
# include "wiring_private.h"
# define SerialDebug Serial3
#else
# define SerialDebug Serial1
#endif

#ifdef DEBUG_SERIAL3
Uart Serial3(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM3_Handler()
{
    Serial3.IrqHandler();
}
#endif

LoRaModem modem;
char appEui[17] = SECRET_APP_EUI;
char appKey[33] = SECRET_APP_KEY;

RTCZero rtc;
secret_keys_t localKeys;
FlashStorage(savedKeys, secret_keys_t); //Declara a variavel savedKeys
//--- Flash endurance: (25k - 100k) cycles

//Variaveis relativas ao modulo ABNT
uint32_t tLastABNTSend;

// Funcoes locais
int8_t loRaSetup(void);
void loRaLoop(uint8_t *msg, uint8_t tam);
void trataLastGasp(void);
void debugSleepTime(void);
void verificaAlimentacaoExterna (void);
void reboot(void);

bool LoRaOK = false;
bool waitResp = false;
uint32_t tLastSend;

uint8_t LoRaBuff[51]; //msg a ser enviada via LoRa
uint8_t *loRaBuffPt;
uint8_t loRaBuffSize;

feriado_t feriadosProgramados[15];
inicioPostosHorarios_t segmentosProgramados[4];

uint8_t stringAtual[10];
uint8_t stringLeitura[10] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};

uint8_t nextCmd;

volatile bool lastGasp = false;
volatile bool confirmaAlimentacaoExterna = false;
volatile bool aguardaAlimentacaoExterna = false;
volatile bool reprogramaISR = false;
volatile bool brownOut = false;

uint8_t contAlive=0;

////// ----> Criar sitema de agenda para tarefas (ABNT e LoRa)!

static const String deviceAddr = ABP_DEVADDR;
static const String NetworkSKey = ABP_NWKSKEY;
static const String AppSessionKey = ABP_APPSKEY;

void setup()
{

#  ifdef DEBUG_SERIAL3
    pinPeripheral(1, PIO_SERCOM);
    pinPeripheral(0, PIO_SERCOM);
#  endif

    SerialDebug.begin(DEBUG_BAUD);
#  ifdef USB_DEBUG
    int waitForSerial = 3;
    while (!SerialUSB && !SerialUSB.available() && waitForSerial ) {
    delay(2000);
    piscaLed(2);
    waitForSerial--;
  }
#  endif


#ifdef TESTE_BOD_INTERRUPT
    SYSCTRL->BOD33.bit.ENABLE = 0;
    SYSCTRL->BOD33.bit.HYST = 1;     // hysteresis ON
    SYSCTRL->BOD33.bit.RUNSTDBY = 1; // enable in sleep
    SYSCTRL->BOD33.bit.ACTION = 2;   // 1 - generates a reset, 2 - generates an interrupt
    SYSCTRL->BOD33.bit.LEVEL = 30;   // 28 = ~2,3V, default = 7
    NVIC_EnableIRQ(SYSCTRL_IRQn);
    SYSCTRL->INTENSET.bit.BOD33DET = 1;  // Enables the interrupt
    SYSCTRL->INTFLAG.bit.BOD33DET = 1;   // clears the flag to use the interrupt
    SYSCTRL->BOD33.bit.ENABLE = 1;
#endif

    pinMode(PIN_POWER_FAILURE, INPUT_PULLUP);
    pinMode(PIN_LEITURA_VIN, INPUT_PULLDOWN);
    pinMode(PIN_TEST_ISR, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_POWER_FAILURE), trataLastGasp, RISING); //programa interrupção para ler sinal de power failure
    LowPower.attachInterruptWakeup(RTC_ALARM_WAKEUP, debugSleepTime, CHANGE);

    SerialDebug.println("Start");

    pinMode(2, INPUT_PULLDOWN);
    if (digitalRead(2) == HIGH) //Teste com pino controlando a gravacao de chaves default
    {
        memcpy(localKeys.appeui, (uint8_t *)&appEui, sizeof(localKeys.appeui));
        memcpy(localKeys.appkey, (uint8_t *)&appKey, sizeof(localKeys.appkey));
        savedKeys.write(localKeys);
        SerialDebug.println("Chaves default salvas");
    }
    else if (digitalRead(2) == LOW)
        SerialDebug.println("2 - Low");

    localKeys = savedKeys.read();

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(SERIAL_RTS, OUTPUT);
    digitalWrite(SERIAL_RTS, LOW); //Low TTL = +5,5V RS232
    digitalWrite(LED_BUILTIN, LOW);

    digitalWrite(PIN_TEST_ISR, LOW);

    rtc.begin();
    rtc.setDate(27, 2, 19);
    rtc.setTime(11, 30, 10); //Incializacao manual do RTC
    ///########## ---> Cmd de acertar relogio no downlink, PRECISA de mudar para classe C e ficar esperando receber qdo IDLE !!!  <<-----------------

    piscaLed(5); //Sinaliza inicializacao antes de fazer o JOIN

#  ifndef SEMLORA
    if (loRaSetup() < 0)
    {
        SerialDebug.println("LoRa Error!");
        LoRaOK = false;
    }
    else
        LoRaOK = true;
#  else
    LoRaOK = false;
#  endif

#  ifndef USB_DEBUG
    if (UsbH.Init());
#  endif

    abntInit();
    tLastABNTSend = millis();
    nextCmd = 0;
    delay(500);
}

void loop()
{
    float Va = 0.0;
    float Vb = 0.0;
    float Vc = 0.0;

    if (lastGasp)
    {
        if (brownOut)
        {
            brownOut = false;
            SerialDebug.println("BOD!");
            SerialDebug.flush();
        }
        piscaLed(2, 50, 100);
        digitalWrite(PIN_TEST_ISR, LOW);
        //SerialDebug.println("Last gasp: " + String(millis()));
        lastGasp = false;
        reprogramaISR = true;
        //LowPower.deepSleep(1000);
        return;
    }

    if (reprogramaISR)
    {
        attachInterrupt(digitalPinToInterrupt(PIN_POWER_FAILURE), verificaAlimentacaoExterna, CHANGE);
        aguardaAlimentacaoExterna = true;
        reprogramaISR = false;
        return;
    }

    if (aguardaAlimentacaoExterna)
    {
        bool vin = digitalRead(PIN_LEITURA_VIN);
        //SerialDebug.println("sl: " + String(millis()));
        // SerialDebug.println("flag: " + String(confirmaAlimentacaoExterna));
        // SerialDebug.println("Vin: " + String(vin));
        if (confirmaAlimentacaoExterna)
        {
            piscaLed(1, 50, 50);
            digitalWrite(PIN_TEST_ISR, LOW);
            for (uint8_t iConf = 0; iConf < 5; iConf++)
            {
                delay(30);
                if (!vin)
                {
                    confirmaAlimentacaoExterna = false;
                    return;
                }
            }
            if (vin)
                reboot();
        }
        //LowPower.deepSleep(1000);   //(SLEEP_PERIOD);
        return;
    }

    SerialDebug.println ("Alive: " + String(contAlive++));
    piscaLed(2, 100, 150);
    delay(500);
    return;

    abnt_resp_le_grand_instant_t *respLeGrandInst;
    maqEstAbnt();
    if (SERIAL_NULL != portaSerial.interface)
    {
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
            if ( ((abnt_resp_generic_t *)pBuffABNTrecv)->id != ID_CMD14) {
                SerialDebug.println("Cmd OK!");
                delay(1500);
                abntInit();
                for (uint8_t i=0; i<30; i++)
                {
                    SerialDebug.print(String(pBuffABNTrecv[i],HEX) + "\t");
                }
                SerialDebug.println();
                return;
            }

            respLeGrandInst = (abnt_resp_le_grand_instant_t *)pBuffABNTrecv;

            Va = respLeGrandInst->tensao_a;
            Vb = respLeGrandInst->tensao_b;
            Vc = respLeGrandInst->tensao_c;
            //respLeGrandInst->

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
        }
    }
    else
    {
        SerialDebug.println("!Serial");
        delay(1000);
        return;
    }
    #ifndef SEMLORA
    if (LoRaOK)
        loRaLoop(loRaBuffPt, loRaBuffSize);
    #endif
    SerialDebug.println("St = " + String(stateABNT));
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
}


#  ifndef SEMLORA
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
    delay(200);

    //-------- DEBUG ----------
    SerialDebug.println("App EUI is: " + String(appEui));
    SerialDebug.println("App key is: " + String(appKey));
    delay(200);
    //-------- DEBUG ----------

    SerialDebug.println("Before JOIN:");
    if (modem.publicNetwork(false))
        SerialDebug.println("Private mode!");

    appEui[16] = appKey[32] = 0;
    int connected = modem.joinOTAA(String(appEui), String(appKey));
    //int connected = modem.joinABP (deviceAddr, NetworkSKey, AppSessionKey); //   ABP_DEVADDR, ABP_NWKSKEY, ABP_APPSKEY);

    if (!connected)
    {
        SerialDebug.println("Something went wrong; are you indoor? Move "
                            "near a window and retry");
        return -1;
    }

    if (modem.dutyCycle(false))
        SerialDebug.println("Duty cycle desligado com sucesso!");

    if (modem.configureClass(CLASS_C))
        SerialDebug.println("Muda classe OK");
    else
        SerialDebug.println("Erro ao mudar de classe");

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
            SerialDebug.println("Msg sent OK!");
        else
            SerialDebug.println("Msg sent  ERR");
    }

    if (waitResp && ((millis() - tLastSend) > LORA_RESP_INTERVAL))
    {
        waitResp = false;
        if (!modem.available())
        {
            SerialDebug.println("No downlink rcvd.");
            return;
        }
        char rcv[64];
        unsigned int i = 0;
        while (modem.available())
        {
            rcv[i++] = (char)modem.read();
        }
        SerialDebug.print("DL rcvd: ");
        for (unsigned int j = 0; j < i; j++)
        {
            SerialDebug.print(rcv[j] >> 4, HEX);
            SerialDebug.print(rcv[j] & 0xF, HEX);
            SerialDebug.print(" ");
        }
        SerialDebug.println();
    }
}
#  endif //SEMLORA

void debugSleepTime(void)
{
    return;
    // SerialDebug.println("Alive: " + String(millis()));
    // LowPower.deepSleep(SLEEP_PERIOD);
}

void trataLastGasp(void)
{
    lastGasp = true;
    digitalWrite(PIN_TEST_ISR, HIGH);
    //detachInterrupt(digitalPinToInterrupt(PIN_POWER_FAILURE));
}

void verificaAlimentacaoExterna(void)
{
    confirmaAlimentacaoExterna = true;
    digitalWrite(PIN_TEST_ISR, HIGH);
}

void reboot(void)
{
    piscaLed(3, 100);
    NVIC_SystemReset();
    while (1)
        ;
}

//Tratamento da interrucao de Brown-out
void SYSCTRL_Handler()
{
    if (SYSCTRL->INTFLAG.bit.BOD33DET)
    {
        lastGasp = true;
        brownOut = true;
        digitalWrite(PIN_TEST_ISR, HIGH);
        SYSCTRL->INTFLAG.bit.BOD33DET = 1; //reconhece a interrupcao
    }
}

#endif // TEST_SEMRTOS

#ifdef TEST_LORA_CLASS_C
#include <cdcftdi.h>
#include <usbhub.h>

#include "abnt.h"
#include "otica_tfdi.h"
#include "pgmstrings.h"
#include <MKRWAN.h>

#if defined (USB_DEBUG)
# define SerialDebug SerialUSB
#elif defined (DEBUG_SERIAL3)
# include "wiring_private.h"
# define SerialDebug Serial3
#else
# define SerialDebug Serial1
#endif

#ifdef DEBUG_SERIAL3
Uart Serial3(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM3_Handler()
{
    Serial3.IrqHandler();
}
#endif

LoRaModem modem;
bool loRaOk = false;

static const String appEui = SECRET_APP_EUI;
static const String appKey = SECRET_APP_KEY;

static const String deviceAddr = ABP_DEVADDR;
static const String NetworkSKey = ABP_NWKSKEY;
static const String AppSessionKey = ABP_APPSKEY;

volatile uint8_t contAlive;
const uint8_t fixo = 0xA5;

void setup()

{
#  ifdef DEBUG_SERIAL3
    pinPeripheral(1, PIO_SERCOM);
    pinPeripheral(0, PIO_SERCOM);
#  endif
    SerialDebug.begin(DEBUG_BAUD);
#  ifdef USB_DEBUG
    int waitForSerial = 3;
    while (!SerialUSB && !SerialUSB.available() && waitForSerial ) {
    delay(2000);
    piscaLed(2);
    waitForSerial--;
  }
#  endif

    // change this to your regional band (eg. US915, AS923, ...)
    if (!modem.begin(AU915))
    {
        SerialDebug.println("Failed to start module");
    }

    delay(500);

    SerialDebug.print("Your module version is: ");
    SerialDebug.println(modem.version());
    SerialDebug.print("Your device EUI is: ");
    SerialDebug.println(modem.deviceEUI());

    //-------- DEBUG ----------
    SerialDebug.println("App EUI is: " + appEui);
    SerialDebug.println("App key is: " + appKey);

    Serial.println("Dev Addr is : " + deviceAddr);
    Serial.println("Nwk S key is: " + NetworkSKey);
    Serial.println("App S key is: " + AppSessionKey);
    //-------- DEBUG ----------

    if (modem.publicNetwork(false))
        SerialDebug.println("Private mode ok!");

    SerialDebug.println("Starting JOIN...");
    //// int connected = modem.joinOTAA(appEui, appKey);
    int connected = modem.joinABP (deviceAddr, NetworkSKey, AppSessionKey); //   ABP_DEVADDR, ABP_NWKSKEY, ABP_APPSKEY);

    if (!connected)
    {
        SerialDebug.println("Something went wrong; are you indoor? Move "
                            "near a window and retry");
    }
    else
        loRaOk = true;

    contAlive = 0;
}
void loop()
{
    int err;
    uint8_t msg[10];

    delay (1000);
    SerialDebug.println("alive... ");

    if (modem.configureClass(CLASS_C))
        SerialDebug.println("Muda classe OK");
    else
        SerialDebug.println("Erro ao mudar de classe");

    if (!loRaOk)
        SerialDebug.println("Sem LoRa!!!");

    else
    {
        //float testFloat[3]= {124.0, 124.0, 124.0};
        //CAYENNE: ch = 01, type = 02 (analog input - signed 0.01), valor, valor
        //memcpy((uint8_t *)msg, (uint8_t *)testFloat, sizeof(testFloat));

        msg[0] = 'H';
        msg[1] = 'e';
        msg[2] = contAlive++;
        msg[3] = 0;
        //SerialDebug.println("msg = " + String((char *)msg));
        modem.beginPacket();
        //modem.print(msg);
        //modem.write(msg, 4); //sizeof(msg));
        modem.write(fixo);
        //modem.write((uint8_t *)&testFloat, 4);
        err = modem.endPacket(true);

        if (err > 0)
        {
            SerialDebug.println("Message sent correctly!");
        }
        else
        {
            SerialDebug.println("Error sending message :(");
        }
        delay(1000);
        if (!modem.available())
        {
            SerialDebug.println("No downlink message received at this time.");
            return;
        }
        char rcv[64];
        int i = 0;
        while (modem.available())
        {
            rcv[i++] = (char)modem.read();
        }
        Serial.print("Received: ");
        for (unsigned int j = 0; j < i; j++)
        {
            SerialDebug.print(rcv[j] >> 4, HEX);
            SerialDebug.print(rcv[j] & 0xF, HEX);
            SerialDebug.print(" ");
        }
        SerialDebug.println();
    }
}

#endif //TEST_LORA_CLASS_C


#ifdef TESTE_PWRFAIL
#include <ArduinoLowPower.h>
# include "wiring_private.h"

# define SerialDebug Serial3
Uart Serial3(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM3_Handler() { Serial3.IrqHandler();}

void trataLastGasp(void);
void verificaAlimentacaoExterna (void);
void reboot(void);
volatile bool lastGasp = false;
volatile bool confirmaAlimentacaoExterna = false;
volatile bool aguardaAlimentacaoExterna = false;
volatile bool reprogramaISR = false;
volatile bool brownOut = false;

uint8_t contAlive=0;

void setup()
{
    SerialDebug.begin(DEBUG_BAUD);
    pinPeripheral(1, PIO_SERCOM);
    pinPeripheral(0, PIO_SERCOM);

    SYSCTRL->BOD33.bit.ENABLE = 0;

    // SYSCTRL->BOD33.bit.HYST = 1;     // hysteresis ON
    // SYSCTRL->BOD33.bit.RUNSTDBY = 1; // enable in sleep
    // SYSCTRL->BOD33.bit.ACTION = 2;   // 1 - generates a reset, 2 - generates an interrupt
    // SYSCTRL->BOD33.bit.LEVEL = 30;   // 28 = ~2,3V, default = 7
    // NVIC_EnableIRQ(SYSCTRL_IRQn);
    // SYSCTRL->INTENSET.bit.BOD33DET = 1;  // Enables the interrupt
    // SYSCTRL->INTFLAG.bit.BOD33DET = 1;   // clears the flag to use the interrupt
    // SYSCTRL->BOD33.bit.ENABLE = 1;

    pinMode(PIN_POWER_FAILURE, INPUT_PULLUP);
    pinMode(PIN_LEITURA_VIN, INPUT_PULLDOWN);
    pinMode(PIN_TEST_ISR, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_POWER_FAILURE), trataLastGasp, RISING); //programa interrupção para ler sinal de power failure
    //LowPower.attachInterruptWakeup(RTC_ALARM_WAKEUP, debugSleepTime, CHANGE);

    SerialDebug.println("Start");
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(PIN_TEST_ISR, LOW);

    //piscaLed(3);
    delay(200);
}

void loop()
{
    if (lastGasp)
    {
        return;
        //piscaLed(2, 50, 100);
        digitalWrite(PIN_TEST_ISR, LOW);
        SerialDebug.println("Last gasp: ");// + String(millis()));
        lastGasp = false;
        //reprogramaISR = true;
        detachInterrupt(digitalPinToInterrupt(PIN_POWER_FAILURE));

        ////// Simula salvamento de contexto, etc.
        delay(10);
        digitalWrite(PIN_TEST_ISR, HIGH);
        delay(15);
        digitalWrite(PIN_TEST_ISR, LOW);

        //LowPower.deepSleep(1000);
        return;
    }

    if (reprogramaISR)
    {
        SerialDebug.println("Troca ISR");
        attachInterrupt(digitalPinToInterrupt(PIN_POWER_FAILURE), verificaAlimentacaoExterna, CHANGE);
        aguardaAlimentacaoExterna = true;
        reprogramaISR = false;
        return;
    }

    if (aguardaAlimentacaoExterna)
    {
        bool vin = digitalRead(PIN_LEITURA_VIN);
        SerialDebug.println("chkVin: " + String(confirmaAlimentacaoExterna));
        SerialDebug.println("Vin: " + String(vin));
        if (confirmaAlimentacaoExterna)
        {
            //piscaLed(1, 50, 50);
            digitalWrite(PIN_TEST_ISR, LOW);
            for (uint8_t iConf = 0; iConf < 5; iConf++)
            {
                delay(30);
                if (!vin)
                {
                    confirmaAlimentacaoExterna = false;
                    return;
                }
            }
            if (vin)
            {
                SerialDebug.println("REBOOT !!!");
                reboot();
            }
        }
        //LowPower.deepSleep(1000);   //(SLEEP_PERIOD);
        delay(250);
        return;
    }

    SerialDebug.println("Alive: " + String(contAlive++));
    piscaLed(2, 100, 150);
    delay(500);
    return;
}


void trataLastGasp(void)
{
    volatile uint32_t aa=0;
    lastGasp = true;
    digitalWrite(PIN_TEST_ISR, HIGH);

    for (uint32_t t_delay1 = 0; t_delay1<30; t_delay1++)
        for (uint32_t t_delay2 = 0; t_delay2<4000000000; t_delay2++)
            aa++;

    digitalWrite(PIN_TEST_ISR, LOW);
}

void verificaAlimentacaoExterna(void)
{
    confirmaAlimentacaoExterna = true;
    digitalWrite(PIN_TEST_ISR, HIGH);
}

void reboot(void)
{
    //piscaLed(3, 100);
    NVIC_SystemReset();
    while (1)
        ;
}
#endif //TESTE_PWRFAIL


#ifdef TEST_SERIALNATIVA

#include "serial_nativa.h"
#include "abnt.h"
#include "otica_tfdi.h"

#ifdef USB_DEBUG
#  define SerialDebug Serial
#endif

uint16_t calc_crc_16(uint8_t *buffer, uint16_t len);

bool sendNow = false;
bool respOk = false;
uint8_t ContENQ = 0;

abnt_resp_generic_t bufABNTrecv; //ABNT rcvd buffer is fixed 258 bytes long
volatile uint8_t *pbufrec = NULL; //pointer to receiving data
uint16_t bytesRecABNT;

bool send = false;
uint8_t cont = 0;
.
RTCZero rtc;

uint8_t nextCmd;

//Variaveis relativas ao modulo ABNT
uint32_t tLastABNTSend;

feriado_t feriadosProgramados[15];
inicioPostosHorarios_t segmentosProgramados[4];

uint8_t stringAtual[10];
uint8_t stringLeitura[10] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};

void setup()
{
    Serial1.begin(ABNT_BAUD);
    Serial1.setTimeout(250);

# ifdef USB_DEBUG
    SerialDebug.begin(115200);
    while (!SerialDebug) ;
    SerialDebug.println("Start");
# endif

    delay(200);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(SERIAL_RTS, OUTPUT);
    digitalWrite(SERIAL_RTS, LOW); //Low TTL = +5,5V RS232
    digitalWrite(LED_BUILTIN, LOW);

    // portaSerial.interface = SERIAL_RS232;
    // portaSerial.recMsgMedidor = recv_dataSerial;
    // portaSerial.sendCmdMedidor = send_dataSerial;

    for (int k=0;k<3;k++)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(150);
        digitalWrite(LED_BUILTIN, LOW);
        delay(150);
    }
    delay(1500);

    // if (UsbH.Init())
    //     SerialDebug.println("USB host did not start.");

    abntInit();
    nextCmd = 0;

    for (int k=0;k<5;k++)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(250);
        digitalWrite(LED_BUILTIN, LOW);
        delay(150);
    }
    delay(500);
}

// void loop()
//   {
//     uint16_t bytesRec = 0;
//     uint8_t nBytes;
//     uint8_t buff1[200];
//     memset(buff1, 0, sizeof(buff1));

//     nBytes = Serial1.available();
//     if (nBytes)
//     {
//         bytesRec = Serial1.readBytes(buff1, nBytes);
//         //   SerialDebug.println(Serial1.read(), HEX);
//         SerialDebug.println("rec: " + String(bytesRec));
//         for (int i = 0; i < nBytes; ++i)
//             SerialDebug.println(buff1[i], HEX);
//     }
//     else
//         SerialDebug.println(".");
//     delay(1300);
// }

#if 1
void loop()
{
    uint32_t rcode = 0;
    uint8_t bufsend[] = CMD_14;

    maqEstAbnt();
    //if (SERIAL_NULL != portaSerial.interface)
# ifdef USB_DEBUG
    SerialDebug.println("st: " + String(stateABNT));
# endif
    if (stateABNT != ABNT_STATE_DISCONNECTED)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        if (SERIAL_RS232 == portaSerial.interface)
        {
            digitalWrite(LED_BUILTIN, LOW);
            delay(200);
            digitalWrite(LED_BUILTIN, HIGH);
        }
        else if (SERIAL_USB == portaSerial.interface)
        {
            digitalWrite(LED_BUILTIN, LOW);
            delay(450);
            digitalWrite(LED_BUILTIN, HIGH);
        }

        if (sendNow)
        {
            //rcode = Serial1.write(bufsend, (size_t)LEN_CMD);
            //send_dataSerial(LEN_CMD, bufsend);
            nextCmd = ID_CMD14;
            sinaliza_cmd_abnt(nextCmd);
            sendNow = false;
            respOk = false;
            //pbufrec = bufABNTrecv;
            ContENQ = 0;
#       ifdef USB_DEBUG
            SerialDebug.println("Preparing to send cmd!");
#       endif
            if (rcode)
            {
#       ifdef USB_DEBUG
                SerialDebug.println("sent: " + String(rcode));
                //SerialDebug.println("\t ->>> ERROR SENDING !!! <<<-");
#       endif
            }
            Serial1.flush();
            return;
        }

        delay(80);
    # ifdef USB_DEBUG
        SerialDebug.println("\t Delay 80m");
    # endif

        bytesRecABNT = recvMsgMedidorABNT(&bufABNTrecv);
        //SerialDebug.println("\t\t --Aft");
        if (1 == bytesRecABNT)
        {
            if (*((uint8_t *)&bufABNTrecv) == ABNT_ENQ) //ENQ
            {
            # ifdef USB_DEBUG
                SerialDebug.println("\n\t\tContENQ = " + String(ContENQ));
            # endif
                if (++ContENQ > 9)
                {
                    sendNow = true;
                    ContENQ = 0;
                }
            }
        }
        # ifdef USB_DEBUG
        SerialDebug.println("ABNT rec: " + String(bytesRecABNT));
        # endif

        if (LEN_RES == bytesRecABNT)
        {
            if (calc_crc_16((uint8_t *)&bufABNTrecv, LEN_RES))
            {
            # ifdef USB_DEBUG
                SerialDebug.println("CRC ruim");
            # endif
            }
            else
            {
                respOk = true;
            # ifdef USB_DEBUG
                SerialDebug.println("CRC OK!");
                SerialDebug.println("Dia: " + String((*(abnt_resp_le_grand_instant_t *)&bufABNTrecv).dia, HEX) +
                                    "\tMes: " + String((*(abnt_resp_le_grand_instant_t *)&bufABNTrecv).mes, HEX) +
                                    "\tAno: " + String((*(abnt_resp_le_grand_instant_t *)&bufABNTrecv).ano, HEX) +
                                    "\t\tHora: " + String((*(abnt_resp_le_grand_instant_t *)&bufABNTrecv).hora, HEX) +
                                    "\tMinuto: " + String((*(abnt_resp_le_grand_instant_t *)&bufABNTrecv).min, HEX) +
                                    "\tSegundo: " + String((*(abnt_resp_le_grand_instant_t *)&bufABNTrecv).seg, HEX) +
                                    "\t\tVa: " + String((*(abnt_resp_le_grand_instant_t *)&bufABNTrecv).tensao_a) +
                                    "\tVbc: " + String((*(abnt_resp_le_grand_instant_t *)&bufABNTrecv).tensao_bc));
            # endif
            }
        }
    }
    else
        digitalWrite(LED_BUILTIN, LOW);
#if 0
        uint8_t buf[SIZE_RECV];
        memset(buf, 0, SIZE_RECV);

        uint16_t rcvd = SIZE_RECV;
        rcode = Ftdi.RcvData(&rcvd, buf);

        if (rcode && rcode != USB_ERRORFLOW)
            ErrorMessage<uint8_t>(PSTR("Ret"), rcode);

        // The device reserves the first two bytes of data to
        // contain the current values of the modem and line status registers.
        //if (rcvd > 2)
        //    SerialDebug.print((char*)(buf+2));

        if (rcvd > 2)
        {
            SerialDebug.println("RECEIVED " + String(rcvd) + " bytes!");
            /*for (uint16_t i_rcv=2; i_rcv < rcvd; i_rcv++){
                SerialDebug.print("\t [" + String(i_rcv-2) + "]-> 0x");
                SerialDebug.print(buf[i_rcv], HEX);
            }
            SerialDebug.println("\t FIM");*/
            if ((rcvd == 3) && (buf[2] == 5)) //Recebeu um ACK
            {
                SerialDebug.println("ContENQ = " + String(ContENQ));
                // uint16_t rnd1 = random(650);
                // SerialDebug.println("Lucky = " + String(rnd1));
                if (++ContENQ > 9)
                    sendNow = true;
            }
            else //recebeu dados!
            {
                rcvd -= 2;
                memcpy((uint8_t *)pbufrec, (uint8_t *)(buf+2), rcvd);
                pbufrec += rcvd; //adjust the pointer to receive next chunk
                SerialDebug.println("bytes = " +\
                                     String((uint16_t) (pbufrec-bufABNTrecv)));
                if (258 == (pbufrec-bufABNTrecv))
                {
                    SerialDebug.println("CRC rec: " +\
                                String(*(uint16_t *)(bufABNTrecv+PAYLOAD_RES)));
                    SerialDebug.println("CRC calc: " +\
                                String(calc_crc_16(bufABNTrecv, PAYLOAD_RES)));
                    SerialDebug.println("CRC compl: " +\
                                String(calc_crc_16(bufABNTrecv, LEN_RES)));

                    // SerialDebug.println("CRC rec: " + String(*(uint16_t *)(bufsend+PAYLOAD_CMD)));
                    // SerialDebug.println("CRC calc: " + String(calc_crc_16(bufsend, PAYLOAD_CMD)));
                    // SerialDebug.println("CRC compl: " + String(calc_crc_16(bufsend, LEN_CMD)));

                    //if (calc_crc_16(bufsend, LEN_CMD))
                    if (calc_crc_16(bufABNTrecv, LEN_RES))
                        SerialDebug.println("CRC ruim");
                    else
                    {
                        SerialDebug.println("CRC OK!");
                        respOk = true;
                        abnt_resp_le_grand_instant_t resp14;
                        resp14 = *(abnt_resp_le_grand_instant_t *)bufABNTrecv;
                        SerialDebug.println("Dia: " + String(resp14.dia, HEX) +\
                                     "\tMes: " + String(resp14.mes, HEX) +\
                                     "\tAno: " + String(resp14.ano, HEX) +\
                                     "\t\tHora: " + String(resp14.hora, HEX) +\
                                     "\tMinuto: " + String(resp14.min, HEX) +\
                                     "\tSegundo: " + String(resp14.seg, HEX) +\
                                     "\t\tVa: " + String(resp14.tensao_a) +\
                                     "\tVbc: " + String(resp14.tensao_bc)\
                                     );
                    }
                }
            }
        }
        //delay(2);
#endif
}
#endif

/*
 * Tabela pre-calculada para o polinomio CRC16 (0xA001)
 */
static unsigned short crc16_table[256] = {
        0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241,\
        0xc601, 0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440,\
        0xcc01, 0x0cc0, 0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40,\
        0x0a00, 0xcac1, 0xcb81, 0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841,\
        0xd801, 0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40,\
        0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01, 0x1dc0, 0x1c80, 0xdc41,\
        0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680, 0xd641,\
        0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081, 0x1040,\
        0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240,\
        0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441,\
        0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41,\
        0xfa01, 0x3ac0, 0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840,\
        0x2800, 0xe8c1, 0xe981, 0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41,\
        0xee01, 0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40,\
        0xe401, 0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640,\
        0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0, 0x2080, 0xe041,\
        0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281, 0x6240,\
        0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480, 0xa441,\
        0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41,\
        0xaa01, 0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840,\
        0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41,\
        0xbe01, 0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40,\
        0xb401, 0x74c0, 0x7580, 0xb541, 0x7700, 0xb7c1, 0xb681, 0x7640,\
        0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080, 0xb041,\
        0x5000, 0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0, 0x5280, 0x9241,\
        0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440,\
        0x9c01, 0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40,\
        0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841,\
        0x8801, 0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40,\
        0x4e00, 0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41,\
        0x4400, 0x84c1, 0x8581, 0x4540, 0x8701, 0x47c0, 0x4680, 0x8641,\
        0x8201, 0x42c0, 0x4380, 0x8341, 0x4100, 0x81c1, 0x8081, 0x4040 \
};

/**
 * @brief Funcao para calcular CRC16 do protocolo ABNT
 *
 * Calcula o CRC16 de uma sequencia de bytes passada como parametro, baseado
 * na tabela pre-calculada para o polinomio CRC16 (0xA001)
 *
 * @param[in] buffer referencia para o vetor de bytes com o conteudo a ser lido
 * @param[in] len tamanho de buffer
 * @return o CRC16 do buffer
*/
uint16_t calc_crc_16(uint8_t *buffer, uint16_t len)
{
    uint16_t crc = 0;
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        crc = (crc >> 8) ^ \
            crc16_table[(crc ^ ((uint16_t)(buffer[i]))) & 0xff];
    }
    return crc;
}

#endif //TEST_SERIALNATIVA
