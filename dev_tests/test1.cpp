#include "test.h"

#ifdef TEST1CONFIG
/*
  First Configuration
  This sketch demonstrates the usage of MKR WAN 1300 LoRa module.
  This example code is in the public domain.
*/

#include <MKRWAN.h>

LoRaModem modem;

// Uncomment if using the Murata chip as a module
//LoRaModem modem(Serial1);

String appEui;
String appKey;
String devAddr;
String nwkSKey;
String appSKey;

String errCode;

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    //Serial1.begin(9600);
    while (!Serial)
        ;
    Serial.println("Welcome to MKRWAN1300 first configuration sketch");
    Serial.println("Register to your favourite LoRa network and we "
                   "are ready to go!");
    // change this to your regional band (eg. US915, AS923, ...)
    if (!modem.begin(AU915))
    {
        Serial.println("Failed to start module");
        while (1)
            ;
    }
    Serial.print("Your module version is: ");
    Serial.println(modem.version());
    Serial.print("Your device EUI is: ");
    Serial.println(modem.deviceEUI());

    int mode = 1;
    #if 0
    while (mode != 1 && mode != 2)
    {
        Serial.println("Are you connecting via OTAA (1) or ABP (2)?");
        while (!Serial.available())
            ;
        mode = Serial.readStringUntil('\n').toInt();
    }
    #endif

    int connected;
    if (mode == 1)
    {
#if 0
        Serial.println("Enter your APP EUI");
        while (!Serial.available())
            ;
        appEui = Serial.readStringUntil('\n');

        Serial.println("Enter your APP KEY");
        while (!Serial.available())
            ;
        appKey = Serial.readStringUntil('\n');

        appKey.trim();
        appEui.trim();
#endif
        appEui = SECRET_APP_EUI;
        appKey = SECRET_APP_KEY;

        //-------- DEBUG ----------
        Serial.print("App EUI is: " + appEui + "\n");
        Serial.print("App key is: " + appKey + "\n");
        //-------- DEBUG ----------

        if (modem.publicNetwork(false))
            Serial .println("Private !");

        Serial.println("Before connect!");
        connected = modem.joinOTAA(appEui, appKey);
        Serial.print("Connected: ");
        Serial.println(connected);
    }
    else if (mode == 2)
    {

        Serial.println("Enter your Device Address");
        while (!Serial.available())
            ;
        devAddr = Serial.readStringUntil('\n');

        Serial.println("Enter your NWS KEY");
        while (!Serial.available())
            ;
        nwkSKey = Serial.readStringUntil('\n');

        Serial.println("Enter your APP SKEY");
        while (!Serial.available())
            ;
        appSKey = Serial.readStringUntil('\n');

        devAddr.trim();
        nwkSKey.trim();
        appSKey.trim();

        //-------- DEBUG ----------
        Serial.print("Dev Addr is : " + devAddr + "\n");
        Serial.print("Nwk S key is: " + nwkSKey + "\n");
        Serial.print("App S key is: " + appSKey + "\n");
        //-------- DEBUG ----------

        Serial.println("Before connect!");
        connected = modem.joinABP(devAddr, nwkSKey, appSKey);
        Serial.print("Connected: ");
        Serial.println(connected);
    }

    if (modem.dataRate(0))
        Serial.println("Set DataRate OK!");
    else
        Serial.println("Set DataRate ERROR");
    Serial.println(String(modem.getDataRate()));

    if (!connected)
    {
        Serial.println("Something went wrong; are you indoor? Move near "
                       "a window and retry");
        while (1)
        {
        }
    }

    delay(5000);

    int err;
    modem.setPort(3);
    modem.beginPacket();
    //modem.print("HeLoRA world!");
    modem.print("LR123");
    err = modem.endPacket(true);
    errCode = err;
    if (err > 0)
    {
        Serial.println("Message sent correctly!");
    }
    else
    {
        Serial.print("Error sending message. Err code: " + errCode + "\n");
        //Serial.println("Error sending message :(");
    }
}

void loop()
{
    while (modem.available())
    {
        Serial.write(modem.read());
    }
    modem.poll();
}
#endif //TEST1CONFIG

//////////////////////////////////////////////////////////////////////

#ifdef SEND_RECEIVE
/*
  Lora Send And Receive
  This sketch demonstrates how to send and receive data with the
  MKR WAN 1300 LoRa module.
  This example code is in the public domain.
*/

#include <MKRWAN.h>

LoRaModem modem;

// Uncomment if using the Murata chip as a module
//LoRaModem modem(Serial1);

String appEui = SECRET_APP_EUI;
String appKey = SECRET_APP_KEY;

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    //Serial1.begin(9600);
    while (!Serial)
        ;
    // change this to your regional band (eg. US915, AS923, ...)
    if (!modem.begin(AU915))
    {
        Serial.println("Failed to start module");
        while (1)
        {
        }
    };
    Serial.print("Your module version is: ");
    Serial.println(modem.version());
    Serial.print("Your device EUI is: ");
    Serial.println(modem.deviceEUI());

    //-------- DEBUG ----------
    Serial.print("App EUI is: " + appEui + "\n");
    Serial.print("App key is: " + appKey + "\n");
    //-------- DEBUG ----------

    int connected = modem.joinOTAA(appEui, appKey);
    if (!connected)
    {
        Serial.println("Something went wrong; are you indoor? Move "
                       "near a window and retry");
        while (1)
        {
        }
    }

    if (modem.dutyCycle(false))
    {
        Serial.println("Duty cycle desligado com sucesso!");
    }

    // Set poll interval to 60 secs.
    modem.minPollInterval(60);
    // NOTE: independently by this setting the modem will
    // not allow to send more than one message every 2 minutes,
    // this is enforced by firmware and can not be changed.
}

void loop()
{
    Serial.println();
    Serial.println("Enter a message to send to network");
    Serial.println("(make sure that end-of-line 'NL' is enabled)");

    while (!Serial.available())
        ;
    String msg = Serial.readStringUntil('\n');

    Serial.println();
    Serial.print("Sending: " + msg + " - ");
    for (unsigned int i = 0; i < msg.length(); i++)
    {
        Serial.print(msg[i] >> 4, HEX);
        Serial.print(msg[i] & 0xF, HEX);
        Serial.print(" ");
    }
    Serial.println();

    int err;
    modem.beginPacket();
    modem.print(msg);
    err = modem.endPacket(true);
    if (err > 0)
    {
        Serial.println("Message sent correctly!");
    }
    else
    {
        Serial.println("Error sending message :(");
        Serial.println("(you may send a limited amount of messages per minute,"
                       " depending on the signal strength");
        Serial.println("it may vary from 1 message every couple of seconds to "
                       "1 message every minute)");
    }
    delay(1000);
    if (!modem.available())
    {
        Serial.println("No downlink message received at this time.");
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
        Serial.print(rcv[j] >> 4, HEX);
        Serial.print(rcv[j] & 0xF, HEX);
        Serial.print(" ");
    }
    Serial.println();
}
#endif //SEND_RECEIVE

#ifdef OLD_TEST_FTDI

#include <cdcftdi.h>
#include <usbhub.h>
#include "abnt.h"
#include "otica_tfdi.h"

#include "pgmstrings.h"

// On SAMD boards where the native USB port is also the serial console, use
// Serial1 for the serial console. This applies to all SAMD boards except for
// Arduino Zero and M0 boards.
#if (USB_VID == 0x2341 && defined(ARDUINO_SAMD_ZERO)) || \
    (USB_VID == 0x2a03 && defined(ARDUINO_SAM_ZERO))
#define SerialDebug SERIAL_PORT_MONITOR
#else
#define SerialDebug Serial1
#endif

//uint32_t send_dataFTDI(uint32_t datasize, uint8_t *databuf);
uint16_t calc_crc_16(uint8_t *buffer, uint16_t len);

bool sendNow = false;
bool respOk = false;
uint8_t ContENQ = 0;

abnt_resp_generic_t bufABNTrecv; //ABNT rcvd buffer is fixed 258 bytes long
volatile uint8_t *pbufrec = NULL; //pointer to receiving data
uint16_t bytesRecABNT;

bool send = false;
uint8_t cont = 0;

void setup()
{
    SerialDebug.begin(9600);
    SerialDebug.println("Start");

    if (UsbH.Init())
        SerialDebug.println("USB host did not start.");

    delay(200);
    // randomSeed(analogRead(1));
}

void loop()
{
    //uint8_t bufABNTsend[66]; //ABNT send buffer is fixed 66 bytes long

    UsbH.Task();

    if (UsbH.getUsbTaskState() == USB_STATE_RUNNING)
    {
        uint32_t rcode = 0;
        uint8_t bufsend[] = CMD_14;

        if (sendNow)
        {
            rcode = send_dataFTDI(LEN_CMD, bufsend);
            sendNow = false;
            respOk = false;
            //pbufrec = bufABNTrecv;
            ContENQ = 0;
            SerialDebug.println("Preparing to send cmd!");
            if (rcode)
            {
                SerialDebug.println("\t ->>> ERROR SENDING !!! <<<-");
                ErrorMessage<uint8_t>(PSTR("SndData"), rcode);
            }
            return;
        }

        delay(80);
        SerialDebug.println("\t Delay 80m");

        bytesRecABNT = abnt_recv_msg_medidor(&bufABNTrecv);
        if (bytesRecABNT == 1)
        {
            if (*((uint8_t *)&bufABNTrecv) == ABNT_ENQ) //ENQ
            {
                SerialDebug.println("\n\t\tContENQ = " + String(ContENQ));
                if (++ContENQ > 9)
                {
                    sendNow = true;
                    ContENQ = 0;
                }
            }
        }

        SerialDebug.println("ABNT rec: " + String(bytesRecABNT));

        if (bytesRecABNT == LEN_RES)
        {
            if (calc_crc_16((uint8_t *)&bufABNTrecv, LEN_RES))
                SerialDebug.println("CRC ruim");
            else
            {
                SerialDebug.println("CRC OK!");
                respOk = true;
                SerialDebug.println("Dia: " + String((*(abnt_resp_le_grand_instant_t *)&bufABNTrecv).dia, HEX) +
                                    "\tMes: " + String((*(abnt_resp_le_grand_instant_t *)&bufABNTrecv).mes, HEX) +
                                    "\tAno: " + String((*(abnt_resp_le_grand_instant_t *)&bufABNTrecv).ano, HEX) +
                                    "\t\tHora: " + String((*(abnt_resp_le_grand_instant_t *)&bufABNTrecv).hora, HEX) +
                                    "\tMinuto: " + String((*(abnt_resp_le_grand_instant_t *)&bufABNTrecv).min, HEX) +
                                    "\tSegundo: " + String((*(abnt_resp_le_grand_instant_t *)&bufABNTrecv).seg, HEX) +
                                    "\t\tVa: " + String((*(abnt_resp_le_grand_instant_t *)&bufABNTrecv).tensao_a) +
                                    "\tVbc: " + String((*(abnt_resp_le_grand_instant_t *)&bufABNTrecv).tensao_bc));
            }
        }

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
    else
    {
        SerialDebug.println("Not running...");
        delay(300);
    }

}

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

#endif //OLD_TEST_FTDI

#ifdef TEST_FTDI
#include <cdcftdi.h>
#include <usbhub.h>

#include "abnt.h"
#include "otica_tfdi.h"
#include "pgmstrings.h"

#include <MKRWAN.h>

// On SAMD boards where the native USB port is also the serial console, use
// Serial1 for the serial console. This applies to all SAMD boards except for
// Arduino Zero and M0 boards.
#if (USB_VID == 0x2341 && defined(ARDUINO_SAMD_ZERO)) || \
    (USB_VID == 0x2a03 && defined(ARDUINO_SAM_ZERO))
#define SerialDebug SERIAL_PORT_MONITOR
#else
#define SerialDebug Serial1
#endif

LoRaModem modem;

String appEui = SECRET_APP_EUI;
String appKey = SECRET_APP_KEY;

bool sendNow = false;
bool respOk = false;
uint8_t ContENQ = 0;

abnt_resp_generic_t bufABNTrecv; //ABNT rcvd buffer is fixed 258 bytes long
volatile uint8_t *pbufrec = NULL; //pointer to receiving data
uint16_t bytesRecABNT;

bool send = false;
uint8_t cont = 0;

bool LoRaOK = false;
bool waitResp = false;
uint32_t tLastSend;

uint8_t LoRaBuff[12]; //msg a ser enviada via LoRa
uint8_t *loRaBuffPt;
uint8_t loRaBuffSize;

//uint32_t send_dataFTDI(uint32_t datasize, uint8_t *databuf);
uint16_t calc_crc_16(uint8_t *buffer, uint16_t len);

int8_t loRaSetup(void);
void loRaLoop (uint8_t *msg, uint8_t tam);

void setup()
{
    SerialDebug.begin(9600);
    SerialDebug.println("Start");

    //delay(200);
    // randomSeed(analogRead(1));

    if (loRaSetup() < 0){
        SerialDebug.println ("LoRa Error!");
        LoRaOK = false;
    }
    else
        LoRaOK = true;

    if (UsbH.Init())
        SerialDebug.println("USB host did not start.");
}

void loop()
{
    float Va = 0.0;
    float Vb = 0.0;
    float Vc = 0.0;

    UsbH.Task();

    if (UsbH.getUsbTaskState() == USB_STATE_RUNNING)
    {
        uint32_t rcode = 0;
        uint8_t bufsend[] = CMD_14;

        if (sendNow)
        {
            rcode = send_dataFTDI(LEN_CMD, bufsend);
            sendNow = false;
            respOk = false;
            //pbufrec = bufABNTrecv;
            ContENQ = 0;
            SerialDebug.println("Preparing to send cmd!");
            if (rcode)
            {
                SerialDebug.println("\t ->>> ERROR SENDING !!! <<<-");
                ErrorMessage<uint8_t>(PSTR("SndData"), rcode);
            }
            return;
        }

        delay(80);
        SerialDebug.println("\t Delay 80m");

        bytesRecABNT = abnt_recv_msg_medidor(&bufABNTrecv);
        if (bytesRecABNT == 1)
        {
            if (*((uint8_t *)&bufABNTrecv) == ABNT_ENQ) //ENQ
            {
                SerialDebug.println("\n\t\tContENQ = " + String(ContENQ));
                if (++ContENQ > 9)
                {
                    sendNow = true;
                    ContENQ = 0;
                }
            }
        }

        SerialDebug.println("ABNT rec: " + String(bytesRecABNT));

        if (bytesRecABNT == LEN_RES)
        {
            if (calc_crc_16((uint8_t *)&bufABNTrecv, LEN_RES))
                SerialDebug.println("CRC ruim");
            else
            {
                SerialDebug.println("CRC OK!");
                respOk = true;
                Va = (*(abnt_resp_le_grand_instant_t *)&bufABNTrecv).tensao_a;
                Vb = (*(abnt_resp_le_grand_instant_t *)&bufABNTrecv).tensao_b;
                Vc = (*(abnt_resp_le_grand_instant_t *)&bufABNTrecv).tensao_c;

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

                SerialDebug.println("Dia: " + String((*(abnt_resp_le_grand_instant_t *)&bufABNTrecv).dia, HEX) +
                                    "\tMes: " + String((*(abnt_resp_le_grand_instant_t *)&bufABNTrecv).mes, HEX) +
                                    "\tAno: " + String((*(abnt_resp_le_grand_instant_t *)&bufABNTrecv).ano, HEX) +
                                    "\t\tHora: " + String((*(abnt_resp_le_grand_instant_t *)&bufABNTrecv).hora, HEX) +
                                    "\tMinuto: " + String((*(abnt_resp_le_grand_instant_t *)&bufABNTrecv).min, HEX) +
                                    "\tSegundo: " + String((*(abnt_resp_le_grand_instant_t *)&bufABNTrecv).seg, HEX) +
                                    "\t\tVa: " + String((*(abnt_resp_le_grand_instant_t *)&bufABNTrecv).tensao_a) +
                                    "\tVbc: " + String((*(abnt_resp_le_grand_instant_t *)&bufABNTrecv).tensao_bc));
            }
        }

    }
    else
    {
        SerialDebug.println("Not running...");
        delay(300);
    }

    if (LoRaOK)
    {
        loRaLoop(loRaBuffPt, loRaBuffSize);
        SerialDebug.println("LoRa connected...");
    }
        else
        SerialDebug.println("LoRa DISConnected...");
}

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

int8_t loRaSetup(void)
{
    // change this to your regional band (eg. US915, AS923, ...)
    if (!modem.begin(AU915))
    {
        SerialDebug.println("Failed to start module");
        return -1;
    }
    SerialDebug.print("Your module version is: ");
    SerialDebug.println(modem.version());
    SerialDebug.print("Your device EUI is: ");
    SerialDebug.println(modem.deviceEUI());

    //-------- DEBUG ----------
    SerialDebug.println("App EUI is: " + appEui);
    SerialDebug.println("App key is: " + appKey);
    //-------- DEBUG ----------

    if (modem.publicNetwork(false))
        SerialDebug.println("Private !");

    int connected = modem.joinOTAA(appEui, appKey);
    if (!connected)
    {
        SerialDebug.println("Something went wrong; are you indoor? Move "
                       "near a window and retry");
        return -1;
    }

    if (modem.dutyCycle(false))
    {
        SerialDebug.println("Duty cycle desligado com sucesso!");
    }

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

void loRaLoop (uint8_t *msg, uint8_t tam)
{
    int err;

    if ((millis() - tLastSend) > MIN_LORA_INTERVAL)
    {
        tLastSend = millis();
        waitResp = true;
        SerialDebug.println("Sending "+String(tam)+" bytes\n [0] = "+String(msg[0]));
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
    //delay(1000);

    if (waitResp && ((millis() - tLastSend) > LORA_RESP_INTERVAL))
    {
        waitResp = false;
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

#endif //TEST_FTDI

#ifdef TEST_LORA_SERIALDEBUG
#include <cdcftdi.h>
#include <usbhub.h>

#include "abnt.h"
#include "otica_tfdi.h"
#include "pgmstrings.h"
#include <MKRWAN.h>

#define SerialDebug Serial1

LoRaModem modem;
bool loRaOk = false;

static const String appEui = SECRET_APP_EUI;
static const String appKey = SECRET_APP_KEY;

void setup()
{
    SerialDebug.begin(9600);
    SerialDebug.println("Start");

    // change this to your regional band (eg. US915, AS923, ...)
    if (!modem.begin(AU915))
    {
        SerialDebug.println("Failed to start module");
    }

    SerialDebug.print("Your module version is: ");
    SerialDebug.println(modem.version());
    SerialDebug.print("Your device EUI is: ");
    SerialDebug.println(modem.deviceEUI());

    //-------- DEBUG ----------
    SerialDebug.print("App EUI is: " + appEui + "\n");
    SerialDebug.print("App key is: " + appKey + "\n");
    //-------- DEBUG ----------

    if (modem.publicNetwork(false))
        SerialDebug.println("Private !");

    int connected = modem.joinOTAA(appEui, appKey);
    //int connected = modem.joinOTAA("70B3D57ED0012777", "AECBAEA6EB06A316413E7B2244DFBF00");
    if (!connected)
    {
        SerialDebug.println("Something went wrong; are you indoor? Move "
                            "near a window and retry");
    }
    else
        loRaOk = true;

    if (UsbH.Init())
        SerialDebug.println("USB host did not start.");
}
void loop()
{
    int err;
    uint8_t msg[10];

    delay (1000);
    SerialDebug.println("alive... ");

    if (!loRaOk)
        SerialDebug.println("Sem LoRa!!!");

    else
    {

        uint8_t msg[12]; // = {0x01, 0x02, 0x00, 0x00};
        float testFloat[3]= {124.0, 124.0, 124.0};
        //CAYENNE: ch = 01, type = 02 (analog input - signed 0.01), valor, valor

        memcpy((uint8_t *)msg, (uint8_t *)testFloat, sizeof(testFloat));

        modem.beginPacket();
        //modem.print(msg);
        modem.write(msg, sizeof(msg));
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

#endif //TEST_LORA_SERIALDEBUG

#ifdef TESTE_RS232
#include "util.h"
#include <Arduino.h>
#include <stdint.h>


#include "abnt.h"
#include "serial_nativa.h"
#include <MKRWAN.h>

#define SerialDebug SerialUSB

//Variaveis relativas ao modulo ABNT
uint32_t tLastABNTSend;


void setup()
{
    Serial1.begin(ABNT_BAUD);
    Serial1.setTimeout(250);
    SerialDebug.begin(9600);
    while (!SerialDebug) ;
    SerialDebug.println("Start");

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(SERIAL_RTS, OUTPUT);
    digitalWrite(SERIAL_RTS, LOW); //Low TTL = +5,5V RS232
    digitalWrite(LED_BUILTIN, LOW);
    abntInit();
    tLastABNTSend = millis();

    for (int k=0;k<5;k++)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(150);
        digitalWrite(LED_BUILTIN, LOW);
        delay(150);
    }

    //////////////
    portaSerial.interface = SERIAL_RS232;
    portaSerial.recMsgMedidor = recv_dataSerial;
    portaSerial.sendCmdMedidor = send_dataSerial;
    /////////////////

    delay(500);
}

void loop()
{
    int16_t nBytes;
    uint8_t buff1[64];
    memset(buff1, 0, sizeof(buff1));

    nBytes = Serial1.available();
    if (nBytes)
    {
        SerialDebug.println("nB: " + String(nBytes));
        nBytes = recvMsgMedidorABNT((abnt_resp_generic_t *)buff1);
        // nBytes = 64;
        // nBytes = portaSerial.recMsgMedidor(&nBytes, buff1); //recv_dataSerial(&nBytes, buff1);
        ///////  --- Testar com recvDataSerial
    }
    if (nBytes > 0)
    {
        SerialDebug.println("rec: " + String(nBytes));
        for (int i = 0; i < nBytes; ++i)
            SerialDebug.println(buff1[i], HEX);
    }
    else
        //Serial1.readBytes(buff1, 200);
        SerialDebug.println(". " + String(nBytes));
    delay(1200);
    return;
}

#endif // TESTE_RS232
