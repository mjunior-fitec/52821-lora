/** @file fimafim.cpp
* @brief Teste fim a fim, versao 1.0 para producao
*
* Aquivo com a implementacao de um teste fim a fim para ser utilizado
* como versao inicial de producao, ao fim da montagem das placas. Este
* teste consiste em enviar um comando 14 (Leitura de grandezas instantaneas)
* para o medidor via USB/serial ou RS-232 nativa, interpretar os resultados
* recebidos e enviar a tabela 3 do protocolo ANSI-FITec com os dados de tensao
* e corrente lidos via LoRa apos uma conexao OTAA. Possui tambem recepcao de
* comandos via serial para comissionamento (programacao de APPEUI e APPKey)
* com o armazenamento em Flash.
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
#ifdef TESTE_FIMAFIM

#include "abnt.h"
#include "otica_tfdi.h"
#include "serial_nativa.h"
#include "pgmstrings.h"
#include "ansi_fitec.h"
#include "wiring_private.h"

#include <cdcftdi.h>
#include <usbhub.h>
#include <RTCZero.h>
#include <FlashStorage.h>
//#include <MKRWAN.h>
#include <time.h>

#define SerialDebug Serial3
uint8_t calcSum(uint8_t *buffer, size_t bufSize);
int8_t localloRaSetup(void);
void localloRaLoop(uint8_t *msg, uint8_t tam);
void initTabelasANSI(void);
bool protocoloComissionamento(void);

//Habilitação de uma nova UART, Serial3, nos pinios 0, PA22 (Tx) e 1, PA23 (Rx)
Uart Serial3(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM3_Handler()
{
    Serial3.IrqHandler();
}

// Valores assumidos caso nao receba comando de comissionamento
char localAppEui[17] = SECRET_APP_EUI;
char localAppKey[33] = SECRET_APP_KEY;

static const String localDeviceAddr = ABP_DEVADDR;
static const String localNetworkSKey = ABP_NWKSKEY;
static const String localAppSessionKey = ABP_APPSKEY;

bool sendNow = false;
bool respOk = false;
uint8_t ContENQ = 0;

abnt_resp_generic_t bufABNTrecv;
uint16_t bytesRecABNT;
uint32_t tLastABNTSend;

RTCZero rtc;

bool send = false;
uint8_t cont = 0;

bool LoRaOK = false;
bool waitResp = false;
uint32_t tLastSend;

uint8_t localLoRaBuff[51]; //msg a ser enviada via LoRa
uint8_t *localloRaBuffPt;
uint8_t localloRaBuffSize;

secret_keys_t localKeys;
FlashStorage(savedKeys, secret_keys_t);

uint8_t nextCmd;

tabela01_t localAnsi_tab1;
tabela02_t localAnsi_tab2;
tabela03_t localAnsi_tab3;
tabela04_t localAnsi_tab4;
tabela05_t localAnsi_tab5;
tabela06_t localAnsi_tab6;
tabela07_t localAnsi_tab7;
tabela08_t localAnsi_tab8;
tabela09_t localAnsi_tab9;

tabelasAnsi_t dadosTabela[10];
uint8_t tabelaAtual = 0;

void setup()
{
    memcpy(localKeys.appeui, (uint8_t *)&localAppEui, sizeof(localKeys.appeui));
    memcpy(localKeys.appkey, (uint8_t *)&localAppKey, sizeof(localKeys.appkey));


    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(SERIAL_RTS, OUTPUT);
    digitalWrite(SERIAL_RTS, LOW); //Low TTL = +5,5V RS232
    digitalWrite(LED_BUILTIN, LOW);

    Serial1.begin(ABNT_BAUD);
    Serial1.setTimeout(250);

    SerialDebug.begin(9600);
    pinPeripheral(1, PIO_SERCOM);
    pinPeripheral(0, PIO_SERCOM);

    localKeys = savedKeys.read();

    SerialDebug.println("Start...");
    SerialDebug.println("Aguardando protocolo comissionamento.\n\n");

    if (protocoloComissionamento())
    {
        savedKeys.write(localKeys);
        SerialDebug.println("Chave recebida!");
    }

    delay(300);
    SerialDebug.println("local secrets:");
    SerialDebug.println("EUI: " + String((char *)(localKeys.appeui)));
    SerialDebug.println("Key: " + String((char *)(localKeys.appkey)));
    delay(200);

    # ifdef SEMLORA
    SerialDebug.println("Executando SEM LoRa!");
    LoRaOK = false;
    # else
    if (localloRaSetup() < 0)
    {
        piscaLed(4, 350, 100);
        SerialDebug.println("LoRa NAO conectado!!!");
        LoRaOK = false;
    }
    else
    {
        SerialDebug.println("LoRa OTAA conectado!");
        LoRaOK = true;
    }
    # endif //SEMLORA

    if (UsbH.Init())
    {
        delay(500);
        piscaLed(3, 100, 100);
    }

    initTabelasANSI();

    nextCmd = 0;
    delay(1200);
    abntInit();
    tLastABNTSend = millis();
}

void loop()
{
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
        if (respABNTRecebida)
        {
            nextCmd = 0;
            respLeGrandInst = (abnt_resp_le_grand_instant_t *)pBuffABNTrecv;
            memset(&localAnsi_tab3, 0, SIZE_TABELA3);

            localAnsi_tab3.tensao_a = (respLeGrandInst->tensao_a)*(1<<14);
            localAnsi_tab3.tensao_b = (respLeGrandInst->tensao_b)*(1<<14);
            localAnsi_tab3.tensao_c = (respLeGrandInst->tensao_c)*(1<<14);
            localAnsi_tab3.corrente_a = (respLeGrandInst->corrente_a)*(1<<14);
            localAnsi_tab3.corrente_b = (respLeGrandInst->corrente_b)*(1<<14);
            localAnsi_tab3.corrente_c = (respLeGrandInst->corrente_c)*(1<<14);
            if (LoRaOK)
            {
                localLoRaBuff[0] = 3; //identificador da tabela
                localLoRaBuff[1] = 0;
                localLoRaBuff[2] = SIZE_TABELA3; //tamanho dos dados
                localLoRaBuff[3] = 0;
                localloRaBuffSize = 4;
                memcpy(localloRaBuffPt + localloRaBuffSize, (uint8_t *)&localAnsi_tab3, SIZE_TABELA3);
                localloRaBuffSize += SIZE_TABELA3;
                localLoRaBuff[localloRaBuffSize] = calcSum(localloRaBuffPt, localloRaBuffSize);
                localloRaBuffSize++;
            }
            abntInit();
        }
    }
    else
    {
        piscaLed(2, 350, 350);
        delay(300);
    }
    if (LoRaOK)
    {
        if ((millis() - tLastSend) > MIN_LORA_INTERVAL)
        {
            localLoRaBuff[0] = dadosTabela[tabelaAtual].id;
            localLoRaBuff[1] = 0;
            localLoRaBuff[2] = dadosTabela[tabelaAtual].size; //tamanho dos dados
            localLoRaBuff[3] = 0;
            localloRaBuffSize = 4;
            memcpy(localloRaBuffPt + localloRaBuffSize, (uint8_t *)dadosTabela[tabelaAtual].tabela, dadosTabela[tabelaAtual].size);
            localloRaBuffSize += dadosTabela[tabelaAtual].size;
            localLoRaBuff[localloRaBuffSize] = calcSum(localloRaBuffPt+4, localLoRaBuff[2]);
            localloRaBuffSize++;
        }
        localloRaLoop(localloRaBuffPt, localloRaBuffSize);
    }
    else
        SerialDebug.println("SEM LoRa.");

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
    }

} //loop()

int8_t localloRaSetup(void)
{
    if (!modem.begin(AU915))
    {
        return -1;
    }
    delay(500);
    SerialDebug.print("Device EUI: ");
    SerialDebug.println(modem.deviceEUI());

    if (!modem.publicNetwork(false))
    {
        piscaLed(3, 300, 100);
    }

    if (modem.configureClass(CLASS_C))
        SerialDebug.println("Muda classe OK");
    else
        SerialDebug.println("Erro ao mudar de classe");

    //int connected = modem.joinOTAA(localKeys.appeui, localKeys.appkey);
    int connected = modem.joinABP (localDeviceAddr, localNetworkSKey, localAppSessionKey); //   ABP_DEVADDR, ABP_NWKSKEY, ABP_APPSKEY);

    if (!connected)
        return -1;

    modem.dutyCycle(false);

    localloRaBuffPt = localLoRaBuff;
    localloRaBuffSize = 0;
    tLastSend = millis();
    modem.minPollInterval(60);
    return 0;
}

void localloRaLoop(uint8_t *msg, uint8_t tam)
{
    int err;

    if ((millis() - tLastSend) > MIN_LORA_INTERVAL)
    {
        tLastSend = millis();
        waitResp = true;
        modem.beginPacket();
        modem.write(msg, tam);
        err = modem.endPacket(true);

        if (err > 0)
            piscaLed(3, 350, 150);
        else
            piscaLed(1, 250, 0);

        SerialDebug.println("Envio LoRa !!! \tTab: " + String(tabelaAtual));
        if (++tabelaAtual > 8)
            tabelaAtual = 0;
    }

    if (waitResp && ((millis() - tLastSend) > LORA_RESP_INTERVAL))
    {
        waitResp = false;
        if (!modem.available())
        {
            piscaLed(2, 500, 200);
            return;
        }
    }
}

void initTabelasANSI(void)
{
    struct tm datahora_tm;
    time_t datahora;

    datahora_tm.tm_hour = 11;
    datahora_tm.tm_min = 45;
    datahora_tm.tm_sec = 0;
    datahora_tm.tm_mday = 21;
    datahora_tm.tm_mon = 8;    //Month - 1
    datahora_tm.tm_year = 114; //Year - 1900
    datahora = mktime(&datahora_tm);
    localAnsi_tab1.timestamp = datahora;
    localAnsi_tab1.posto_horario = 2;
    localAnsi_tab1.enAtivaTotDireto = 100000000;
    localAnsi_tab1.enAtivaPontaDireto = 85000000;
    localAnsi_tab1.enAtivaForaDireto = 10000000;
    localAnsi_tab1.enAtivaIntermDireto = 225000;
    localAnsi_tab1.enAtivaReservDireto = 4775000;
    localAnsi_tab1.enReativaTotDireto = 50000000;
    localAnsi_tab1.demAtivaPonta = 25000;
    localAnsi_tab1.demAtivaFora = 5000;
    localAnsi_tab1.demAtivaInterm = 3500;
    localAnsi_tab1.demAtivaReserv = 1500;

    localAnsi_tab2.enAtivaTotReverso =   1000000;
    localAnsi_tab2.enAtivaPontaReverso =  400000;
    localAnsi_tab2.enAtivaIntermReverso = 200000;
    localAnsi_tab2.enAtivaForaReverso =   300000;
    localAnsi_tab2.enAtivaReservReverso = 100000;

    localAnsi_tab3.enReativaCapTot = 15000000;
    localAnsi_tab3.tensao_a = 110.11*(1<<14);
    localAnsi_tab3.tensao_b = 127.27*(1<<14);
    localAnsi_tab3.tensao_c = 220.22*(1<<14);
    localAnsi_tab3.corrente_a = 33.33*(1<<14);
    localAnsi_tab3.corrente_b = 22.22*(1<<14);
    localAnsi_tab3.corrente_c = 11.11*(1<<14);
    localAnsi_tab3.drp = 30000;
    localAnsi_tab3.drc = 1470;

    localAnsi_tab4.dataHoraIniFalta[0] = datahora + 2000000;
    localAnsi_tab4.dataHoraIniFalta[1] = datahora + 3500000;
    localAnsi_tab4.dataHoraIniFalta[2] = datahora + 5000000;
    localAnsi_tab4.dataHoraIniFalta[3] = datahora + 6000000;
    localAnsi_tab4.dataHoraIniFalta[4] = datahora + 9900000;
    localAnsi_tab4.dataHoraFimFalta[0] = datahora + 2007000;
    localAnsi_tab4.dataHoraFimFalta[1] = datahora + 3503000;
    localAnsi_tab4.dataHoraFimFalta[2] = datahora + 5002500;
    localAnsi_tab4.dataHoraFimFalta[3] = datahora + 6006000;
    localAnsi_tab4.dataHoraFimFalta[4] = datahora + 9910000;

    localAnsi_tab5.alarmes.falha_comunicacao = 1;
    localAnsi_tab5.alarmes.abertura_bloco = 0;
    localAnsi_tab5.alarmes.aberura_principal = 0;
    localAnsi_tab5.alarmes.i_reversa = 0;
    localAnsi_tab5.alarmes.temperatura_dev = 0;
    localAnsi_tab5.alarmes.relogio_medidor = 1;
    localAnsi_tab5.enReativaIndPonta =  20000000;
    localAnsi_tab5.enReativaIndInterm = 10000000;
    localAnsi_tab5.enReativaIndFora =   15000000;
    localAnsi_tab5.enReativaIndReserv =  5000000;
    localAnsi_tab5.enReativaCapPonta =  50000000;
    localAnsi_tab5.enReativaCapInterm = 25000000;
    localAnsi_tab5.enReativaCapFora =   30000000;
    localAnsi_tab5.enReativaCapReserv =  2000000;

    localAnsi_tab6.demReativaIndPonta = 15000;
    localAnsi_tab6.demReativaIndFora  =  3500;
    localAnsi_tab6.demReativaIndInterm = 2000;
    localAnsi_tab6.demReativaIndReserv = 1000;
    localAnsi_tab6.demReativaCapPonta = 10000;
    localAnsi_tab6.demReativaCapFora =   2500;
    localAnsi_tab6.demReativaCapInterm = 1000;
    localAnsi_tab6.demReativaCapReserv =  750;

    localAnsi_tab7.postoHorario[PONTA].horaInicio1 = 0x08;
    localAnsi_tab7.postoHorario[PONTA].minutoInicio1 = 0x00;
    localAnsi_tab7.postoHorario[PONTA].horaInicio2 = 0x14;
    localAnsi_tab7.postoHorario[PONTA].minutoInicio2 = 0x30;
    localAnsi_tab7.postoHorario[PONTA].horaInicio3 = 0x18;
    localAnsi_tab7.postoHorario[PONTA].minutoInicio3 = 0x00;
    localAnsi_tab7.postoHorario[PONTA].horaInicio4 = 0x99;
    localAnsi_tab7.postoHorario[PONTA].minutoInicio4 = 0x99;
    localAnsi_tab7.postoHorario[FORA_PONTA].horaInicio1 = 0x10;
    localAnsi_tab7.postoHorario[FORA_PONTA].minutoInicio1 = 0x30;
    localAnsi_tab7.postoHorario[FORA_PONTA].horaInicio2 = 0x16;
    localAnsi_tab7.postoHorario[FORA_PONTA].minutoInicio2 = 0x30;
    localAnsi_tab7.postoHorario[FORA_PONTA].horaInicio3 = 0x20;
    localAnsi_tab7.postoHorario[FORA_PONTA].minutoInicio3 = 0x30;
    localAnsi_tab7.postoHorario[FORA_PONTA].horaInicio4 = 0x99;
    localAnsi_tab7.postoHorario[FORA_PONTA].minutoInicio4 = 0x99;
    localAnsi_tab7.postoHorario[RESERVADO].horaInicio1 = 0x11;
    localAnsi_tab7.postoHorario[RESERVADO].minutoInicio1 = 0x00;
    localAnsi_tab7.postoHorario[RESERVADO].horaInicio2 = 0x12;
    localAnsi_tab7.postoHorario[RESERVADO].minutoInicio2 = 0x30;
    localAnsi_tab7.postoHorario[RESERVADO].horaInicio3 = 0x99;
    localAnsi_tab7.postoHorario[RESERVADO].minutoInicio3 = 0x99;
    localAnsi_tab7.postoHorario[RESERVADO].horaInicio4 = 0x00;
    localAnsi_tab7.postoHorario[RESERVADO].minutoInicio4 = 0x00;
    localAnsi_tab7.postoHorario[INTERMEDIARIO].horaInicio1 = 0x11;
    localAnsi_tab7.postoHorario[INTERMEDIARIO].minutoInicio1 = 0x30;
    localAnsi_tab7.postoHorario[INTERMEDIARIO].horaInicio2 = 0x16;
    localAnsi_tab7.postoHorario[INTERMEDIARIO].minutoInicio2 = 0x30;
    localAnsi_tab7.postoHorario[INTERMEDIARIO].horaInicio3 = 0x99;
    localAnsi_tab7.postoHorario[INTERMEDIARIO].minutoInicio3 = 0x99;
    localAnsi_tab7.postoHorario[INTERMEDIARIO].horaInicio4 = 0x99;
    localAnsi_tab7.postoHorario[INTERMEDIARIO].minutoInicio4 = 0x99;
    localAnsi_tab7.horVerao.ativar = 1;
    localAnsi_tab7.horVerao.diaFimHorInverno = 0x20;
    localAnsi_tab7.horVerao.mesFimHorInverno = 0x10;
    localAnsi_tab7.horVerao.diaFimHorVerao = 0x17;
    localAnsi_tab7.horVerao.mesFimHorVerao = 0x02;

    localAnsi_tab8.feriados[0].dia = 0x05;
    localAnsi_tab8.feriados[0].mes = 0x03;
    localAnsi_tab8.feriados[0].ano = 0x19;
    localAnsi_tab8.feriados[1].dia = 0x19;
    localAnsi_tab8.feriados[1].mes = 0x04;
    localAnsi_tab8.feriados[1].ano = 0x19;
    localAnsi_tab8.feriados[2].dia = 0x20;
    localAnsi_tab8.feriados[2].mes = 0x06;
    localAnsi_tab8.feriados[2].ano = 0x19;
    for (int fer = 3; fer < 15; fer++)
    {
        localAnsi_tab8.feriados[fer].dia = 0;
        localAnsi_tab8.feriados[fer].mes = 0;
        localAnsi_tab8.feriados[fer].ano = 0;
    }

    localAnsi_tab9.regAlteracoes[0].codAlteracao = 1;
    localAnsi_tab9.regAlteracoes[0].dataHoraAlteracao = datahora + 59000000;
    localAnsi_tab9.regAlteracoes[0].numSerieLeitor = 0x010203;
    localAnsi_tab9.regAlteracoes[1].codAlteracao = 2;
    localAnsi_tab9.regAlteracoes[1].dataHoraAlteracao = datahora + 79000000;
    localAnsi_tab9.regAlteracoes[1].numSerieLeitor = 0x030201;
    localAnsi_tab9.regAlteracoes[2].codAlteracao = 3;
    localAnsi_tab9.regAlteracoes[2].dataHoraAlteracao = datahora + 99000000;
    localAnsi_tab9.regAlteracoes[2].numSerieLeitor = 0x040506;
    localAnsi_tab9.regAlteracoes[3].codAlteracao = 4;
    localAnsi_tab9.regAlteracoes[3].dataHoraAlteracao = datahora + 119000000;
    localAnsi_tab9.regAlteracoes[3].numSerieLeitor = 0x060504;
    localAnsi_tab9.regAlteracoes[4].codAlteracao = 5;
    localAnsi_tab9.regAlteracoes[4].dataHoraAlteracao = datahora + 159000000;
    localAnsi_tab9.regAlteracoes[4].numSerieLeitor = 0x070809;

    dadosTabela[0].id = 1;
    dadosTabela[0].size = SIZE_TABELA1_COMPLETA;
    dadosTabela[0].tabela = (uint8_t *)&localAnsi_tab1;
    dadosTabela[1].id = 2;
    dadosTabela[1].size = SIZE_TABELA2;
    dadosTabela[1].tabela = (uint8_t *)&localAnsi_tab2;
    dadosTabela[2].id = 3;
    dadosTabela[2].size = SIZE_TABELA3;
    dadosTabela[2].tabela = (uint8_t *)&localAnsi_tab3;
    dadosTabela[3].id = 4;
    dadosTabela[3].size = SIZE_TABELA4;
    dadosTabela[3].tabela = (uint8_t *)&localAnsi_tab4;
    dadosTabela[4].id = 5;
    dadosTabela[4].size = SIZE_TABELA5;
    dadosTabela[4].tabela = (uint8_t *)&localAnsi_tab5;
    dadosTabela[5].id = 6;
    dadosTabela[5].size = SIZE_TABELA6;
    dadosTabela[5].tabela = (uint8_t *)&localAnsi_tab6;
    dadosTabela[6].id = 7;
    dadosTabela[6].size = SIZE_TABELA7;
    dadosTabela[6].tabela = (uint8_t *)&localAnsi_tab7;
    dadosTabela[7].id = 8;
    dadosTabela[7].size = SIZE_TABELA8;
    dadosTabela[7].tabela = (uint8_t *)&localAnsi_tab8;
    dadosTabela[8].id = 9;
    dadosTabela[8].size = SIZE_TABELA9;
    dadosTabela[8].tabela = (uint8_t *)&localAnsi_tab9;
}

/**
* @brief Interpretacao de comandos para definicao de senha de conexao LoRa
*
* Funcao para interpretar comandos que podem ser enviados ao se ligar o end device
* Estes comandos definem os parametros AppEUI e APPKey usados para o join no modo
* OTAA. Estes comandos devem ser enviados como texto (ASCII) nos primeiros segundos
* ao se ligar o end device. O formato dos comandos é:
*   appeui=1234567890123456
*   appkey=12345678901234567890123456789012
*
* @return retrun TRUE se recebeu algum comando
*
************************************************************************/
bool protocoloComissionamento(void)
{
    piscaLed(5, 150, 150);

    uint8_t cmdRec[9], paramRec[33];
    bool recebido = false;

    SerialUSB.begin(9600);
    uint32_t tStart = millis();

    while ((millis() - tStart) < T_WAITUSB)
    {
        if (SerialUSB)
            break;
    }
    if (!SerialUSB)
        return false;

    SerialUSB.println("Envie os comandos AQUI !!!\r\n\nEx.\r\nappeui=[valor]\r\nappkey=[valor]");
    SerialDebug.println("Porta de debug ...\r\n Nao preparada para receber comandos");
    piscaLed(2, 300, 50);
    tStart = millis();
    while ((millis() - tStart) < T_COMANDO)
    {
        if (SerialUSB.available())
        {
            if ('A' == toUpperCase(SerialUSB.peek()))
            {
                if (6 == SerialUSB.readBytesUntil('=', cmdRec, 8))
                {
                    cmdRec[8] = 0;
                    SerialDebug.println(String((char *)cmdRec));
                    if (String((char *)cmdRec) == "appeui")
                    {
                        if (16 == SerialUSB.readBytes((char *)paramRec, 16))
                        {
                            paramRec[16] = 0;
                            memcpy(localKeys.appeui, (uint8_t *)&paramRec, sizeof(localKeys.appeui));
                            piscaLed(2, 100, 50);
                            SerialDebug.println("Rec EUI: " + String((char *)paramRec));
                            tStart = millis();
                            recebido = true;
                        }
                    }
                    else if (String((char *)cmdRec) == "appkey")
                    {
                        if (32 == SerialUSB.readBytes((char *)paramRec, 32))
                        {
                            paramRec[32] = 0;
                            memcpy(localKeys.appkey, (uint8_t *)&paramRec, sizeof(localKeys.appkey));
                            piscaLed(2, 100, 50);
                            SerialDebug.println("Rec KEY: " + String((char *)paramRec));
                            tStart = millis();
                            recebido = true;
                        }
                    }
                    else
                        SerialDebug.println("Comando nao implementado!");
                }
            }
            else
                break;
        }
    }
    piscaLed(3, 1500, 250);
    SerialUSB.println("Tempo esgotado !!!");
    return (recebido);
}

#endif //TESTE_FIMAFIM


#ifdef TESTE_SERIAL3
#include "wiring_private.h"

Uart Serial3(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM3_Handler()
{
    Serial3.IrqHandler();
}
uint8_t i = 0;

void setup()
{
    Serial3.begin(9600);
    pinPeripheral(1, PIO_SERCOM);
    pinPeripheral(0, PIO_SERCOM);

    Serial.begin(9600);
    while (!Serial) ;
    delay (200);
    piscaLed(3, 500, 200);
    Serial3.println("Start...");
    Serial3.flush();
    Serial.println("Start...");
}

void loop()
{
    Serial3.println("teste: " + String(i++));
    while (Serial3.available())
        Serial.print(String(Serial3.read()));

    piscaLed (1, 300, 100);
}

#endif // TESTE_SERIAL3

#ifdef TEST_CONSUMO

#include <Arduino.h>
#include <ArduinoLowPower.h>
#include <MKRWAN.h>
#include <RTCZero.h>

#define SLEEP_PERIOD ( (uint32_t) 10000 )

LoRaModem modem;

void InitIO()
{
    for (int iPin = 0; iPin < 15; iPin++)
    {
        if ((iPin != 11) && (iPin != 12))
            pinMode(iPin, OUTPUT);
    }
}

void alarmEvent()
{
    // RTC alarm wake interrupt callback
    // do nothing
}

void setup()
{
    InitIO();
    LowPower.attachInterruptWakeup(RTC_ALARM_WAKEUP, alarmEvent, CHANGE);
    if (!modem.begin(AU915))
    {
        delay(1000);
        piscaLed(10, 200, 200);
        delay(1000);
        digitalWrite(LED_BUILTIN, HIGH);
        while(1) ;
    }
}

void loop()
{
    piscaLed(3, 300, 200);
    delay(1000);
    piscaLed (3, 400, 250);
    delay(500);
    piscaLed(3, 250, 50);
    LowPower.deepSleep(SLEEP_PERIOD);
}
#endif //TEST_CONSUMO
