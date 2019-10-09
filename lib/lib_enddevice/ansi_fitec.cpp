/** @file ansi_fitec.cpp
* @brief Arquivo com a implementacao do modulo ANSI-FITec para transmissao
* via LoRa.
*
* Este arquivo contem a implementacao do modulo ANSI-FITec, responsavel
* por manter todas as tabelas com os ultimos valores recebidos do medidor
* para envio via LoRa.
*
* @date Feb/2019
* @author FITec - Fundacao para Inovacoes Tecnologicas
*         Marcio J Teixeira Jr. <mjunior@fitec.org.br>
*
************************************************************************/

#include <stdint.h>
#include <RTCZero.h>

#define DEBUG_PROTO

#include "enddevice.h"
#include "util.h"
#include "ansi_fitec.h"
#include "abnt.h"
#include "agenda.h"

/**********************************
 * Variaveis do modulo, exportadas
 **********************************/
volatile uint8_t *  loRaBuffPt;
volatile uint8_t    loRaBuffSize;
volatile bool       bloqPtANSI = false;
volatile uint32_t   t0_LoRa, t0_ABNT, novoIntervaloLoRa;
FITecModem modem;   //LoRaModem modem;
volatile bool       salvarNaoVolatil = false;

/* Variaveis globais com as tabelas a serem enviadas via LoRa */
tabela01_t ansi_tab1;
tabela02_t ansi_tab2;
tabela03_t ansi_tab3;
tabela04_t ansi_tab4;
tabela05_t ansi_tab5;
tabela06_t ansi_tab6;
tabela07_t ansi_tab7;
tabela08_t ansi_tab8;
tabela09_t ansi_tab9;

/**********************************
 * Variaveis internas ao modulo
 **********************************/
static uint8_t LoRaBuff[51];
static itemListaTabAnsi_t listaTabelasAEnviar[TAM_LISTA_CMD];
volatile static uint8_t ptEscritaTabela;
volatile static uint8_t ptLeituraTabela;
static uint8_t tabelaAtual;
static uint8_t tamEspecial;
static tabelasAnsi_t dadosTabela[NUM_TABELAS_UP];
static uint32_t tLastLoRaSend;

// Valores assumidos caso nao receba comando de comissionamento
char appEui[17] = SECRET_APP_EUI;
char appKey[33] = SECRET_APP_KEY;

static const String deviceAddr = ABP_DEVADDR;
static const String NetworkSKey = ABP_NWKSKEY;
static const String AppSessionKey = ABP_APPSKEY;

// Prototipos das funcoes locais
uint8_t buscaTabelaANSI(void);
void loRaSend(uint8_t *msg, uint8_t tam, bool ack = false);
int trataDownLink(void);
uint8_t calcSum(uint8_t *buffer, size_t bufSize);

/**
 * @brief Funcao para iniciar o modulo ANSI FITec
 *
 * Esta funcao inicia o modulo ANSI FITec. Aqui as variaveis sao inicializadas
 * e os controles sao colocados em seus estados iniciais.
 */
void ansiFitecInit(void)
{
    ptEscritaTabela = ptLeituraTabela = 0;
    tLastLoRaSend = 0;
    tamEspecial = 0;

    for (uint8_t i_tab = 0; i_tab < NUM_TABELAS_UP; ++i_tab)
    {
        dadosTabela[i_tab].id = 0;
        dadosTabela[i_tab].size = 0;
        dadosTabela[i_tab].tabela = nullptr;
    }
    dadosTabela[0].id = 1;
    dadosTabela[0].size = SIZE_TABELA1_COMPLETA;
    dadosTabela[0].tabela = (uint8_t *)&ansi_tab1;
    dadosTabela[1].id = 2;
    dadosTabela[1].size = SIZE_TABELA2;
    dadosTabela[1].tabela = (uint8_t *)&ansi_tab2;
    dadosTabela[2].id = 3;
    dadosTabela[2].size = SIZE_TABELA3;
    dadosTabela[2].tabela = (uint8_t *)&ansi_tab3;
    dadosTabela[3].id = 4;
    dadosTabela[3].size = SIZE_TABELA4;
    dadosTabela[3].tabela = (uint8_t *)&ansi_tab4;
    dadosTabela[4].id = 5;
    dadosTabela[4].size = SIZE_TABELA5;
    dadosTabela[4].tabela = (uint8_t *)&ansi_tab5;
    dadosTabela[5].id = 6;
    dadosTabela[5].size = SIZE_TABELA6;
    dadosTabela[5].tabela = (uint8_t *)&ansi_tab6;
    dadosTabela[6].id = 7;
    dadosTabela[6].size = SIZE_TABELA7;
    dadosTabela[6].tabela = (uint8_t *)&ansi_tab7;
    dadosTabela[7].id = 8;
    dadosTabela[7].size = SIZE_TABELA8;
    dadosTabela[7].tabela = (uint8_t *)&ansi_tab8;
    dadosTabela[8].id = 9;
    dadosTabela[8].size = SIZE_TABELA9;
    dadosTabela[8].tabela = (uint8_t *)&ansi_tab9;
} //ansiFitecInit(

/**
* @brief Funcao para inserir uma tabela ANSI FITec a ser enviada via LoRa
*
* Esta funcao recebe o codigo de uma tabela ANSI FITec e a inclui na lista de
* tabelas que serao enviadas via LoRa no proximo instante de envio
*
* @param[in] IdTabela Numero da tabela a ser inserida na lista
* @return true caso a tabela seja inserida corretamente, false em caso de erro
*
************************************************************************/
bool insereTabelaANSI(uint8_t IdTabela, uint8_t tam)
{
    if (LISTA_NEXT_PT(ptEscritaTabela) == LISTA_PT(ptLeituraTabela))
        return false;       //lista cheia

    bloqPtANSI = true; //Bloqueia acesso a lista para evitar inconsistencia
    itemListaTabAnsi_t rec;
    rec.idTabela = IdTabela;
    rec.tamEspecial = tam;
    listaTabelasAEnviar[LISTA_INC_PT(ptEscritaTabela)] = rec;
    bloqPtANSI = false; //Libera acesso a lista
    return true;
} //insereTabelaANSI(

/**
* @brief Funcao para recuperar uma tabela ANSI FITec da lista de tabelas a ser
* enviada
*
* Esta funcao retorna o codigo da proxima tabela da lista a ser enviada via
* LoRa. A lista de tabelas e um buffer circular, assim caso os ponteiros
* de escrita e leitura sejam coincidentes, retorna zero indicando que a lista
* esta vazia.
*
* @return Codigo do proximo comando, zero para lista vazia
*
************************************************************************/
uint8_t buscaTabelaANSI(void)
{
    uint8_t ret;

    if (LISTA_PT(ptLeituraTabela) == LISTA_PT(ptEscritaTabela))
        return 0;

    bloqPtANSI = true; //Bloqueia acesso a lista para evitar inconsistencia
    SerialDebug.println("Wr: " + String(LISTA_PT(ptEscritaTabela)) + "\tRd: " +
                                 String(LISTA_PT(ptLeituraTabela)));
    SerialDebug.flush();
    tamEspecial = listaTabelasAEnviar[LISTA_PT(ptLeituraTabela)].tamEspecial;
    ret = listaTabelasAEnviar[LISTA_INC_PT(ptLeituraTabela)].idTabela;
    bloqPtANSI = false; //Libera acesso a lista

    //--- log de sanidade
    logSanidadeLocal.ptEscritaLoRa = ptEscritaTabela;
    logSanidadeLocal.ptLeituraLoRa = ptLeituraTabela;

    return ret;
} //buscaTabelaANSI(

int8_t loRaSetup(void)
{
    if (!modem.begin(AU915))
    {
        SerialDebug.println("¡¡¡ ERRO no modem.begin() !!! \r\n\r\n");
        while (1) ;
    }
    delay(500);
    if (!modem.publicNetwork(false))
        SerialDebug.println("¡¡¡ ERRO ao definir modo PRIVADO!\r\n\r\n");

    delay(500);
    if (!modem.configureClass(CLASS_C))
        SerialDebug.println("¡¡¡ ERRO ao mudar de classe!\r\n\r\n");

    //-------- DEBUG ----------
    SerialDebug.println("Your module version is: " + modem.version());
    SerialDebug.println("Your device EUI is: " + modem.deviceEUI());
    SerialDebug.println("App EUI is: " + String(localKeys.appeui));
    SerialDebug.println("App key is: " + String(localKeys.appkey) + "\r\n\r\n");
    //-------- DEBUG ----------

    loRaBuffPt = LoRaBuff;
    loRaBuffSize = 0;
    modem.minPollInterval(12);
    return 0;
} //loRaSetup(

void updateTLoRaSend(void)
{
    tLastLoRaSend = millis();
}

bool trataLoRa(void)
{
    static uint8_t contLoRaNConect = 0;

    if (!modem.isJoined())
    {
        if (++contLoRaNConect > MAX_LORA_NCONECT)
        {
            contLoRaNConect = 0;
            return false;
        }
    }
    else
        contLoRaNConect = 0;

    if ((millis() - tLastLoRaSend) > MIN_LORA_INTERVAL)
    {
        tabelaAtual = buscaTabelaANSI();
        if (tabelaAtual)
        {
#       ifdef DEBUG_LORA
            SerialDebug.println("LoRa tab: " + String(tabelaAtual));
            SerialDebug.flush();
#       endif
            //Tratamento para retirar possiveis entradas de solicitacao de
            //acerto de hora duplicadas, desnecessarias
            if ((tabelaAtual == TAB_ANSI05) &&
                (tamEspecial == SIZE_TABELA5_SOLICITA_RTC))
            {
                if (!ansi_tab5.solicitaRTC)
                {
// #             ifdef DEBUG_LORA
//                   SerialDebug.println("\r\n ---Remove entrada de Tab5");
// #             endif
                    return true;
                }
            }
            tabelaAtual--; //Ajusta para usar como indice do vetor de dados
            LoRaBuff[0] = dadosTabela[tabelaAtual].id;
            LoRaBuff[1] = 0;
            if (tamEspecial)
                LoRaBuff[2] = tamEspecial;
            else
                LoRaBuff[2] = dadosTabela[tabelaAtual].size; //tamanho dos dados
            LoRaBuff[3] = 0;
            loRaBuffSize = 4;
            memcpy((void *)(loRaBuffPt + loRaBuffSize), \
                   (uint8_t *)dadosTabela[tabelaAtual].tabela, \
                   LoRaBuff[2]);
            loRaBuffSize += LoRaBuff[2];
            LoRaBuff[loRaBuffSize] = calcSum((uint8_t *)(loRaBuffPt + 4), \
                                             LoRaBuff[2]);
            loRaBuffSize++;

            SerialDebug.println(" <- Bfr send");
            loRaSend((uint8_t *)(loRaBuffPt), loRaBuffSize);
            SerialDebug.println(" -> Aft send");
        }
    }
    if (-1 == trataDownLink())
    {
#   ifdef DEBUG_LORA
        SerialDebug.println("\r\nErro no downlink !!!\r\n");
#   endif
        piscaLed(3, 500, 100);
    }
    return true;
} //trataLoRa(

void loRaSend(uint8_t *msg, uint8_t tam, bool ack)
{
    int err;

    //###### DEBUG do envio LoRa
    // SerialDebug.println("Vai enviar " + String(tam) + " bytes:");
    // for (auto i = 0; i < tam;  i++)
    //     SerialDebug.println("[" + String(i) +"]: " + String(msg[i], HEX));
    // SerialDebug.println();
    //#######

    tLastLoRaSend = millis();
    modem.beginPacket();
    modem.write(msg, tam);
    Watchdog.reset();
    err = modem.endPacket(ack);

    if (err > 0)
    {
        piscaLed(2, 350, 100);
   #ifdef DEBUG_LORA
        SerialDebug.println("Envio LoRa !!! \tTab: " + String(tabelaAtual + 1)); //+1 pq foi ajustada pra usar como indice
        localKeys.log_sanidade.cont_up++;
   #endif
    }
    else
    {
        //se falhou o envio, reinicia o modem!
        modem.restart();
        delay(500);
        modem.begin(AU915);
        delay(500);

        piscaLed(4, 200, 100);
    #ifdef DEBUG_LORA
        SerialDebug.println("FALHA no envio!!!");
    #endif

    }
} //loRaSend(

int trataDownLink(void)
{
    uint8_t rcv[64];
    uint8_t i = 0;

    tabela10_so_relogio_t *recebeRelogio;
    struct tm *datahora_tm;
    tabela10_t *tab10;
    tabela11_t *tab11;
    tabela12_t *tab12;

    if (modem.available())
    {
        while (modem.available())
        {
            rcv[i++] = (uint8_t)modem.read();
        }

#ifdef DEBUG_LORA
        SerialDebug.print("DL Recvd: ");
        for (uint8_t j = 0; j < i; ++j)
        {
            SerialDebug.print(rcv[j] >> 4, HEX);
            SerialDebug.print(rcv[j] & 0xF, HEX);
            SerialDebug.print(" ");
        }
        SerialDebug.println();
        localKeys.log_sanidade.cont_dw++;

        ///-----------------------------------------------------
        //#### Debug do log de sanidade
        //
        logSanidadeLocal.currUptime = (((((uint64_t)logSanidadeLocal.uptime_rollMilli) << 32)
                   + millis()) / 1000);

        SerialDebug.println("\r\n-------------------------\r\nLog de sanidade:");
        SerialDebug.println("POR: " + String(localKeys.log_sanidade.cont_POR));
        SerialDebug.println("SW: " + String(localKeys.log_sanidade.cont_SWrst));
        SerialDebug.println("WDT: " + String(localKeys.log_sanidade.cont_WDT));
        SerialDebug.println("Ext: " + String(localKeys.log_sanidade.cont_EXTrst));
        SerialDebug.println("Stk Ovf: " + String(localKeys.log_sanidade.cont_StOvflw));
        SerialDebug.println("Uplinks: " + String(localKeys.log_sanidade.cont_up));
        SerialDebug.println("Dwlinks: " + String(localKeys.log_sanidade.cont_dw));
        //SerialDebug.println("Uptime roll: " + String(localKeys.log_sanidade.uptime_rollMilli));
        SerialDebug.println("Uptime sec: " + String((uint8_t)logSanidadeLocal.currUptime));
        SerialDebug.println("MaxUpTime: " + String((uint32_t)localKeys.log_sanidade.maxUptime));
        // SerialDebug.println("ptLeitABNT Urgente: " + String((uint8_t)localKeys.log_sanidade.ptLeituraABNTUrgente));
        // SerialDebug.println("ptEscrABNT Urgente: " + String((uint8_t)localKeys.log_sanidade.ptEscritaABNTUrgente));
        // SerialDebug.println("ptLeitABNT: " + String((uint8_t)localKeys.log_sanidade.ptLeituraABNT));
        // SerialDebug.println("ptEscrABNT: " + String((uint8_t)localKeys.log_sanidade.ptEscritaABNT));
        // SerialDebug.println("ptLeitLoRa: " + String((uint8_t)localKeys.log_sanidade.ptLeituraLoRa));
        // SerialDebug.println("ptEscrLoRa: " + String((uint8_t)localKeys.log_sanidade.ptEscritaLoRa));
        SerialDebug.println("\r\n------- Fim do Log de sanidade -------");
        //------------------------------------------------------
#endif

        switch (rcv[0])
        {
        case TAB_ANSI10:
            if (SIZE_TABELA10_SO_RELOGIO == rcv[2])
            {
                time_t recDataHora;
                if (rcv[4 + SIZE_TABELA10_SO_RELOGIO] != calcSum(rcv + 4, SIZE_TABELA10_SO_RELOGIO))
                    return -1;
                recebeRelogio = (tabela10_so_relogio_t *)(rcv + 4);
                recDataHora = (recebeRelogio->dataHoraCFG);
                datahora_tm = localtime(&recDataHora);
            }
            else if (SIZE_TABELA10 == rcv[2])
            {
                time_t recDataHora;
                if (rcv[4 + SIZE_TABELA10] != calcSum((uint8_t *)(rcv + 4),\
                                                             SIZE_TABELA10))
                    return -1;
                tab10 = (tabela10_t *)(rcv + 4);
                recDataHora = (tab10->dataHoraCFG);
                datahora_tm = localtime(&recDataHora);

                abntAlteraHorVer.horarioVerao.ativar = tab10->horVeraoCFG.ativar;
                abntAlteraHorVer.horarioVerao.diaFimHorInverno = \
                            abntByteToBcd(tab10->horVeraoCFG.diaFimHorInverno);
                abntAlteraHorVer.horarioVerao.mesFimHorInverno = \
                            abntByteToBcd(tab10->horVeraoCFG.mesFimHorInverno);
                abntAlteraHorVer.horarioVerao.diaFimHorVerao = \
                            abntByteToBcd(tab10->horVeraoCFG.diaFimHorVerao);
                abntAlteraHorVer.horarioVerao.mesFimHorVerao = \
                            abntByteToBcd(tab10->horVeraoCFG.mesFimHorVerao);

                for (uint8_t i_posto = PONTA; i_posto <= INTERMEDIARIO; ++i_posto)
                {
                    abntProgSegmentosHorarios.progPostoHor[i_posto].\
    horaInicio1 =   abntByteToBcd(tab10->postoHorarioCFG[i_posto].horaInicio1);
                    abntProgSegmentosHorarios.progPostoHor[i_posto].\
    minutoInicio1 = abntByteToBcd(tab10->postoHorarioCFG[i_posto].minutoInicio1);
                    abntProgSegmentosHorarios.progPostoHor[i_posto].\
    horaInicio2 =   abntByteToBcd(tab10->postoHorarioCFG[i_posto].horaInicio2);
                    abntProgSegmentosHorarios.progPostoHor[i_posto].\
    minutoInicio2 = abntByteToBcd(tab10->postoHorarioCFG[i_posto].minutoInicio2);
                    abntProgSegmentosHorarios.progPostoHor[i_posto].\
    horaInicio3 =   abntByteToBcd(tab10->postoHorarioCFG[i_posto].horaInicio3);
                    abntProgSegmentosHorarios.progPostoHor[i_posto].\
    minutoInicio3 = abntByteToBcd(tab10->postoHorarioCFG[i_posto].minutoInicio3);
                    abntProgSegmentosHorarios.progPostoHor[i_posto].\
    horaInicio4 =   abntByteToBcd(tab10->postoHorarioCFG[i_posto].horaInicio4);
                    abntProgSegmentosHorarios.progPostoHor[i_posto].\
    minutoInicio4 = abntByteToBcd(tab10->postoHorarioCFG[i_posto].minutoInicio4);
                }
                //memcpy(&(abntProgSegmentosHorarios.progPostoHor), &(tab10->postoHorarioCFG), sizeof(tab10->postoHorarioCFG));
                abntProgSegmentosHorarios.numSegmentos = tab10->numSegHorarios;
            }
            else
                return -1;

            //Ajusta o RTC com a programacao recebida
            rtc.setDate(datahora_tm->tm_mday, (datahora_tm->tm_mon + 1),\
                                              (datahora_tm->tm_year - 100));
            rtc.setTime(datahora_tm->tm_hour, datahora_tm->tm_min, \
                                              datahora_tm->tm_sec);

            //recalcula a agenda, para refletir o acerto de hora
            iniciaAlarmes();
            //Sinaliza que ja tem hora valida
            relogioValido = true;
            ansi_tab5.solicitaRTC = false;

            //Agenda o envio dos comandos para o medidor com as programacoes recebidas
            insereCmdABNT(LISTA_NORMAL, ID_CMD29);
            insereCmdABNT(LISTA_NORMAL, ID_CMD30);
            ansi_tab5.alarmes.relogio_dispositivo = 0;
            ansi_tab5.alarmes.relogio_medidor = 0;
            if (SIZE_TABELA10 == rcv[2])
            {
                insereCmdABNT(LISTA_NORMAL, ID_CMD35);
                insereCmdABNT(LISTA_NORMAL, ID_CMD64);
            }
            insereCmdABNT(LISTA_NORMAL, ID_CMD20);

            //Atualiza as leituras
            insereCmdABNT(LISTA_NORMAL, ID_CMD14);
            insereCmdABNT(LISTA_NORMAL, ID_CMD25);
            insereCmdABNT(LISTA_NORMAL, ID_CMD21, CANAL_123);
            insereCmdABNT(LISTA_NORMAL, ID_CMD23);
            insereCmdABNT(LISTA_NORMAL, ID_CMDE2, CMD_LANDIS_LEITURA_PARAM);
            insereCmdABNT(LISTA_NORMAL, ID_CMD28);
            insereCmdABNT(LISTA_NORMAL, ID_CMD87);
            insereCmdABNT(LISTA_NORMAL, ID_CMDE2, CMD_LANDIS_DRP_DRC);
            break;

        case TAB_ANSI11:
            tab11 = (tabela11_t *)(rcv + 4);
            if (rcv[4 + SIZE_TABELA11] != calcSum(rcv + 4, SIZE_TABELA11))
                return -1;

            for (uint8_t fer = 0; fer < 15; fer++)
            {
                abntProgFeriados.feriados[fer].dia = \
                            abntByteToBcd(tab11->feriadosCFG[fer].dia);
                abntProgFeriados.feriados[fer].mes = \
                            abntByteToBcd(tab11->feriadosCFG[fer].mes);
                abntProgFeriados.feriados[fer].ano = \
                            abntByteToBcd(tab11->feriadosCFG[fer].ano);
            }
            insereCmdABNT(LISTA_NORMAL, ID_CMD32);
            insereCmdABNT(LISTA_NORMAL, ID_CMD20);
            //Atualiza as leituras
            insereCmdABNT(LISTA_NORMAL, ID_CMD14);
            insereCmdABNT(LISTA_NORMAL, ID_CMD25);
            insereCmdABNT(LISTA_NORMAL, ID_CMD21, CANAL_123);
            insereCmdABNT(LISTA_NORMAL, ID_CMD23);
            insereCmdABNT(LISTA_NORMAL, ID_CMDE2, CMD_LANDIS_LEITURA_PARAM);
            insereCmdABNT(LISTA_NORMAL, ID_CMD28);
            insereCmdABNT(LISTA_NORMAL, ID_CMD87);
            insereCmdABNT(LISTA_NORMAL, ID_CMDE2, CMD_LANDIS_DRP_DRC);
            break;

        case TAB_ANSI12:
        {
            uint8_t tam = rcv[2];
            tab12 = (tabela12_t *)(rcv + 4);
            if (rcv[4 + tam] != calcSum(rcv + 4, tam))
                return -1;
            programacaoCorteReliga = tab12->corteReliga;
            if (tab12->intervalo && (localKeys.intervalo != tab12->intervalo))
            {
                novoIntervaloLoRa = tab12->intervalo;
            }
            if (tam > 2)
            {
                for (uint8_t i_tab = 0; i_tab < (tam - 2); ++i_tab)
                {
                    if (tab12->solicitaTabela[i_tab])
                        insereTabelaANSI(tab12->solicitaTabela[i_tab]);
                    else
                        break;
                }
            }

            if (tam == SIZE_TABELA12)
            {
                memcpy(localKeys.senhaABNT, tab12->senhaABNT,
                       sizeof(localKeys.senhaABNT));
                localKeys.senhaABNT_ok = FLASH_VALID;
                ansi_tab5.alarmes.senha_abnt = 0;
                salvarNaoVolatil = true;
            }
            if (programacaoCorteReliga)
                insereCmdABNT(LISTA_NORMAL, ID_CMDE2, CMD_LANDIS_CORTE_RELIGA);

            //Atualiza as leituras
            insereCmdABNT(LISTA_NORMAL, ID_CMD14);
            insereCmdABNT(LISTA_NORMAL, ID_CMD25);
            insereCmdABNT(LISTA_NORMAL, ID_CMD21, CANAL_123);
            insereCmdABNT(LISTA_NORMAL, ID_CMD23);
            insereCmdABNT(LISTA_NORMAL, ID_CMDE2, CMD_LANDIS_LEITURA_PARAM);
            insereCmdABNT(LISTA_NORMAL, ID_CMD28);
            insereCmdABNT(LISTA_NORMAL, ID_CMD87);
            insereCmdABNT(LISTA_NORMAL, ID_CMDE2, CMD_LANDIS_DRP_DRC);
            break;
        }
        default:
            break;
        }
    }
    return 0;
} //trataDownLink(

/**
* @brief Calcula checksum simples de um buffer
*
* Esta funcao calcula um checksum, de 1 byte, para um buffer passado
* como parametro. O cálculo é feito somando-se todos os bytes desconsiderando
* o overflow e tomando-se o complemento de 2 do resultado final.
* Para compatibilidade com o protocolo ANSI, deve-se calcular este checksum
* sobre o campo de dados da tabela e envia-lo no fim do pacote.
*
* @param[in] buffer Buffer contendo os dados a serem considerados no checksum
* @param[in] bufSize Tamanho do buffer
* @return checksum de todos os bytes do buffer, em 1 byte
*/
uint8_t calcSum(uint8_t *buffer, size_t bufSize)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < bufSize; ++i)
        sum += buffer[i];

    return (~sum) + 1;
} //calcSum(
