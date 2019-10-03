/** @file agenda.cpp
* @brief Arquivo com a implementacao das funcoes relacionada a agenda usando o rtc
*
* Este modulo contem toda a implementacao da agenda, que e utilizada para gerenciar
* todos os eventos baseados em tempo como o envio de pacotes LoRa e comandos ABNT
* frequentes e diarios.
*
* @date Jun/2019
* @author FITec - Fundacao para Inovacoes Tecnologicas
*         Marcio J Teixeira Jr. <mjunior@fitec.org.br>
*
************************************************************************/

#include <Arduino.h>
#include <stdint.h>
#include <RTCZero.h>

#define DEBUG_AGENDA

#include "enddevice.h"
#include "agenda.h"
#include "util.h"
#include "ansi_fitec.h"

/** Variaveis do modulo, exportadas */

itemAgenda_t *listaAgenda, *itemAtual;
RTCZero rtc;
uint8_t intervaloLoRa;
int32_t localMaxTemp = 0;
bool relogioValido = false;

/**
* @brief Funcao para inicializar a agenda
*
* Esta funcao inicializa o modulo de agenda. Aqui sao definidos os instantes
* para envio dos pacotes frequentes e diarios, eh inicializado o RTC e em
* seguida montada a Agenda a partir dos dados definidos anteriormente.
*
************************************************************************/
void initAgenda(void)
{
    if (localKeys.comissionado != FLASH_VALID)
    {
        t0_LoRa = 65; //random(NUM_SEG_DIA);
        intervaloLoRa = 1;
    }
    else
    {
        t0_LoRa = localKeys.seed;
        intervaloLoRa = localKeys.intervalo;
    }
    t0_ABNT = t0_LoRa + OFFSET_LORA_ABNT;
    listaAgenda = nullptr;
    rtc.begin();
    rtc.setDate(4, 4, 19);
    rtc.setTime(23, 59, 50); //Incializacao manual do RTC

    #ifdef DEBUG_AGENDA
    SerialDebug.println("bfr MontaAgenda");
    SerialDebug.flush();
    #endif
    montaAgenda();
    itemAtual = listaAgenda;
    #ifdef DEBUG_AGENDA
    SerialDebug.println("\r\nFim MontaAgenda!\r\n");
    SerialDebug.flush();
    imprimeAgenda();
    #endif
} //initAgenda(

/**
* @brief Funcao para montar a agenda diaria
*
* Esta funcao monta a lista com os eventos diarios. Antes de montar a lista
* ela e esvaziada, atraves da funcao limpaLista(). Em seguida, baseado nos
* horarios ja sorteados e no intervalo definido os eventos diarios sao
* calculados e as respectivas funcoes cadastradas para serem chamadas pela
* agenda do RTC.
***************************************************************************/
void montaAgenda(void)
{
    #ifdef DEBUG_AGENDA
    SerialDebug.println("\r\nmontaAgenda! ");
    SerialDebug.flush();
    #endif
    limpaLista(listaAgenda);
    listaAgenda = nullptr;

    #ifdef DEBUG_AGENDA
    SerialDebug.println("\r\nlistaAgenda ja eh NULL... ");
    SerialDebug.flush();
    #endif

    uint8_t nEventosFreqLoRa = (24 / intervaloLoRa);
    itemAgenda_t * novoItem;

    uint32_t intervaloLoRaSeg = intervaloLoRa * NUM_SEG_HORA;

    #ifdef DEBUG_AGENDA
    SerialDebug.println("\r\nintervaloLora seg = " + String(intervaloLoRaSeg));
    SerialDebug.flush();
    #endif

    //Pacotes frequentes LoRa
    for (uint8_t n = 0; n < nEventosFreqLoRa; ++n)
    {
        novoItem = criaItemAgenda((t0_LoRa + (n * intervaloLoRaSeg))% \
                                   NUM_SEG_DIA, transmiteLoRaFrequente);
        insereAgenda(&listaAgenda, novoItem);
    }

    #ifdef DEBUG_AGENDA
    SerialDebug.println("\r\nCriados " + String(nEventosFreqLoRa) + " eventos LoRa freq\r\n");
    SerialDebug.flush();
    #endif

    //Pacotes eventuais LoRa
    novoItem = criaItemAgenda((t0_LoRa + OFFSET_LORA_EVENTUAL)% \
                               NUM_SEG_DIA, transmiteLoRaEventual);
    insereAgenda(&listaAgenda, novoItem);

    //Envio de comandos ABNT frequentes
    for (uint8_t n = 0; n < 24; ++n)
    {
        novoItem = criaItemAgenda((t0_ABNT + (n * NUM_SEG_HORA))% \
                                   NUM_SEG_DIA, preparaABNTFrequente);
        insereAgenda(&listaAgenda, novoItem);
    }

    #ifdef DEBUG_AGENDA
    SerialDebug.println("\r\nCriados 24 eventos ABNT freq\r\n");
    SerialDebug.flush();
    #endif

    //Envio de comandos ABNT diario
    novoItem = criaItemAgenda((t0_ABNT + OFFSET_ABNT_EVENTUAL)% \
                               NUM_SEG_DIA, preparaABNTEventual);
    insereAgenda(&listaAgenda, novoItem);

    //Atualizacao do uptime, verificando o estouro de millis()
    // 1x / dia, 2 minutos depois do ABNT eventual
    novoItem = criaItemAgenda((t0_ABNT + OFFSET_ABNT_EVENTUAL + 120)% \
                               NUM_SEG_DIA, atualizaUptime);
    insereAgenda(&listaAgenda, novoItem);

    #ifdef DEBUG_AGENDA
    SerialDebug.println("\r\nCriado evento ABNT diario\r\n");
    SerialDebug.flush();
    #endif

    localKeys.intervalo = intervaloLoRa;

    #ifdef DEBUG_AGENDA
    SerialDebug.println("\r\nIntervalo escrito na mem\r\n");
    SerialDebug.flush();
    #endif
}// void montaAgenda(

/**
* @brief Funcao para inserir ordenado na lista encadeada de itens da agenda
*
* Esta funcao insere um novo item de agenda na lista encadeada de agenda,
* ordenando pelo horario do item.
*
* @param[in] **head ponteiro para inicio da lista encadeada
* @param[in] *itemAInserir ponteiro para item a ser inserido
*
************************************************************************/
void insereAgenda(itemAgenda_t **head, itemAgenda_t *itemAInserir)
{
    itemAgenda_t *tmp = nullptr;
    if (*head == nullptr || itemAInserir->tAgenda < ((*head)->tAgenda))
    {
        itemAInserir->next = *head;
        *head = itemAInserir;
        return;
    }
    tmp = *head;

    while(tmp->next && (itemAInserir->tAgenda > tmp->next->tAgenda))
    {
        tmp = tmp->next;
    }
    itemAInserir->next = tmp->next;
    tmp->next = itemAInserir;
} //insereAgenda(

/**
* @brief Funcao para criar um novo item da agenda
*
* Esta funcao aloca memoria e cria um novo item de agenda a
* partir do horario desejado e funcao a ser chamada
*
* @param[in] new_tAgenda horario desejado, em segundos apos 00h00m00s
* @param[in] new_fAgenda ponteiro para funcao a ser chamada no horario agendado
*
************************************************************************/
itemAgenda_t * criaItemAgenda(uint32_t new_tAgenda, void (* new_fAgenda) (void))
{
    itemAgenda_t * newItem = (itemAgenda_t *) malloc(sizeof(itemAgenda_t));
    newItem->tAgenda = new_tAgenda;
    newItem->fAgenda = new_fAgenda;
    newItem->next = nullptr;

    return(newItem);
} //criaItemAgenda(

/*------------------------------
***
*** FUNCAO APENAS PARA DEBUG
***
-------------------------------*/
void imprimeAgenda(void)
{
    uint8_t contItem = 0;
    itemAgenda_t *pImprime = listaAgenda;//itemAtual;
    SerialDebug.println(" --- AGENDA Atual - rnd: " + String(t0_LoRa));
    SerialDebug.flush();
    while (pImprime->next)
    {
        SerialDebug.println(String(contItem++) + ": " + String(pImprime->tAgenda));
        pImprime = pImprime->next;
    }
    SerialDebug.flush();
    SerialDebug.println(String(contItem++) + ": " + String(pImprime->tAgenda));
    SerialDebug.println(" -- FIM -- \r\n");
    SerialDebug.flush();
} //imprimeAgenda(

/**
* @brief Funcao para iniciar a agenda do RTC baseado no horario atual.
*
* Esta funcao inicializa a agenda do RTC com o horario do proximo evento
* cadastrando a respectiva funcao a ser chamada.
*
************************************************************************/
void iniciaAlarmes(void)
{
    uint32_t horaAtual = 0;
    uint8_t horaAlarme, minAlarme, segAlarme;
    horaAtual = hhmmssParaSegundos(rtc.getHours(), rtc.getMinutes(),\
                                   rtc.getSeconds());
    itemAtual = listaAgenda;

    #ifdef DEBUG_AGENDA
    SerialDebug.println("\r\nhora atual: " + String(horaAtual) + "\r\n");
    SerialDebug.flush();
    #endif

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

    #ifdef DEBUG_AGENDA
    SerialDebug.println("\r\nPrim evento:" + String(itemAtual->tAgenda) +"\r\n");
    SerialDebug.flush();
    #endif

    segundosParaHHMMSS(itemAtual->tAgenda, &horaAlarme, &minAlarme, &segAlarme);
    rtc.setAlarmTime(horaAlarme, minAlarme, segAlarme);
    rtc.enableAlarm(rtc.MATCH_HHMMSS);
    rtc.attachInterrupt(itemAtual->fAgenda);
    #ifdef DEBUG_AGENDA
    SerialDebug.println("\r\nAgenda adicionada!\r\n");
    SerialDebug.flush();
    #endif
} //iniciaAlarmes(

/**
* @brief Funcao para programar o proximo alarme
*
* Esta funcao atualiza a variavel global itemAtual com o proximo evento
* da lista. Ao chegar no ultimo evento do dia avanca para o primeiro.
************************************************************************/
void programaAlarmeAgenda(void)
{
    uint8_t horaAlarme, minAlarme, segAlarme;

    if (itemAtual->next)
        itemAtual = itemAtual->next;
    else
        itemAtual = listaAgenda; //volta ao inicio
    segundosParaHHMMSS(itemAtual->tAgenda, &horaAlarme, &minAlarme,\
                                                        &segAlarme);
    rtc.setAlarmTime(horaAlarme, minAlarme, segAlarme);
    rtc.attachInterrupt(itemAtual->fAgenda);

    //// DEBUG agenda
    uint32_t horaAtual = hhmmssParaSegundos(rtc.getHours(), rtc.getMinutes(),\
                                                            rtc.getSeconds());
    #ifdef DEBUG_AGENDA
    SerialDebug.println("atual: " + String(horaAtual));
    SerialDebug.println("agenda para: " + String(itemAtual->tAgenda) + "\r\n");
    #endif
} //programaAlarmeAgenda(

void transmiteLoRaFrequente(void)
{
    if (bloqPtANSI)
    {
        SerialDebug.println("LoRa freq - Bloqueada !");
        //Soma 2 segundos para evitar mudanca instantanea
        auto now = rtc.getEpoch() + 2;
        rtc.setAlarmEpoch(now);
        return;
    }
    SerialDebug.println("transmiteLoRaFrequente");
    programaAlarmeAgenda();
    ansi_tab1.temperatura = localMaxTemp;
    // SerialDebug.println("\r\nTemperatura localMax: " + String(localMaxTemp));
    // SerialDebug.println("\r\nTemperatura a ser enviada: " + String(ansi_tab1.temperatura));
    localMaxTemp = 0;
    // SerialDebug.println("\r\nMaxTemp zerada!\r\n\r\n");

    //leAlarmeMedidor();
    insereTabelaANSI(TAB_ANSI01);
    insereTabelaANSI(TAB_ANSI02);
} //transmiteLoRaFrequente(

void transmiteLoRaEventual(void)
{
    if (bloqPtANSI)
    {
        SerialDebug.println("LoRa eventual - Bloqueada !");
        //Soma 2 segundos para evitar mudanca instantanea
        auto now = rtc.getEpoch() + 2;
        rtc.setAlarmEpoch(now);
        return;
    }
    SerialDebug.println ("transmiteLoRaEventual");
    programaAlarmeAgenda();
    //leAlarmeMedidor();
    insereTabelaANSI(TAB_ANSI03);
    insereTabelaANSI(TAB_ANSI04);
} //transmiteLoRaEventual(

void preparaABNTFrequente(void)
{
    if (bloqPtABNT)
    {
        SerialDebug.println("ABNT freq - Bloqueada !");
        //Soma 2 segundos para evitar mudanca instantanea
        auto now = rtc.getEpoch() + 2;
        rtc.setAlarmEpoch(now);
        return;
    }
    SerialDebug.println("preparaABNTFrequente");
    programaAlarmeAgenda();
    insereCmdABNT(LISTA_NORMAL, ID_CMD14);
    insereCmdABNT(LISTA_NORMAL, ID_CMD25);
    insereCmdABNT(LISTA_NORMAL, ID_CMD21, CANAL_123);
    insereCmdABNT(LISTA_NORMAL, ID_CMD23);
    insereCmdABNT(LISTA_NORMAL, ID_CMDE2, CMD_LANDIS_LEITURA_PARAM);

    // Le Alarmes
} //preparaABNTFrequente(

void preparaABNTEventual(void)
{
    if (bloqPtABNT)
    {
        SerialDebug.println("ABNT eventual - Bloqueada !");
        //Soma 2 segundos para evitar mudanca instantanea
        auto now = rtc.getEpoch() + 2;
        rtc.setAlarmEpoch(now);
        return;
    }
    SerialDebug.println("preparaABNTEventual");
    programaAlarmeAgenda();
    insereCmdABNT(LISTA_NORMAL, ID_CMD14);
    insereCmdABNT(LISTA_NORMAL, ID_CMD25);
    insereCmdABNT(LISTA_NORMAL, ID_CMD28);
    insereCmdABNT(LISTA_NORMAL, ID_CMD87);
    insereCmdABNT(LISTA_NORMAL, ID_CMD21, CANAL_123);
    insereCmdABNT(LISTA_NORMAL, ID_CMD23);
    insereCmdABNT(LISTA_NORMAL, ID_CMDE2, CMD_LANDIS_DRP_DRC);
} //preparaABNTEventual(

/**
* @brief Funcao para limpar a lista de enventos
*
* Esta funcao limpa a lista atual de eventos, desalocando as variaveis
* dinamicas. Deve ser chamada sempre que houver a necessidade de se
* recalcular a lista de eventos do dia.
*
* @param[in] head Ponteiro para o inicio da lista
************************************************************************/
void limpaLista(itemAgenda_t * head)
{
    itemAgenda_t * tmp;

    #ifdef DEBUG_AGENDA
    SerialDebug.println("\r\n limpaLista!");
    SerialDebug.flush();
    #endif
    while (head != nullptr)
    {
       #ifdef DEBUG_AGENDA
       SerialDebug.println("Here!");
       SerialDebug.flush();
       #endif
        tmp = head;
        head = head->next;
        free(tmp);
    }
} //limpaLista(

/**
* @brief Funcao para atualizar o contador de uptime, verificando o estou de
*        millis()
*
* Esta funcao verifica o estouro do contador de milissegundos do sistema e
* incrementa o contador de vezes que ele estourou. Este contador define o
* tempo que o dispositivo está ligado. No log de senaidade este contador e
* somado ao valor atualde millis() e dividido por 1000 para manter um contador
* de uptime em segundos.
* Para o correto funcionamento esta funcao deve ser chamada, no minimo 2x a cada
* ciclo do contador de milissegundos (~49,7 dias).
*
************************************************************************/
void atualizaUptime(void){

    static uint32_t currUptime, lastMillis;
    auto currMillis = millis();

    if (currMillis < lastMillis)
    {
        localKeys.log_sanidade.uptime_rollMilli++;
    }
    currUptime = (((uint64_t)(localKeys.log_sanidade.uptime_rollMilli << 32)
                   + currMillis) / 1000);
    if (currUptime > localKeys.log_sanidade.maxuptime)
        localKeys.log_sanidade.maxuptime = currUptime;


} //atualizaUptime(
