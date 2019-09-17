/** @file enddevice.cpp
* @brief Versao com integracao das funcionalidades.
*
* Aquivo com a implementacao do firmware integrando as funcionalidades
* de comunicacao ABNT via USB ou RS-232, LoRa com protocolo ANSI-FITec
* e agendamento de todos os eventos baseado no RTC, sem RTOS.
*
* @date Mar/2019
* @author FITec - Fundacao para Inovacoes Tecnologicas
*         Marcio J Teixeira Jr. <mjunior@fitec.org.br>
*
************************************************************************/

#include "enddevice.h"
#include "util.h"
#include <Arduino.h>
#include <stdint.h>
#ifdef TESTE_INTEGRACAO

#include "abnt.h"
#include "otica_ftdi.h"
#include "serial_nativa.h"
#include "ansi_fitec.h"
#include "agenda.h"
#include "wiring_private.h"
#include "adc.h"
#include "tempsens.h"
#include "tc_timer5.h"
#include "cripto.h"

#include <cdcftdi.h>
#include <usbhub.h>
#include <RTCZero.h>
#include <FlashStorage.h>
#include <time.h>

bool protocoloComissionamento(void);
void initComissionamento(void);
bool conectaLoRa(startType_t tipo);
void initHW(void);
uint32_t medeTemperatura(void);
void trataRespABNT(void);
void gravaSessaoLoRa(void);
int trataDownLink(void);
void inicializaSinalizacao(uint32_t sampleRate,
                           tipoTimer_t tipo = TIMER_MILLIS);
void configuraSinalizacao(sinalizaStatus_t status, uint32_t sampleRate,
                          tipoTimer_t tipo = TIMER_MILLIS);
void trataNovoIntervalo(void);
void trataSinalizacaoSucesso(void);
void maqEstSinalizacao(void);
void trataSinalizacaoNComissionado(void);
void trataSinalizacaoAguardaAck(void);
void trataSinalizacaoSemComunic(void);
void trataSinalizacaoSemLoRa(void);
void trataSinalizacaoSemABNT(void);
void trataSinalizacaoOpNormal(void);

sinalizaStatus_t sinalizaStatus = ST_NENHUM;

//Habilitacao de uma nova UART, Serial3, nos pinos 0, PA22 (Tx) e 1, PA23 (Rx)
Uart Serial3(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM3_Handler()
{
    Serial3.IrqHandler();
}

uint16_t bytesRecABNT;
uint32_t tLastABNTSend;
uint32_t tLastConectaLoRa;
bool semCmd87 = false;
bool LoRaOK = false;
bool comunicouABNT = false;
uint32_t horaSolicitaRTC, stepSolicitaRTC;
uint32_t tLastABNT;
volatile static bool ledSinalizacao;
volatile static uint8_t contPulso;

secret_keys_t localKeys;
FlashStorage(savedKeys, secret_keys_t);

uint32_t tLastTemp = 0;
//uint32_t tLasTab1TEST = 0; ///#### mmjr Teste enviando tab1

char localAppEui[17] = SECRET_APP_EUI;
char localAppKey[33] = SECRET_APP_KEY;

void setup()
{

    // 'Tempo morto' na inicialização...
    while (millis() < TEMPO_MORTO) ;
    initHW();

    //#########
    // Avaliar habilitacao de interrupcao para monitorar PWR_FAIL, geranfo um
    // reset total do sistema.
    //#########


#   ifdef SEMLORA
    SerialDebug.println("Executando SEM LoRa!");
#   else
    SerialDebug.println("Aguardando protocolo comissionamento.\r\n\r\n");
    //Aqui os valores locais (localKeys) sao zerado e se receber parametros
    //de comissionamento, sao gravados na memoria nao volatil com os demais
    //valores zerados.
    initComissionamento();
#   endif

    if (UsbH.Init())
    {
        delay(500);
    }

    //Sinaliza a inicializacao do end device
    piscaLed(3, 600, 600);

    delay(1200);
    abntInit();

    //No caso de ter recebido parametros de comissionamento, estara gravado
    //na memoria as chaves recebidas no comissionamento e os demais valores
    //zerados. Na situacao normal, tera os ultimos valores.
    localKeys = savedKeys.read();
    initAgenda();

    //SerialDebug.println("bfr Ansi INIT");
    ansiFitecInit();
    //SerialDebug.println("bfr conectaLoRa");
    loRaSetup();

    #ifndef SEMLORA
    if (localKeys.comissionado != FLASH_VALID)
    {
        configuraSinalizacao(ST_NCOMISSIONADO, TIMER_STNCOMISSIONADO);
        while (1) ; // Nao pode continuar sem senha para rede LoRa
    }
    #endif

    configuraSinalizacao(ST_SEMCOMUNIC, TIMER_STSEMCOMUNIC);

    iniciaAlarmes();
    //SerialDebug.println("Antes temp init");
    temp_init();
    //SerialDebug.println("Depois temp init");
    adc_init();

    uint8_t senhaE430[11] = {"TAETEEEUGT"};
    //uint8_t senhaCliente[11] = {"UJEUTETUGH"};

    memcpy(localKeys.senhaABNT, senhaE430, 10);
    savedKeys.write(localKeys);
    //#### Inicializa a senha para comunicacao com medidor!

    //Se nao receber acerto de RTC, envia solicitacao com t=[5 - 15]min)
    horaSolicitaRTC = stepSolicitaRTC = random(MILLIS_5MIN, MILLIS_15MIN);
    ansi_tab5.solicitaRTC = true;

    // //-----
    // SerialDebug.println("Hora sol. RTC: " + String(horaSolicitaRTC));
    // //-----

    //SerialDebug.println("bfr insereCMD");
    insereCmdABNT(LISTA_NORMAL, ID_CMD14);
    insereCmdABNT(LISTA_NORMAL, ID_CMD25);
    insereCmdABNT(LISTA_NORMAL, ID_CMD21, CANAL_123);
    insereCmdABNT(LISTA_NORMAL, ID_CMD23);
    insereCmdABNT(LISTA_NORMAL, ID_CMDE2, CMD_LANDIS_LEITURA_PARAM);
    insereCmdABNT(LISTA_NORMAL, ID_CMD28);

    // //Teste cmd parametrizacao (necessita senha)
    // programacaoCorteReliga = ANSI_PARAM_CORTE;
    // insereCmdABNT(LISTA_NORMAL, ID_CMDE2, CMD_LANDIS_CORTE_RELIGA);

    insereCmdABNT(LISTA_NORMAL, ID_CMD87);
    insereCmdABNT(LISTA_NORMAL, ID_CMDE2, CMD_LANDIS_DRP_DRC);

    tLastConectaLoRa = millis();
    SerialDebug.println("FIM Setup!");

    //Inicializa medicao de temperatura
    medeTemperatura();
    ansi_tab1.temperatura = localMaxTemp;

    // char senhaGravada[11];
    // for (uint8_t i = 0; i<10; i++)
    //     senhaGravada[i] = localKeys.senhaABNT[i];
    // senhaGravada[10] = 0;
    // SerialDebug.println("senha ABNT 1: " + String(senhaGravada));
    // SerialDebug.println();
    // SerialDebug.flush();
}

void loop()
{
    maqEstAbnt();
    trataRespABNT();
    if ((millis() - tLastABNT) > TMAX_SEM_ABNT)
        comunicouABNT = false;

    trataNovoIntervalo();

    if (salvarNaoVolatil)
    {
        savedKeys.write(localKeys);
        salvarNaoVolatil = false;
    }

    #ifndef SEMLORA
    if (LoRaOK)
        trataLoRa();
    else
    {
        if ((millis() - tLastConectaLoRa) > TMIN_SEM_CONECTALORA)
        {
           #ifdef SEM_CHMASK
            LoRaOK = conectaLoRa(COLD_START);
           #else
            if (localKeys.sessao_ok != FLASH_VALID)
                LoRaOK = conectaLoRa(COLD_START);
            else
                LoRaOK = conectaLoRa(WARM_START);
           #endif
        }
    }
    #else
    if (millis() > 35000)
        LoRaOK = true;
    #endif

    // A funcao de medir temperatura e muito demorada (~200ms), portanto
    // so e chamada se a maquina de estados ABNT esta em IDLE
    if ((stateABNT < ABNT_STATE_ENVIA_CMD) &&
        ((millis() - tLastTemp) > T_MIN_MEDE_TEMP))
    {
        //SerialDebug.println("Chama mede temp!\r\n");
        medeTemperatura();
        //SerialDebug.println("\r\nTemperatura localMax atual: " + String(localMaxTemp));
    }
    if ((millis() > horaSolicitaRTC) && !relogioValido)
    {
        //SerialDebug.println("Vai enviar solicitacao de RTC...");
        if (ansi_tab5.solicitaRTC)
        {
            insereTabelaANSI(TAB_ANSI05, SIZE_TABELA5_SOLICITA_RTC);
            horaSolicitaRTC += stepSolicitaRTC;
        }
    }

    // if ((millis() - tLasTab1TEST) > 45000)
    // {
    //     insereTabelaANSI(TAB_ANSI01);
    //     tLasTab1TEST = millis();
    //     ansi_tab1.temperatura = localMaxTemp;
    //     localMaxTemp = 0;
    // }

    trataSinalizacaoSucesso();
    //SerialDebug.println("St: " + String(sinalizaStatus) +
    //                 " Serial = " + String(portaSerial.interface));
    delay(50);
} //loop()

void trataRespABNT(void)
{
    abnt_resp_generic_t *buffRespRecebida;
    struct tm datahora_tm;
    time_t datahora;

    if (respABNTRecebida)
    {
        comunicouABNT = true;
        tLastABNT = millis();
        buffRespRecebida = (abnt_resp_generic_t *)pBuffABNTrecv;
        SerialDebug.println("Rec cmd: " + String(buffRespRecebida->id, HEX));

        switch (buffRespRecebida->id)
        {

        case ID_CMD11:
        {
            abnt_resp_generic_t *respCmd11;
            respCmd11 = buffRespRecebida;
            if (respCmd11->payload[1])
            {
                ///###DEBUG
                SerialDebug.println("\t --> Senha validada ! <--");
                insereCmdABNT(LISTA_URGENTE, cmdAtrasado.cmd,
                                  cmdAtrasado.codEstendido);
            }
        }
        break;

        case ID_CMD13:
        {
            abnt_resp_generic_t *respCmd13;
            respCmd13 = buffRespRecebida;
            memcpy (sementeABNT, respCmd13->payload, sizeof(sementeABNT));
        }
        break;

        case ID_CMD14:
        {
            abnt_resp_le_grand_instant_t *respLeGrandInst;
            respLeGrandInst = (abnt_resp_le_grand_instant_t *)buffRespRecebida;

            datahora_tm.tm_hour  = abntBcdToByte(respLeGrandInst->hora);
            datahora_tm.tm_min   = abntBcdToByte(respLeGrandInst->min);
            datahora_tm.tm_sec   = abntBcdToByte(respLeGrandInst->seg);
            datahora_tm.tm_mday  = abntBcdToByte(respLeGrandInst->dia);
            datahora_tm.tm_mon   = (abntBcdToByte(respLeGrandInst->mes)) - 1;
            datahora_tm.tm_year  = (abntBcdToByte(respLeGrandInst->ano)) + 100;
            datahora = mktime(&datahora_tm);
            ansi_tab1.timestamp = datahora;

            // Converte tensoes e corrente de float para Q.14
            ansi_tab3.tensao_a   = (respLeGrandInst->tensao_a) * (1 << 14);
            ansi_tab3.tensao_b   = (respLeGrandInst->tensao_b) * (1 << 14);
            ansi_tab3.tensao_c   = (respLeGrandInst->tensao_c) * (1 << 14);
            ansi_tab3.corrente_a = (respLeGrandInst->corrente_a) * (1 << 14);
            ansi_tab3.corrente_b = (respLeGrandInst->corrente_b) * (1 << 14);
            ansi_tab3.corrente_c = (respLeGrandInst->corrente_c) * (1 << 14);

            if (!relogioValido)
            {
                // acerta o RTC interno com a hora do medidor
                rtc.setDate(datahora_tm.tm_mday, (datahora_tm.tm_mon + 1),
                            (datahora_tm.tm_year - 100));
                rtc.setTime(datahora_tm.tm_hour, datahora_tm.tm_min,
                            datahora_tm.tm_sec);
                iniciaAlarmes();
                relogioValido = true;
                ansi_tab5.solicitaRTC = false;
            }
        }
        break;

        case ID_CMD21:
        {
            abnt_resp_leitura_param_t *respLeParam;
            uint16_t modeloLido;

            respLeParam = (abnt_resp_leitura_param_t *)buffRespRecebida;
            datahora_tm.tm_hour = abntBcdToByte(respLeParam->hora);
            datahora_tm.tm_min = abntBcdToByte(respLeParam->min);
            datahora_tm.tm_sec = abntBcdToByte(respLeParam->seg);
            datahora_tm.tm_mday = abntBcdToByte(respLeParam->dia);
            datahora_tm.tm_mon = (abntBcdToByte(respLeParam->mes)) - 1;
            datahora_tm.tm_year = (abntBcdToByte(respLeParam->ano)) + 100;
            datahora = mktime(&datahora_tm);
            ansi_tab1.timestamp = datahora;

            for (uint8_t i_posto = PONTA; i_posto <= RESERVADO; ++i_posto)
            {
                ansi_tab7.postoHorario[i_posto].horaInicio1 = abntBcdToByte(
                    respLeParam->inicioPostosHorarios[i_posto].horaInicio1);
                ansi_tab7.postoHorario[i_posto].minutoInicio1 = abntBcdToByte(
                    respLeParam->inicioPostosHorarios[i_posto].minutoInicio1);
                ansi_tab7.postoHorario[i_posto].horaInicio2 = abntBcdToByte(
                    respLeParam->inicioPostosHorarios[i_posto].horaInicio2);
                ansi_tab7.postoHorario[i_posto].minutoInicio2 = abntBcdToByte(
                    respLeParam->inicioPostosHorarios[i_posto].minutoInicio2);
                ansi_tab7.postoHorario[i_posto].horaInicio3 = abntBcdToByte(
                    respLeParam->inicioPostosHorarios[i_posto].horaInicio3);
                ansi_tab7.postoHorario[i_posto].minutoInicio3 = abntBcdToByte(
                    respLeParam->inicioPostosHorarios[i_posto].minutoInicio3);
                ansi_tab7.postoHorario[i_posto].horaInicio4 = abntBcdToByte(
                    respLeParam->inicioPostosHorarios[i_posto].horaInicio4);
                ansi_tab7.postoHorario[i_posto].minutoInicio4 = abntBcdToByte(
                    respLeParam->inicioPostosHorarios[i_posto].minutoInicio4);
            }

            for (uint8_t i_fer = 0; i_fer < 15; ++i_fer)
            {
                ansi_tab8.feriados[i_fer].ano = abntBcdToByte(
                    respLeParam->feriados[i_fer].ano);
                ansi_tab8.feriados[i_fer].mes = abntBcdToByte(
                    respLeParam->feriados[i_fer].mes);
                ansi_tab8.feriados[i_fer].dia = abntBcdToByte(
                    respLeParam->feriados[i_fer].dia);
            }

            ansi_tab7.horVerao.ativar = respLeParam->horVerao.ativar;
            ansi_tab7.horVerao.diaFimHorInverno = abntBcdToByte(respLeParam->
                                                    horVerao.diaFimHorInverno);
            ansi_tab7.horVerao.mesFimHorInverno = abntBcdToByte(respLeParam->
                                                    horVerao.mesFimHorInverno);
            ansi_tab7.horVerao.diaFimHorVerao   = abntBcdToByte(respLeParam->
                                                    horVerao.diaFimHorVerao);
            ansi_tab7.horVerao.mesFimHorVerao   = abntBcdToByte(respLeParam->
                                                    horVerao.mesFimHorVerao);

            //salva como modelo, os 2 bytes BCD recebidos
            modeloLido = TOGGLEENDIAN16(respLeParam->modeloMedidor);
            ///DEBUG
            //SerialDebug.println("Modelo lido: " + String(modeloLido, HEX));

            if (localKeys.modeloMedidor != modeloLido)
            {
                localKeys.modeloMedidor = modeloLido;
                savedKeys.write(localKeys);
            }
        }
        break;

        case ID_CMD23:
        {
            abnt_resp_leitura_regs_atuais_t *respLeRegsAtuais;

            respLeRegsAtuais = (abnt_resp_leitura_regs_atuais_t *)
                buffRespRecebida;
            ansi_tab6.numSerieMedidor = abntBcdToInt((uint8_t *)
                            &(respLeRegsAtuais->num_serie_registrador), 4);
            if (CANAL_123 == canaisVisiveis)
            {
                ansi_tab1.enAtivaTotDireto = abntBcdToInt(
                    respLeRegsAtuais->totalGeralCanal1, 5);
                ansi_tab1.enAtivaPontaDireto = abntBcdToInt(
                    respLeRegsAtuais->totalCanal1Ponta, 5);
                ansi_tab1.enAtivaIntermDireto = abntBcdToInt(
                    respLeRegsAtuais->totalCanal1Interm, 5);
                ansi_tab1.enAtivaForaDireto = abntBcdToInt(
                    respLeRegsAtuais->totalCanal1Fora, 5);
                ansi_tab1.enAtivaReservDireto = abntBcdToInt(
                    respLeRegsAtuais->totalCanal1Reserv, 5);
                ansi_tab1.enReativaTotDireto = abntBcdToInt(
                    respLeRegsAtuais->totalGeralCanal2, 5);
                ansi_tab1.demAtivaPonta = abntBcdToInt(
                    respLeRegsAtuais->demMaxCanal1Ponta, 5);
                ansi_tab1.demAtivaFora = abntBcdToInt(
                    respLeRegsAtuais->demMaxCanal1Fora, 5);
                ansi_tab1.demAtivaInterm = abntBcdToInt(
                    respLeRegsAtuais->demMaxCanal1Interm, 5);
                ansi_tab1.demAtivaReserv = abntBcdToInt(
                    respLeRegsAtuais->demMaxCanal1Reserv, 5);
                ansi_tab3.enReativaCapTot = abntBcdToInt(
                    respLeRegsAtuais->totalGeralCanal3, 5);
                ansi_tab5.enReativaIndPonta = abntBcdToInt(
                    respLeRegsAtuais->totalCanal2Ponta, 5);
                ansi_tab5.enReativaIndFora = abntBcdToInt(
                    respLeRegsAtuais->totalCanal2Fora, 5);
                ansi_tab5.enReativaIndReserv = abntBcdToInt(
                    respLeRegsAtuais->totalCanal2Reserv, 5);
                ansi_tab5.enReativaCapPonta = abntBcdToInt(
                    respLeRegsAtuais->totalCanal3Ponta, 5);
                ansi_tab5.enReativaCapFora = abntBcdToInt(
                    respLeRegsAtuais->totalCanal3Fora, 5);
                ansi_tab5.enReativaCapReserv = abntBcdToInt(
                    respLeRegsAtuais->totalCanal3Reserv, 5);
                ansi_tab6.demReativaIndPonta = abntBcdToInt(
                    respLeRegsAtuais->demMaxCanal2Ponta, 5);
                ansi_tab6.demReativaIndFora = abntBcdToInt(
                    respLeRegsAtuais->demMaxCanal2Fora, 5);
                ansi_tab6.demReativaIndReserv = abntBcdToInt(
                    respLeRegsAtuais->demMaxCanal2Reserv, 5);
                ansi_tab6.demReativaCapPonta = abntBcdToInt(
                    respLeRegsAtuais->demMaxCanal3Ponta, 5);
                ansi_tab6.demReativaCapFora = abntBcdToInt(
                    respLeRegsAtuais->demMaxCanal3Fora, 5);
                ansi_tab6.demReativaCapReserv = abntBcdToInt(
                    respLeRegsAtuais->demMaxCanal3Reserv, 5);

                if (localKeys.numSerieMedidor != ansi_tab6.numSerieMedidor)
                {
                    localKeys.numSerieMedidor = ansi_tab6.numSerieMedidor;
                    savedKeys.write(localKeys);
                }
                //prepara para ler os outros canais
                insereCmdABNT(LISTA_NORMAL, ID_CMD21, CANAL_456);
                insereCmdABNT(LISTA_NORMAL, ID_CMD23);
            }

            else if (CANAL_456 == canaisVisiveis)
            {
                ansi_tab2.enAtivaTotReverso = abntBcdToInt(
                    respLeRegsAtuais->totalGeralCanal1, 5);
                ansi_tab2.enAtivaPontaReverso = abntBcdToInt(
                    respLeRegsAtuais->totalCanal1Ponta, 5);
                ansi_tab2.enAtivaIntermReverso = abntBcdToInt(
                    respLeRegsAtuais->totalCanal1Interm, 5);
                ansi_tab2.enAtivaForaReverso = abntBcdToInt(
                    respLeRegsAtuais->totalCanal1Fora, 5);
                ansi_tab2.enAtivaReservReverso = abntBcdToInt(
                    respLeRegsAtuais->totalCanal1Reserv, 5);
                //Volta aos canais padrao
                insereCmdABNT(LISTA_NORMAL, ID_CMD21, CANAL_123);
            }
        }
        break;

        case ID_CMD25:
        {
            abnt_resp_leitura_falta_energia_t *respLeFaltaEnergia;
            uint8_t i, j;
            for (i = 0, j = 19; i < 5; ++i, --j)
            {
                respLeFaltaEnergia = (abnt_resp_leitura_falta_energia_t *)
                    buffRespRecebida;
                datahora_tm.tm_hour = abntBcdToByte(respLeFaltaEnergia->
                                                    faltas[j].horaFalta);
                datahora_tm.tm_min  = abntBcdToByte(respLeFaltaEnergia->
                                                    faltas[j].minutoFalta);
                datahora_tm.tm_sec  = abntBcdToByte(respLeFaltaEnergia->
                                                    faltas[j].segundoFalta);
                datahora_tm.tm_mday = abntBcdToByte(respLeFaltaEnergia->
                                                    faltas[j].diaFalta);
                datahora_tm.tm_mon  = (abntBcdToByte(respLeFaltaEnergia->
                                                    faltas[j].mesFalta)) - 1;
                datahora_tm.tm_year = (abntBcdToByte(respLeFaltaEnergia->
                                                    faltas[j].anoFalta)) + 100;
                datahora = mktime(&datahora_tm);
                ansi_tab4.dataHoraIniFalta[i] = datahora;
                datahora_tm.tm_hour = abntBcdToByte(respLeFaltaEnergia->
                                                    faltas[j].horaRet);
                datahora_tm.tm_min  = abntBcdToByte(respLeFaltaEnergia->
                                                    faltas[j].minutoRet);
                datahora_tm.tm_sec  = abntBcdToByte(respLeFaltaEnergia->
                                                    faltas[j].segundoRet);
                datahora_tm.tm_mday = abntBcdToByte(respLeFaltaEnergia->
                                                    faltas[j].diaRet);
                datahora_tm.tm_mon  = (abntBcdToByte(respLeFaltaEnergia->
                                                     faltas[j].mesRet)) - 1;
                datahora_tm.tm_year = (abntBcdToByte(respLeFaltaEnergia->
                                                    faltas[j].anoRet)) + 100;
                datahora = mktime(&datahora_tm);
                ansi_tab4.dataHoraFimFalta[i] = datahora;
            }
        }
        break;

        case ID_CMD28:
        {
            abnt_resp_leitura_reg_alt_t *respLeRegAlter;
            uint8_t i, j;
            uint32_t numeroLeitor;

            respLeRegAlter = (abnt_resp_leitura_reg_alt_t *)buffRespRecebida;
            for (i = 0, j = 15; i < 5; ++i, --j)
            {
                datahora_tm.tm_hour = abntBcdToByte(respLeRegAlter->
                                                    alteracoes[j].hora);
                datahora_tm.tm_min  = abntBcdToByte(respLeRegAlter->
                                                    alteracoes[j].minuto);
                datahora_tm.tm_sec  = abntBcdToByte(respLeRegAlter->
                                                    alteracoes[j].segundo);
                datahora_tm.tm_mday = abntBcdToByte(respLeRegAlter->
                                                    alteracoes[j].dia);
                datahora_tm.tm_mon  = (abntBcdToByte(respLeRegAlter->
                                                    alteracoes[j].mes)) - 1;
                datahora_tm.tm_year = (abntBcdToByte(respLeRegAlter->
                                                    alteracoes[j].ano)) + 100;
                datahora = mktime(&datahora_tm);
                ansi_tab9.regAlteracoes[i].dataHoraAlteracao = datahora;
                ansi_tab9.regAlteracoes[i].codAlteracao      = abntBcdToByte
                                        (respLeRegAlter->alteracoes[j].codigo);
                numeroLeitor = (respLeRegAlter->alteracoes[j].numLeitor);
                ansi_tab9.regAlteracoes[i].numSerieLeitor   = abntBcdToInt
                                                ((uint8_t *)&numeroLeitor, 3);
            }
        }
        break;

        case ID_CMD39:
        {
            abnt_resp_generic_t *cmd39;
            cmd39 = (abnt_resp_generic_t *)buffRespRecebida;

            // caso o medidor nao implemente 87, le cod consumidor (cmd80)
            if (ID_CMD87 == cmd39->payload[0])
            {
                insereCmdABNT(LISTA_NORMAL, ID_CMD80);
                semCmd87 = true;
            }
        }
        break;

        case ID_CMD40:
        {
            abnt_resp_generic_t *cmd40;
            cmd40 = (abnt_resp_generic_t *)buffRespRecebida;

            /// ***** -- VERIFICAR O COMPORTAMENTO DESEJADO!!! --
            if (ABNT_OCORRENCIA43 != cmd40->payload[0])
            //Nao envia CMD37 para limpar ocorrencia, caso seja ocorrencia 43,
            //indicando necessidade de abertura de sessao ou parametrizacao
            //incompleta
            {
                ansi_tab5.cod_ocorrencia = cmd40->payload[0];
                ansi_tab5.subcod_ocorrencia = cmd40->payload[1];
                insereTabelaANSI(TAB_ANSI05, SIZE_TABELA5_CMD40);
                abntOcorrencia = ansi_tab5.cod_ocorrencia;
                //Envia cmd37 pra limpar cond de ocorrencia
                insereCmdABNT(LISTA_URGENTE, ID_CMD37);
            }

            else //ocorrencia 43
                 //Abre a sessao com senha e reenvia o mesmo comando
            {
                cmdAtrasado = buscaUltimoCmd();
                //Nao envia novo CMD11 se receber erro de senha
                if ((cmdAtrasado.cmd != ID_CMD11) &&
                    (cmdAtrasado.cmd != ID_CMD13))
                {
                    insereCmdABNT(LISTA_URGENTE, ID_CMD13);
                    insereCmdABNT(LISTA_URGENTE, ID_CMD11);
                }
                else //senha incorreta
                {
                    ansi_tab5.alarmes.senha_abnt = 1;
                    insereTabelaANSI(TAB_ANSI05, SIZE_TABELA5_CMD40);
                }
            }
        }
        break;

        case ID_CMD80:
        {
            abnt_resp_leitura_param_medicao_t *cmd80;
            cmd80 = (abnt_resp_leitura_param_medicao_t *)buffRespRecebida;

            ansi_tab7.postoHorario[INTERMEDIARIO].horaInicio1 = abntBcdToByte
                (cmd80->inicioPostoHorarioInter.horaInicio1);
            ansi_tab7.postoHorario[INTERMEDIARIO].minutoInicio1 = abntBcdToByte
                (cmd80->inicioPostoHorarioInter.minutoInicio1);
            ansi_tab7.postoHorario[INTERMEDIARIO].horaInicio2 = abntBcdToByte
                (cmd80->inicioPostoHorarioInter.horaInicio2);
            ansi_tab7.postoHorario[INTERMEDIARIO].minutoInicio2 = abntBcdToByte
                (cmd80->inicioPostoHorarioInter.minutoInicio2);
            ansi_tab7.postoHorario[INTERMEDIARIO].horaInicio3 = abntBcdToByte
                (cmd80->inicioPostoHorarioInter.horaInicio3);
            ansi_tab7.postoHorario[INTERMEDIARIO].minutoInicio3 = abntBcdToByte
                (cmd80->inicioPostoHorarioInter.minutoInicio3);
            ansi_tab7.postoHorario[INTERMEDIARIO].horaInicio4 = abntBcdToByte
                (cmd80->inicioPostoHorarioInter.horaInicio4);
            ansi_tab7.postoHorario[INTERMEDIARIO].minutoInicio4 = abntBcdToByte
                (cmd80->inicioPostoHorarioInter.minutoInicio4);

            if (semCmd87)
            {
                memcpy(ansi_tab6.codInstalacao, cmd80->codConsumidor,
                                                TAM_COD_INSTALACAO);
            }
        }
        break;

        case ID_CMD87:
        {
            abnt_resp_cod_instalacao_t *cmd87;
            cmd87 = (abnt_resp_cod_instalacao_t *)buffRespRecebida;

            memcpy(ansi_tab6.codInstalacao, cmd87->codInstalacao,
                                            TAM_COD_INSTALACAO);
            semCmd87 = false;
        }
        break;

        case ID_CMDE2:
        {
            switch (buffRespRecebida->payload[0]) //Cod estendido
            {
            case CMD_LANDIS_DRP_DRC:
            {
                abnt_landis_resp_drp_drc_t *respLeDrpDrc;

                respLeDrpDrc = (abnt_landis_resp_drp_drc_t *)buffRespRecebida;
                ansi_tab3.drp = max3(respLeDrpDrc->drp_drc[0].drp0,
                                     respLeDrpDrc->drp_drc[0].drp1,
                                     respLeDrpDrc->drp_drc[0].drp2);
                ansi_tab3.drc = max3(respLeDrpDrc->drp_drc[0].drc0,
                                     respLeDrpDrc->drp_drc[0].drc1,
                                     respLeDrpDrc->drp_drc[0].drc2);
            }
            break;

            case CMD_LANDIS_LEITURA_PARAM:
            {
                abnt_landis_resp_le_param_t *respLeParamLandis;

                respLeParamLandis = (abnt_landis_resp_le_param_t *)
                                                            buffRespRecebida;
                ansi_tab1.posto_horario = respLeParamLandis->postoAtual;
            }
            break;

            case CMD_LANDIS_ANTIFRAUDE:
            {
                abnt_landis_resp_leitura_monitores_t *respLeMonitoresLandis;
                uint8_t i_mon;
                bool tem_alarme;

                respLeMonitoresLandis =
                    (abnt_landis_resp_leitura_monitores_t *)buffRespRecebida;
                i_mon = 0;
                tem_alarme = false;
                while (respLeMonitoresLandis->monitores[i_mon].id)
                {
                    tem_alarme = true;
                    // switch (respLeMonitoresLandis->monitores[i_mon].id)
                    // {
                    //     //case

                    //     break;
                    // }
                    ++i_mon;
                }
                if (tem_alarme)
                    insereTabelaANSI(TAB_ANSI05, SIZE_TABELA5_SO_ALARMES);
            }
            break;

            default:
                break;
            }
        }
        break;

        default:
            break;
        }

        ///// --- DEBUG
        // for (uint16_t i = 0; i < 258; i++)
        // {
        //     SerialDebug.print(String(pBuffABNTrecv[i], HEX) + "\t");
        // }
        // SerialDebug.println();
        /////

        abntRespostaTratada();
    }
} //trataRespABNT(

/**
* @brief Funcao para inicilializar o HW
*
* Esta funcao inicializa os pinos para suas funcoes especificas
* e configura o hardware da placa MKRWAN para o devido uso, inclusive
* as portas seriais.
*
************************************************************************/
void initHW(void)
{
    pinPeripheral(1, PIO_SERCOM);
    pinPeripheral(0, PIO_SERCOM);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(SERIAL_RTS, OUTPUT);
    digitalWrite(SERIAL_RTS, LOW); //Low TTL = +5,5V RS232
    digitalWrite(LED_BUILTIN, LOW);

    pinMode(PIN_A5, INPUT);
    pinMode(PIN_A6, INPUT);
    uint32_t rd1 = analogRead(PIN_A5);
    uint32_t rd2 = analogRead(PIN_A6);
    rd1 = ((rd1 * rd2 * 0xA5) ^ rd1) % 0xFFFF;
    //SerialDebug.println("Seed: " + String(rd1));
    randomSeed(rd1);

    Serial1.begin(ABNT_BAUD);
    Serial1.setTimeout(250);
    SerialDebug.begin(9600);

    SerialDebug.println("Start...");

    return;
}

void initComissionamento(void)
{
    memset(&localKeys, 0, sizeof(localKeys));
    if (protocoloComissionamento())
    {
        // Se acabou de ser comissionado,
        // usa valores predefinidos para todos parametros
        t0_LoRa = random(NUM_SEG_DIA); //65;
        intervaloLoRa = 1;
        localKeys.seed = t0_LoRa;
        localKeys.intervalo = intervaloLoRa;
        localKeys.comissionado = FLASH_VALID; //indica que tem valores validos
        localKeys.sessao_ok = false; // indica que nao ha sessao valida
        savedKeys.write(localKeys);
        SerialDebug.println("Chave recebida!");
    }
    SerialDebug.println("local secrets:");
    SerialDebug.println("EUI: " + String((char *)(localKeys.appeui)));
    SerialDebug.println("Key: " + String((char *)(localKeys.appkey)));
}// initComissionamento(

bool conectaLoRa(startType_t tipo)
{
#ifdef SEMLORA
    SerialDebug.println("Executando SEM LoRa!");
#else
    tLastConectaLoRa = millis();
    // Interrompe a maq de estados ABNT e forca desconexao da porta serial.
    stateABNT = ABNT_STATE_DISCONNECTED;
    portaSerial.interface = SERIAL_NULL;

    SerialDebug.print("Start join... ");
    tipo ? SerialDebug.println("Warm !") : SerialDebug.println("COLD !");

    uint8_t nJoin = MAX_TENTATIVAS_JOIN;
    uint8_t nInit = MAX_TENTATIVAS_INIT;
    bool joinLoRa = false;
    bool init = false;

    modem.restart();

    while (nJoin--)
    {
        if (tipo == WARM_START)
            joinLoRa = modem.joinABP(localKeys.devaddr, localKeys.nwkskey,
                                     localKeys.appskey);
        else
            joinLoRa = modem.fitecJoinOTAA(200000L, 3, localKeys.appeui,
                                           localKeys.appkey);
    }

    if (joinLoRa)
    {
        configuraSinalizacao(ST_AGUARDA_ACK, TIMER_STSEMACK);
        while (nInit--)
        {
            init = modem.initTransmit();
            delay(500);
        }
    }

    if (init)
    {
        updateTLoRaSend();
        modem.dutyCycle(false);
        delay(300);
        SerialDebug.println("Transmissoes sincronizadas com ACK!");
        ////// Teste de transmissao ao inicializar!
        insereTabelaANSI(TAB_ANSI01);
        insereTabelaANSI(TAB_ANSI02);
        insereTabelaANSI(TAB_ANSI03);
        insereTabelaANSI(TAB_ANSI04);
        gravaSessaoLoRa();
        sinalizaStatus = ST_NENHUM;
        return true;
    }

    else
    {
        SerialDebug.println("FITec join falhou!");
        if (localKeys.sessao_ok)
        {
            localKeys.sessao_ok = false;
            savedKeys.write(localKeys);
        }
        //piscaLed(3, 200, 300);
    }
#endif
    return false;
} //conectaLoRa(

void gravaSessaoLoRa(void)
{
    SerialDebug.println(" --Grava sessao!\r\n");

    // char senhaGravada[11];
    // for (uint8_t i = 0; i<10; i++)
    //     senhaGravada[i] = localKeys.senhaABNT[i];
    // senhaGravada[10] = 0;
    // SerialDebug.println("senha ABNT antes: " + String(senhaGravada));
    // SerialDebug.println();
    // SerialDebug.flush();

    String nwkS = modem.getNwkSKey();
    String appS = modem.getAppSKey();
    String devA = modem.getDevAddr();
    strcpy(localKeys.nwkskey, nwkS.c_str());
    strcpy(localKeys.appskey, appS.c_str());
    strcpy(localKeys.devaddr, devA.c_str());
    localKeys.sessao_ok = FLASH_VALID;

    savedKeys.write(localKeys);

    // for (uint8_t i = 0; i<10; i++)
    //     senhaGravada[i] = localKeys.senhaABNT[i];
    // senhaGravada[10] = 0;
    // SerialDebug.println("senha ABNT depois: " + String(senhaGravada));
    // SerialDebug.println();
    // SerialDebug.flush();

} //gravaSessaoLoRa(

/**
* @brief Interpretacao de comandos para definicao de senha de conexao LoRa
*
* Funcao para interpretar comandos que podem ser enviados ao se ligar o end
* device. Estes comandos definem os parametros AppEUI e APPKey usados para o
* join no modo OTAA. Estes comandos devem ser enviados como texto (ASCII) nos
* primeiros segundos ao se ligar o end device. O formato dos comandos é:
*   appeui=1234567890123456
*   appkey=12345678901234567890123456789012
*
* @return retorna TRUE se recebeu algum comando
************************************************************************/
bool protocoloComissionamento(void)
{
    //piscaLed(5, 150, 150);

    char comando[40];
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

    SerialUSB.println("Envie os comandos AQUI !!!\r\n\r\nEx.\r\n"
                       "appeui=[valor]\r\nappkey=[valor]\r\n");
    SerialDebug.println("Porta de debug ...\r\n Nao preparada para "
                         "receber comandos");

    if (!modem.begin(AU915))
        Serial.println("Falha no modem LoRa!");

    // Informativo para operador saber o EUI e programar a senha correspondente
    SerialUSB.print("Your module version is: ");
    SerialUSB.println(modem.version());
    SerialUSB.print("Your device EUI is: ");
    SerialUSB.println(modem.deviceEUI() + "\r\n");

    SerialDebug.flush();
    SerialUSB.flush();
    piscaLed(3, 300, 50);
    tStart = millis();
    while ((millis() - tStart) < T_COMANDO)
    {
        if (SerialUSB.available())
        {
            if ('A' == toUpperCase(SerialUSB.peek()))
            {
                uint8_t tamMsg = SerialUSB.readBytes(comando, 40);
                comando[tamMsg] = 0;
                char *msgPrefix = comando;
                char *msgParam = strchr(comando, '=');
                if (msgParam)
                    *msgParam = 0;

                if ((msgPrefix != NULL) && (msgPrefix[0]))
                {
                    //Posiciona no parametro, apos ter quebrado a string
                    msgParam++;
                    if (String(msgPrefix) == "appeui")
                    {
                        memcpy(localKeys.appeui, msgParam, sizeof(localKeys.
                                                                  appeui));
                        SerialDebug.println("Rec EUI: " + String(msgParam));
                        SerialUSB.println("Rec EUI: " + String(msgParam));
                        piscaLed(2, 100, 50);
                        recebido = true;
                    }
                    else if (String(msgPrefix) == "appkey")
                    {
                        memcpy(localKeys.appkey, msgParam, sizeof(localKeys.
                                                                  appkey));
                        SerialDebug.println("Rec KEY: " + String(msgParam));
                        SerialUSB.println("Rec KEY: " + String(msgParam));
                        piscaLed(2, 100, 50);
                        recebido = true;
                    }
                    else
                    {
                        SerialDebug.println("Msg: " + String(msgPrefix[0],
                                                             HEX));
                        SerialUSB.println("Msg: " + String(msgPrefix[0], HEX));
                        piscaLed(2, 1000, 200);
                        SerialDebug.println("Comando nao implementado!");
                        SerialUSB.println("Comando nao implementado!");
                        //SerialUSB.readBytes(comando, 40);
                    }
                    tStart = millis();
                }
            }
            else
            {
                SerialUSB.read(); //despreza o caractere indesejado
                tStart = millis();
            }
        }
    }
    piscaLed(3, 1500, 250);
    SerialUSB.println("Tempo esgotado !!!");
    return (recebido);

} //protocoloComissionamento(

/**
* @brief Funcao para medir a temperatura interna do processador
*
* Esta funcao le a temperatura interna do processador e converte para
* milesimos de grau Celsius. Sao usados os valores de calibracao de fabrica
* para corrigir a leitura.
* ATENCAO: A conversao e baseada numa espera ocupada, aguardando o fim do ADC,
* assim a funcao leva aproximadamente 210 milissegundos para terminar.
*
* @return Temperatura interna em milesimos de grau Celsius
************************************************************************/
uint32_t medeTemperatura(void)
{
    auto tempInterna = temp_raw_to_mdeg(temp_read_raw());
    tempInterna = (((float)tempInterna/1000) * (1<<14));
    if (tempInterna > localMaxTemp)
        localMaxTemp = tempInterna;

    tLastTemp = millis();
    //SerialDebug.println("\r\nTemperatura medida: " + String(tempInterna));
    return localMaxTemp;
} //medeTemperatura(

void trataNovoIntervalo(void)
{
    if (novoIntervaloLoRa)
    {
        // SerialDebug.println("Novo interv: " + String(novoIntervaloLoRa));
        intervaloLoRa = novoIntervaloLoRa;
        montaAgenda();
        savedKeys.write(localKeys);
        // SerialDebug.println("\r\nFim MontaAgenda!\r\n");
        // SerialDebug.flush();
        iniciaAlarmes();
        // SerialDebug.println("\r\nFim iniciaAlarmes\r\n");
        // SerialDebug.flush();
        // imprimeAgenda();
        novoIntervaloLoRa = 0;
    }
} //trataNovoIntervalo(

void inicializaSinalizacao(uint32_t sampleRate, tipoTimer_t tipo)
{
    tcConfigure(sampleRate, tipo);
    tcStartCounter();
}

void trataSinalizacaoSucesso(void)
{
    if (!LoRaOK)
    {
        if (!comunicouABNT)
            configuraSinalizacao(ST_SEMCOMUNIC, TIMER_STSEMCOMUNIC);
        else
            configuraSinalizacao(ST_SEMLORA, TIMER_STSEMLORA);
    }
    else
    {
        if (!comunicouABNT)
            configuraSinalizacao(ST_SEMABNT, TIMER_STSEMABNT);
        else
            configuraSinalizacao(ST_NORMAL, TIMER_STNORMAL);
    }
} //trataSinalizacaoSucesso(

void configuraSinalizacao(sinalizaStatus_t status,
                          uint32_t sampleRate, tipoTimer_t tipo)
{
    if (sinalizaStatus == status)
        return;
    sinalizaStatus = status;
    ledSinalizacao = false;
    contPulso = 0;
    inicializaSinalizacao(sampleRate, tipo);
} //configuraSinalizacao(

void maqEstSinalizacao(void)
{
    switch (sinalizaStatus)
    {
    case ST_NENHUM:
        digitalWrite(LED_BUILTIN, LOW);
        tcStopCounter();
        break;

    case ST_NCOMISSIONADO:
        trataSinalizacaoNComissionado();
        break;

    case ST_AGUARDA_ACK:
        trataSinalizacaoAguardaAck();
        break;

    case ST_SEMCOMUNIC:
        trataSinalizacaoSemComunic();
        break;

    case ST_SEMLORA:
        trataSinalizacaoSemLoRa();
        break;

    case ST_SEMABNT:
        trataSinalizacaoSemABNT();
        break;

    case ST_NORMAL:
        trataSinalizacaoOpNormal();
        break;

    default:
        break;
    }
} //maqEstSinalizacao(

void trataSinalizacaoNComissionado()
{
    digitalWrite(LED_BUILTIN, ledSinalizacao);
    if (ledSinalizacao)
        inicializaSinalizacao(50);
    else
        inicializaSinalizacao(1000);

    ledSinalizacao = !ledSinalizacao;
} //trataSinalizacaoNComissionado(

void trataSinalizacaoAguardaAck()
{
if (!contPulso)
    {
        inicializaSinalizacao(300);
    }
    digitalWrite(LED_BUILTIN, ledSinalizacao);
    ledSinalizacao = !ledSinalizacao;
    if (++contPulso > NUM_INV_SEMACK)
    {
        inicializaSinalizacao(1200);
        ledSinalizacao = false;
        contPulso = 0;
    }
}//trataSinalizacaoAguardaAck()

void trataSinalizacaoSemComunic()
{
    digitalWrite(LED_BUILTIN, ledSinalizacao);
    ledSinalizacao = !ledSinalizacao;
} //trataSinalizacaoSemComunic(

void trataSinalizacaoSemLoRa()
{
    digitalWrite(LED_BUILTIN, ledSinalizacao);
    ledSinalizacao = !ledSinalizacao;
} //trataSinalizacaoSemLoRa(

void trataSinalizacaoSemABNT()
{
    if (!contPulso)
    {
        inicializaSinalizacao(150);
    }
    digitalWrite(LED_BUILTIN, ledSinalizacao);
    ledSinalizacao = !ledSinalizacao;
    if (++contPulso > NUM_INV_INTERMITENTE)
    {
        inicializaSinalizacao(1200);
        ledSinalizacao = false;
        contPulso = 0;
    }
} //trataSinalizacaoSemABNT(

void trataSinalizacaoOpNormal()
{
    if (ledSinalizacao)
    {
        digitalWrite(LED_BUILTIN, ledSinalizacao);
        inicializaSinalizacao(50);
        contPulso = 0;
        ledSinalizacao = !ledSinalizacao;
    }
    else
    {
        digitalWrite(LED_BUILTIN, ledSinalizacao);
        inicializaSinalizacao(1000);
        if (++contPulso > NUM_SEG_SINALIZANORMAL)
        {
            ledSinalizacao = !ledSinalizacao;
            inicializaSinalizacao(100);
        }
    }
} //trataSinalizacaoOpNormal(

#endif //TESTE_INTEGRACAO

///////////////////////////////////////////////////////////////
//-----------------------------------------------------------//
#ifdef TESTE_FIMAFIM

#include "abnt.h"
#include "otica_ftdi.h"
#include "serial_nativa.h"
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
                memcpy(localloRaBuffPt + localloRaBuffSize,
                       (uint8_t *)&localAnsi_tab3, SIZE_TABELA3);
                localloRaBuffSize += SIZE_TABELA3;
                localLoRaBuff[localloRaBuffSize] = calcSum(localloRaBuffPt,
                                                           localloRaBuffSize);
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
            memcpy(localloRaBuffPt + localloRaBuffSize,
                   (uint8_t *)dadosTabela[tabelaAtual].tabela,
                   dadosTabela[tabelaAtual].size);
            localloRaBuffSize += dadosTabela[tabelaAtual].size;
            localLoRaBuff[localloRaBuffSize] = calcSum(localloRaBuffPt+4,
                                                       localLoRaBuff[2]);
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

    SerialDebug.println(" ----> Enviando JOIN OTAA ! <----");
    int connected = modem.joinOTAA(localKeys.appeui, localKeys.appkey);
    //int connected = modem.joinABP (localDeviceAddr, localNetworkSKey, localAppSessionKey); //   ABP_DEVADDR, ABP_NWKSKEY, ABP_APPSKEY);

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
    localAnsi_tab7.postoHorario[INTERMEDIARIO].horaInicio2 = 0x17;
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
* Funcao para interpretar comandos que podem ser enviados ao se ligar o end
* device. Estes comandos definem os parametros AppEUI e APPKey usados para o
* join no modo OTAA. Estes comandos devem ser enviados como texto (ASCII) nos
* primeiros segundos ao se ligar o end device. O formato dos comandos é:
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
                            memcpy(localKeys.appeui, (uint8_t *)&paramRec,
                                   sizeof(localKeys.appeui));
                            piscaLed(2, 100, 50);
                            SerialDebug.println("Rec EUI: " + String(
                                                            (char *)paramRec));
                            tStart = millis();
                            recebido = true;
                        }
                    }
                    else if (String((char *)cmdRec) == "appkey")
                    {
                        if (32 == SerialUSB.readBytes((char *)paramRec, 32))
                        {
                            paramRec[32] = 0;
                            memcpy(localKeys.appkey, (uint8_t *)&paramRec,
                                   sizeof(localKeys.appkey));
                            piscaLed(2, 100, 50);
                            SerialDebug.println("Rec KEY: " + String(
                                                            (char *)paramRec));
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



#ifdef SEND_RECEIVE
/*
  Lora Send And Receive
  This sketch demonstrates how to send and receive data with the
  MKR WAN 1300 LoRa module.
  This example code is in the public domain.
*/

//#include <MKRWAN.h>

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

#ifdef TESTE_JOIN

//#include <MKRWAN.h>
#include "wiring_private.h"

//Habilitacao de uma nova UART, Serial3, nos pinios 0, PA22 (Tx) e 1, PA23 (Rx)
Uart Serial3(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM3_Handler()
{
    Serial3.IrqHandler();
}
#define SerialDebug SerialUSB

#define SECRET_APP_EUI "70B3D57ED0012777"
//#define SECRET_APP_KEY "f4092d86e075ad28fc6c04e0973e8d4d" // - dev. 0115 | EUI: ...32467316
//#define SECRET_APP_KEY "2ca6c21ec44c45984ee111534f5f355c" // - dev. 0105 | EUI: ...393C7A0B
//#define SECRET_APP_KEY "27ca801eadbd2b30b52c64581b4b37a4" // - dev. debugger | EUI: ...39476906
#define SECRET_APP_KEY "c11b6c1d71a26eca4dbb1cf7e0722882" // - dev. MKR avulso EUI: ...3923750B
LoRaModem modem;
bool joinOk = false;
uint32_t lastTry;

char appEui[17] = SECRET_APP_EUI;
char appKey[33] = SECRET_APP_KEY;
static const String deviceAddr = ABP_DEVADDR;
static const String NetworkSKey = ABP_NWKSKEY;
static const String AppSessionKey = ABP_APPSKEY;

void reiniciaLoRa(void);
void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    pinPeripheral(1, PIO_SERCOM);
    pinPeripheral(0, PIO_SERCOM);

    pinMode(PIN_TEST_ISR, INPUT_PULLDOWN);
    //attachInterrupt(digitalPinToInterrupt(PIN_TEST_ISR), reiniciaLoRa, RISING);

    SerialDebug.begin(9600);
    //USB
    uint32_t tTryUSB = millis();
    while (!SerialUSB)
    {
        if ((millis() - tTryUSB) > 5000)
            break;
    }
    SerialDebug.println("Start...");
    SerialDebug.println("Caracterizacao da API!\r\n\r\n");

    if (!modem.begin(AU915))
    {
        SerialDebug.println("!!! ERRO no modem.begin() !!! \r\n\r\n");
        while (1) ;
    }
    delay(500);

    //-------- DEBUG ----------
    SerialDebug.print("Your module version is: ");
    SerialDebug.println(modem.version());
    SerialDebug.print("Your device EUI is: ");
    SerialDebug.println(modem.deviceEUI());
    SerialDebug.println("App EUI is: " + String(appEui));
    SerialDebug.println("App key is: " + String(appKey));
    //-------- DEBUG ----------

    if (modem.publicNetwork(false))
        SerialDebug.println("Private mode Ok !");
    else
    {
        SerialDebug.println("Private mode ERROR!");
    }
    delay(300);

    SerialDebug.print(String(millis() / 1000) + ": ");
    SerialDebug.println("Tentando 1o. JOIN");
    //int connected = modem.joinABP(deviceAddr, NetworkSKey, AppSessionKey);
    int connected = modem.joinOTAA(appEui, appKey);
    if (connected)
    {
        SerialDebug.println("JOIN OTAA com sucesso");
        joinOk = true;
    }
    else
    {
        SerialDebug.println("Erro no JOIN!");
        reiniciaLoRa();
        delay(1000);
    }
    lastTry = millis();
}

void loop()
{
    static uint8_t testLoRa[5] = {0xDE, 0xAD, 0xBE, 0xEF, 0x00};
    String buf;

    if (joinOk)
    {
        SerialDebug.print(String(millis() / 1000) + ": ");
        SerialDebug.println("Conectado, enviando pacote");
        int err;
        testLoRa[4]++;
        modem.beginPacket();
        modem.write(testLoRa, 5);
        err = modem.endPacket(false);
        SerialDebug.print(String(millis() / 1000) + ": ");
        SerialDebug.println("Send retornou: " + String(err));
        delay(60000L);
    }
    else
    {
        if ((millis() - lastTry) > 120000)
        {
            SerialDebug.print("\r\n" + String(millis() / 1000) + ": ");
            SerialDebug.println("Tentando novo JOIN...");
            int connected = modem.joinOTAA(appEui, appKey);
            SerialDebug.println("joinOTAA ret: " + String(connected));
            if (connected)
            {
                SerialDebug.print(String(millis() / 1000) + ": ");
                SerialDebug.println("JOIN OTAA com sucesso");
                joinOk = true;
            }
            else
            {
                SerialDebug.print(String(millis() / 1000) + ": ");
                SerialDebug.println("Erro no JOIN!");
                reiniciaLoRa();
                delay(1000);
            }
            lastTry = millis();
        }
    }
    if (digitalRead(PIN_TEST_ISR))
    {
        joinOk = false;
        reiniciaLoRa();
        delay(3000);
    }
}

void reiniciaLoRa(void)
{
    SerialDebug.print("\r\n" + String(millis() / 1000) + ": ");
    SerialDebug.println("  -- MODEM RESTART !!! --\r\n\r\n");
    if (!modem.restart())
    {
        SerialDebug.println("Erro ao tentar reiniciar!\r\n\r\n");
        return;
    }
    modem.begin(AU915);
    delay(500);
    modem.publicNetwork(false);
}
#endif

#ifdef TEST_TERMINAL_AT

//#include <MKRWAN.h>
#include "wiring_private.h"

//Habilitacao de uma nova UART, Serial3, nos pinios 0, PA22 (Tx) e 1, PA23 (Rx)
Uart Serial3(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM3_Handler()
{
    Serial3.IrqHandler();
}
#define SerialDebug SerialUSB

#define SECRET_APP_EUI "70B3D57ED0012777"
//#define SECRET_APP_KEY "f4092d86e075ad28fc6c04e0973e8d4d" // - dev. 0115 | EUI: ...32467316
#define SECRET_APP_KEY "2ca6c21ec44c45984ee111534f5f355c" // - dev. 0105 | EUI: ...393C7A0B
//#define SECRET_APP_KEY "27ca801eadbd2b30b52c64581b4b37a4" // - dev. debugger | EUI: ...39476906
//#define SECRET_APP_KEY "c11b6c1d71a26eca4dbb1cf7e0722882" // - dev. MKR avulso EUI: ...3923750B

#define ABP_DEVADDR     "0030b79c"
#define ABP_NWKSKEY     "349eca63ffc23f220ee9429d019c0c08"
#define ABP_APPSKEY     "bbdf2e1a7331cb1ed19039d9b54b8119"

FITecModem modem;
bool joinOk = false;
uint32_t lastTry;

char appEui[17] = SECRET_APP_EUI;
char appKey[33] = SECRET_APP_KEY;
static const String deviceAddr = ABP_DEVADDR;
static const String NetworkSKey = ABP_NWKSKEY;
static const String AppSessionKey = ABP_APPSKEY;

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    pinPeripheral(1, PIO_SERCOM);
    pinPeripheral(0, PIO_SERCOM);

    SerialDebug.begin(9600);
    //USB
    uint32_t tTryUSB = millis();
    while (!SerialUSB)
    {
        if ((millis() - tTryUSB) > 5000)
            break;
    }
    SerialDebug.println("Start...");
    SerialDebug.println("Teste terminal AT!\r\n\r\n");

    if (!modem.begin(AU915))
    {
        SerialDebug.println("!!! ERRO no modem.begin() !!! \r\n\r\n");
        while (1)
            ;
    }
    delay(500);
    if (!modem.publicNetwork(false))
        SerialDebug.println("!!! ERRO ao definir modo PRIVADO!\r\n\r\n");

    //-------- DEBUG ----------
    SerialDebug.print("Your module version is: ");
    SerialDebug.println(modem.version());
    SerialDebug.print("Your device EUI is: ");
    SerialDebug.println(modem.deviceEUI());
    SerialDebug.println("App EUI is: " + String(appEui));
    SerialDebug.println("App key is: " + String(appKey) + "\r\n\r\n");
    //-------- DEBUG ----------
}

void loop()
{
    String buf;
    if (SerialDebug.available())
    {
        buf = SerialDebug.readStringUntil('\n');
        buf.trim();
        SerialDebug.println();
        if (buf == "api_joinotaa")
        {
            int connected = modem.joinOTAA(appEui, appKey);
            SerialDebug.println("Join OTAA retornou: " + String(connected));
        }
        else if (buf == "api_joinabp")
        {
            int connected = modem.joinABP(deviceAddr, NetworkSKey,
                                          AppSessionKey);
            SerialDebug.println("Join ABP retornou: " + String(connected));
        }
        else if (buf == "reset")
        {
            SerialDebug.println(" --- MODEM Begin + Modo privado ---\r\n");
            if (!modem.begin(AU915))
                SerialDebug.println("!!! ERRO no modem.begin() !!!\r\n\r\n");
            delay(500);
            if (!modem.publicNetwork(false))
                SerialDebug.println("!!! ERRO ao definir modo PRIVADO!\r\n");
        }
        else if (buf == "check")
        {
            auto ret = modem.isJoined();
            SerialDebug.println("GetJoinStatus retornou: " + String(ret));
        }
        else if (buf == "reboot")
        {
            auto ret = modem.restart();
            SerialDebug.println("modem.restart retornou: " + String(ret));
        }
        else if (buf == "meujoin")
        {
            bool ret = modem.fitecJoinOTAA(150000L, 4, appEui, appKey);
            if (ret)
                SerialDebug.println("Join com sucesso");
            else
                SerialDebug.println("Erro no FITec Join");
        }
        else if (buf == "init")
        {
            auto ret = modem.initTransmit();
            if (ret)
                SerialDebug.println("Transmissao inicializada");
            else
                SerialDebug.println("Erro ao inicializar transmissoes");
        }
        else
        {
            SerialDebug.println(buf);
            modem.fitStreamPrint(buf);
            modem.fitStreamPrint('\r');
            modem.fitStreamFlush();
        }
    }
    while (modem.fitStreamAvailable())
    {
        SerialDebug.write(modem.fitStreamRead());
    }
}
#endif

#ifdef TEST_TIMER5

#include "wiring_private.h"
#include "tc_timer5.h"

//Habilitacao de uma nova UART, Serial3, nos pinos 0, PA22 (Tx) e 1, PA23 (Rx)
Uart Serial3(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM3_Handler()
{
    Serial3.IrqHandler();

}

uint16_t cont_test;
bool paridade = false;

void setup()
{
    pinPeripheral(1, PIO_SERCOM);
    pinPeripheral(0, PIO_SERCOM);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    SerialDebug.begin(9600);
    SerialDebug.println("Start...\r\n\r\n");
    SerialDebug.flush();

    //Considerando que está com DIV1024
    //   escrevendo diretamente este valor no CCx, 46875 = 1Hz
    //   usando o calculo (SystemCoreClock / sampleRate - 1), 1024 = 1Hz

    tcConfigure(333333, TIMER_MICROS);
    tcStartCounter();

    cont_test = 0;
}

void loop()
{
    SerialDebug.println("atual: " + String(++cont_test));
    SerialDebug.flush();
    delay(100);
    // digitalWrite(LED_BUILTIN, paridade);
    // paridade = !paridade;
    if (cont_test == 200)
        tcStopCounter();
    if (cont_test == 300)
        tcStartCounter();
}

#endif

#ifdef TESTE_CRIPTO

#include "wiring_private.h"
#include "cripto.h"

/**
* @brief Funcao para inicilializar o HW
*
* Esta funcao inicializa os pinos para suas funcoes especificas
* e configura o hardware da placa MKRWAN para o devido uso, inclusive
* as portas seriais.
*
************************************************************************/
void initHW(void)
{
    pinPeripheral(1, PIO_SERCOM);
    pinPeripheral(0, PIO_SERCOM);
    pinMode(LED_BUILTIN, OUTPUT);

    digitalWrite(LED_BUILTIN, LOW);

    pinMode(PIN_A5, INPUT);
    randomSeed(analogRead(PIN_A5));

    Serial1.begin(9600);
    Serial1.setTimeout(250);
    SerialDebug.begin(9600);

    SerialDebug.println("Start...");

    return;
}

//Habilitacao de uma nova UART, Serial3, nos pinos 0, PA22 (Tx) e 1, PA23 (Rx)
Uart Serial3(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM3_Handler()
{
    Serial3.IrqHandler();
}

void stringToBytes (uint8_t *pStream, std::string strIn)
{
    for (uint8_t i = 0; i < strIn.length(); i++)
    {
        uint8_t msb = tolower(strIn.c_str()[2*i]);
        uint8_t lsb = tolower(strIn.c_str()[(2*i)+1]);
        msb -= isdigit(msb) ? 0x30 : 0x57;
        lsb -= isdigit(lsb) ? 0x30 : 0x57;

        *pStream = (lsb | (msb << 4));
        pStream++;
    }
}

void setup()
{
    initHW();

     SerialUSB.begin(9600);
     delay(2000);
     //while (!SerialUSB) ;

    //#define SerialDebug SerialUSB

    SerialUSB.println("Start ... \r\n Teste na serial da USB!!!");

    //auto testeMD5 = md5("12345678101112131415161718190102030405060708090A");
    char input[25] = {0x12, 0x34, 0x56, 0x78,
                      0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
                      0x16, 0x17, 0x18, 0x19,
                      0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                      0x07, 0x08, 0x09, 0x0A, 0x00};
    auto testeMD5 = md5({0x12, 0x34, 0x56, 0x78,
                         0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
                         0x16, 0x17, 0x18, 0x19,
                         0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                         0x07, 0x08, 0x09, 0x0A});
    //auto testeSHA = sha256("0102030405060708090A10111213141516171819");
    auto testeSHA = sha256({0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A,
                            0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19});
    SerialDebug.print("Result MD5 = ");
    SerialDebug.println(testeMD5.c_str());
    SerialDebug.print("Result SHA256 = ");
    SerialDebug.println(testeSHA.c_str());

    SerialDebug.println("Input: ");
    for (uint8_t i = 0; i < 24; i++)
        SerialDebug.print(String(input[i], HEX) + " ");
    SerialDebug.println();
    SerialDebug.flush();

    auto testExemplo = md5(input);
    SerialDebug.println("Novo teste 1: ");
    SerialDebug.println(testExemplo.c_str());

    auto testString = std::string(input);
    SerialDebug.print("\r\nMD5 de string = ");
    auto strMD5 = md5(testString);
    SerialDebug.println(strMD5.c_str());

    uint8_t cpMD5[16];
    stringToBytes(cpMD5, strMD5);
    for (uint8_t k = 0; k < 16; k++)
        SerialDebug.print(String(cpMD5[k], HEX) + " ");
    SerialDebug.println();
    SerialDebug.flush();

    return;

    /**************************************************************************
     * Resultado esperado:
     * MD5: C98ECC81191F020836503FE8C9D632C4
     * SHA256: B1C26F2CBE1F019D333C4742B1C75E8F77F51411D372B9E92914F7D94E307F68
     *
     *************************************************************************/
}
void loop()
{
}

#endif

#ifdef TESTE_MODELO

#include "wiring_private.h"

// Modelo dos medidores L+G
// ICGs:
// E-750:   8721 e 8722
// E-650:   8621 e 8622
// Residenciais:
// E-450:   8451 e 8452
// E-430:   8431 e 8432
#define LG_ID_ICG                   0x8620
#define LG_ID_RESIDENCIAL           0x8410
#define FINAL1OU2(x)                ((x & 0x0001) ^ ((x & 0x0002) >> 1))
#define PENULTIMO3OU5(x)            (((x & 0x0040) >> 6) ^ ((x & 0x0020) >> 5))
#define EH_MEDIDOR_ICG(x)           (((x & 0xFEFC) == LG_ID_ICG) && \
                                     FINAL1OU2(x))
#define EH_MEDIDOR_RESIDENCIAL(x)   (((x & 0xFF90) == LG_ID_RESIDENCIAL) && \
                                     PENULTIMO3OU5(x) && \
                                     FINAL1OU2(x))

/**
* @brief Funcao para inicilializar o HW
*
* Esta funcao inicializa os pinos para suas funcoes especificas
* e configura o hardware da placa MKRWAN para o devido uso, inclusive
* as portas seriais.
*
************************************************************************/
void initHW(void)
{
    pinPeripheral(1, PIO_SERCOM);
    pinPeripheral(0, PIO_SERCOM);
    pinMode(LED_BUILTIN, OUTPUT);

    digitalWrite(LED_BUILTIN, LOW);

    pinMode(PIN_A5, INPUT);
    randomSeed(analogRead(PIN_A5));

    Serial1.begin(9600);
    Serial1.setTimeout(250);
    SerialDebug.begin(9600);

    SerialDebug.println("Start...");

    return;
}

//Habilitacao de uma nova UART, Serial3, nos pinos 0, PA22 (Tx) e 1, PA23 (Rx)
Uart Serial3(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM3_Handler()
{
    Serial3.IrqHandler();
}

void setup()
{
    initHW();
    uint16_t modelos[16] = {0x8721, 0x8722, 0x8621, 0x8622,
                            0x8431, 0x8432, 0x8451, 0x8452,
                            0x8700, 0x8631, 0xF721, 0x5622,
                            0x8450, 0xF431, 0x8732, 0x0451};

    SerialDebug.println("\r\nModelos:");
    for (auto i : modelos)
    {
        SerialDebug.println(String(i, HEX) + " -"
                             " ICG: " + String(EH_MEDIDOR_ICG(i)) +
                             " Res: " + String(EH_MEDIDOR_RESIDENCIAL(i)));
    }
    SerialDebug.println();
    SerialDebug.flush();
}

void loop()
{
}

#endif

#ifdef TEST_INITXMIT
#include "wiring_private.h"

/**
* @brief Funcao para inicilializar o HW
*
* Esta funcao inicializa os pinos para suas funcoes especificas
* e configura o hardware da placa MKRWAN para o devido uso, inclusive
* as portas seriais.
*
************************************************************************/
void initHW(void)
{
    pinPeripheral(1, PIO_SERCOM);
    pinPeripheral(0, PIO_SERCOM);
    pinMode(LED_BUILTIN, OUTPUT);

    digitalWrite(LED_BUILTIN, LOW);

    pinMode(PIN_A5, INPUT);
    randomSeed(analogRead(PIN_A5));

    Serial1.begin(9600);
    Serial1.setTimeout(250);
    SerialDebug.begin(9600);

    SerialDebug.println("Start...");

    return;
}

//Habilitacao de uma nova UART, Serial3, nos pinos 0, PA22 (Tx) e 1, PA23 (Rx)
Uart Serial3(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM3_Handler()
{
    Serial3.IrqHandler();
}
#define SerialDebug SerialUSB

#define SECRET_APP_EUI "70B3D57ED0012777"
//#define SECRET_APP_KEY "f4092d86e075ad28fc6c04e0973e8d4d" // - dev. 0115 | EUI: ...32467316
#define SECRET_APP_KEY "2ca6c21ec44c45984ee111534f5f355c" // - dev. 0105 | EUI: ...393C7A0B
//#define SECRET_APP_KEY "27ca801eadbd2b30b52c64581b4b37a4" // - dev. debugger | EUI: ...39476906
//#define SECRET_APP_KEY "c11b6c1d71a26eca4dbb1cf7e0722882" // - dev. MKR avulso EUI: ...3923750B

#define ABP_DEVADDR     "0030b79c"
#define ABP_NWKSKEY     "349eca63ffc23f220ee9429d019c0c08"
#define ABP_APPSKEY     "bbdf2e1a7331cb1ed19039d9b54b8119"

FITecModem modem;

char appEui[17] = SECRET_APP_EUI;
char appKey[33] = SECRET_APP_KEY;
static const String deviceAddr = ABP_DEVADDR;
static const String NetworkSKey = ABP_NWKSKEY;
static const String AppSessionKey = ABP_APPSKEY;

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    pinPeripheral(1, PIO_SERCOM);
    pinPeripheral(0, PIO_SERCOM);

    SerialDebug.begin(9600);
    //USB
    uint32_t tTryUSB = millis();
    while (!SerialUSB)
    {
        if ((millis() - tTryUSB) > 5000)
            break;
    }
    SerialDebug.println("Start...");
    SerialDebug.println("Teste FITecJoin\r\n\r\n");

    if (!modem.begin(AU915))
    {
        SerialDebug.println("!!! ERRO no modem.begin() !!! \r\n\r\n");
        while (1) ;
    }
    delay(500);
    if (!modem.publicNetwork(false))
        SerialDebug.println("!!! ERRO ao definir modo PRIVADO!\r\n\r\n");

    //-------- DEBUG ----------
    SerialDebug.print("Your module version is: ");
    SerialDebug.println(modem.version());
    SerialDebug.print("Your device EUI is: ");
    SerialDebug.println(modem.deviceEUI());
    SerialDebug.println("App EUI is: " + String(appEui));
    SerialDebug.println("App key is: " + String(appKey) + "\r\n\r\n");
    //-------- DEBUG ----------

    uint16_t vez = 0;
    while (1)
    {
        SerialDebug.println("---------- VEZ " + String(++vez) + "----------");
        uint32_t tLog;
        bool joinLoRa = false;
        uint8_t n = 0;

        tLog = millis();
        SerialDebug.println("Start Join: " + String(tLog));
        while (!joinLoRa)
        {
            joinLoRa = modem.fitecJoinOTAA(200000L, 3, appEui,
                                           appKey);
            SerialDebug.println(String(++n));
        }
        SerialDebug.println("Duracao Join: " + String(millis() - tLog));

        bool init = false;
        n = 0;
        tLog = millis();
        SerialDebug.println("Start Xmit: " + String(tLog));
        while (!init)
        {
            init = modem.initTransmit();
            SerialDebug.println(String(++n));
            delay(500);
        }
        SerialDebug.println("Duracao Xmit: " + String(millis() - tLog));
        SerialDebug.println("\r\n-=-=-=-=-=-=-= FIM -=-=-=-=-=-=-=\r\n\r\n");
        modem.restart();
        delay(500);
        modem.begin(AU915);
        delay(1000);
        modem.publicNetwork(false);
        delay(500);
    }
}

void loop()
{
}

#endif
