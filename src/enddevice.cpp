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
#ifdef FW_ENDDEVICE

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
#include <Adafruit_SleepyDog.h>
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
void trataCausaRst(uint8_t rcause_lido);
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

sanidade_local_t logSanidadeLocal;

uint32_t tLastTemp = 0;
//uint32_t tLasTab1TEST = 0; ///#### mmjr Teste enviando tab1

char localAppEui[17] = SECRET_APP_EUI;
char localAppKey[33] = SECRET_APP_KEY;

void setup()
{

    auto fonte_rst = PM->RCAUSE.reg;
    //(PM->RCAUSE.reg & (PM_RCAUSE_SYST | PM_RCAUSE_WDT | PM_RCAUSE_EXT))) {

    // 'Tempo morto' na inicialização...
    while (millis() < TEMPO_MORTO) ;
    initHW();



    piscaLed(3, 50, 150);

    sanidade_t teste_sanidade1;

    SerialUSB.begin(9600);
    uint32_t tStart = millis();

    while ((millis() - tStart) < T_WAITUSB)
    {
        if (SerialUSB)
            break;
    }

    //#########
    // Avaliar habilitacao de interrupcao para monitorar PWR_FAIL, gerando um
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
    trataCausaRst(fonte_rst);
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

    //uint8_t senhaE430[11] = {"TAETEEEUGT"};
    //uint8_t senhaCliente[11] = {"UJEUTETUGH"};

    // memcpy(localKeys.senhaABNT, senhaE430, 10);
    // savedKeys.write(localKeys);
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

    int valorWDT = Watchdog.enable(16000);
    SerialDebug.println("WDT On! T=" + String(valorWDT) + " ms\r\n\r\n");

    //#### Debug do log de sanidade
    //
    //logSanidadeLocal.currUptime = (((((uint64_t)logSanidadeLocal.uptimeRollMilli) << 32)
    //               + millis()) / 1000);

    SerialDebug.println("\r\nLog de sanidade:");
    SerialDebug.println("POR: " + String(localKeys.log_sanidade.contPOR));
    SerialDebug.println("SW: " + String(localKeys.log_sanidade.contSWrst));
    SerialDebug.println("WDT: " + String(localKeys.log_sanidade.contWDT));
    SerialDebug.println("Ext: " + String(localKeys.log_sanidade.contEXTrst));
    SerialDebug.println("Uplinks: " + String(localKeys.log_sanidade.contUp));
    SerialDebug.println("Dwlinks: " + String(localKeys.log_sanidade.contDw));
    SerialDebug.println("Uptime sec: " + String((uint32_t)(localKeys.log_sanidade.curUptime)));
    SerialDebug.println("MaxUptime sec: " + String((uint32_t)(localKeys.log_sanidade.maxUptime)));
    SerialDebug.println("\r\n---- LOCAL: ----");
    SerialDebug.println("Uptime roll: " + String(logSanidadeLocal.uptimeRollMilli));
    SerialDebug.println("ptLeitABNT Urgente: " + String((uint8_t)logSanidadeLocal.ptLeituraABNTUrgente));
    SerialDebug.println("ptEscrABNT Urgente: " + String((uint8_t)logSanidadeLocal.ptEscritaABNTUrgente));
    SerialDebug.println("ptLeitABNT: " + String((uint8_t)logSanidadeLocal.ptLeituraABNT));
    SerialDebug.println("ptEscrABNT: " + String((uint8_t)logSanidadeLocal.ptEscritaABNT));
    SerialDebug.println("ptLeitLoRa: " + String((uint8_t)logSanidadeLocal.ptLeituraLoRa));
    SerialDebug.println("ptEscrLoRa: " + String((uint8_t)logSanidadeLocal.ptEscritaLoRa));

    SerialDebug.println("\r\n------- Fim do Log de sanidade -------");
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
    {
        if (!trataLoRa())
        {
            LoRaOK = false;
            SerialDebug.println("\r\n  --->trataLoRa() retornou FALSE! <---");
            SerialDebug.println("Vai iniciar novo JoinOTAA...\r\n");
        }
    }
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
    // so e chamada quando a maquina de estados ABNT esta em IDLE
    if ((stateABNT < ABNT_STATE_ENVIA_CMD) &&
        ((millis() - tLastTemp) > T_MIN_MEDE_TEMP))
    {
        medeTemperatura();
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

    trataSinalizacaoSucesso();
    //SerialDebug.println("St: " + String(sinalizaStatus) +
    //                 " Serial = " + String(portaSerial.interface));
    delay(50);
    Watchdog.reset();
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
                ansi_tab5.alarmes.senha_abnt = 0;
            }
        }
        break;

        case ID_CMD13:
        {
            abnt_resp_generic_t *respCmd13;
            respCmd13 = buffRespRecebida;
            memcpy (sementeABNT, respCmd13->payload, sizeof(sementeABNT));
            ansi_tab5.alarmes.medidor_bloq = 0;
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

            verificaUptime();
            ansi_tab3.logSanidade = localKeys.log_sanidade;

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
            itemCmdABNT_t cmdEnviado;

            cmdEnviado = buscaUltimoCmd();

            //Caso seja ocorrencia 43, indicando necessidade de abertura
            //de sessao ou parametrizacao incompleta
            if (ABNT_OCORRENCIA43 == cmd40->payload[0])
            {
                if (localKeys.senhaABNT_ok != FLASH_VALID)
                {
                    //#### DEBUG !!!
                    SerialDebug.println("NAO vai abrir sessao para envio do cmd" +
                                        String(cmdEnviado.cmd, HEX) +
                                        "\r\n Senha ABNT nao programada!\r\n");
                    ansi_tab5.alarmes.senha_abnt = 1;
                    insereTabelaANSI(TAB_ANSI05, SIZE_TABELA5_CMD40);
                }
                else
                {
                    //Abre a sessao com senha e reenvia o mesmo comando
                    cmdAtrasado = cmdEnviado;
                    //#### DEBUG !!!
                    SerialDebug.println("Vai abrir sessao para envio do cmd" +
                                        String(cmdAtrasado.cmd, HEX));
                    insereCmdABNT(LISTA_URGENTE, ID_CMD13);
                    insereCmdABNT(LISTA_URGENTE, ID_CMD11);
                }
            }
            else if (ABNT_OCORRENCIA41 == cmd40->payload[0]) //senha incorreta
            {
                ansi_tab5.alarmes.senha_abnt = 1;
                insereTabelaANSI(TAB_ANSI05, SIZE_TABELA5_CMD40);
                localKeys.senhaABNT_ok = false;
                savedKeys.write(localKeys);
            }
            else if (ABNT_OCORRENCIA45 == cmd40->payload[0])//medidor bloqueado
            {
                ansi_tab5.alarmes.medidor_bloq = 1;
                insereTabelaANSI(TAB_ANSI05, SIZE_TABELA5_CMD40);
                //localKeys.senhaABNT_ok = false;
            }

            else if (cmdEnviado.cmd != ID_CMD37)
            //Envia CMD37 para limpar ocorrencia, para todos os comandos, exceto
            //o proprio 37...
            {
                ansi_tab5.cod_ocorrencia = cmd40->payload[0];
                ansi_tab5.subcod_ocorrencia = cmd40->payload[1];
                insereTabelaANSI(TAB_ANSI05, SIZE_TABELA5_CMD40);
                abntOcorrencia = ansi_tab5.cod_ocorrencia;
                //Envia cmd37 pra limpar cond de ocorrencia
                insereCmdABNT(LISTA_URGENTE, ID_CMD37);
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

        ///// --- DEBUG - Resposta recebida
        // SerialDebug.println("\r\n Resposta recebida do medidor(ABNT):");
        // for (uint16_t i = 0; i < 258; i++)
        // {
        //     SerialDebug.print(String(pBuffABNTrecv[i], HEX) + " ");
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

    SerialDebug.println("\r\n\r\n=-=-=-=-=-=-=-=-\r\nStart...");

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
        localKeys.senhaABNT_ok = false; // indica que nao ha senha ABNT valida
        savedKeys.write(localKeys);
        SerialDebug.println("Chave(s) recebida(s)!\r\n Valores default gravados!");
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
        Watchdog.reset();
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
            Watchdog.reset();
            init = modem.initTransmit();
            delay(500);
        }
    }

    if (init)
    {
        /*updateTLoRaSend();*/
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
    SerialDebug.println("Porta de debug ...\r\nNao preparada para "
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
    piscaLed(3, 300);
    piscaLed(3, 100, 100);
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
                        piscaLed(2, 600, 200);
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
    piscaLed(3, 250, 50);
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

/**
* @brief Funcao para tratar a causa do ultimo reset
*
* Esta funcao trata a causa do ultimo reset e deve ser chamada no inicio do
* setup(). A leitura o registro deve ser feita previamente e passada para
* esta funcao que irá identificar a causa e incrementar o contador especifico
* prerando para ser salvo.
*
* @param[in] registro lido a ser avaliado
*
************************************************************************/
void trataCausaRst(uint8_t rcause_lido)
{
    SerialDebug.print("\r\n\r\n\r\n---------------------\r\nRCAUSE = 0x");
    SerialDebug.println(rcause_lido, HEX);
    if (rcause_lido & PM_RCAUSE_SYST)
    {
        localKeys.log_sanidade.contSWrst++;
        SerialDebug.println("\r\n\r\n ---> RESET por Software !!! <---\r\n");
    }
    else if (rcause_lido & PM_RCAUSE_WDT)
    {
        localKeys.log_sanidade.contWDT++;
        SerialDebug.println("\r\n\r\n ---> RESET por WDT !!! <---\r\n");
    }
    else if (rcause_lido & PM_RCAUSE_EXT)
    {
        localKeys.log_sanidade.contEXTrst++;
        SerialDebug.println("\r\n\r\n ---> RESET Externo !!! <---\r\n");
    }
    else if (rcause_lido & PM_RCAUSE_POR)
    {
        localKeys.log_sanidade.contPOR++;
        SerialDebug.println("\r\n\r\n ---> RESET por POR !!! <---\r\n");
    }
    else
        SerialDebug.println("\r\n\r\n ---> RESET desconhecido !!! <---\r\n");

    salvarNaoVolatil = true;
} //trataCausaRst(

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


#endif //FW_ENDDEVICE

///////////////////////////////////////////////////////////////
//-----------------------------------------------------------//


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


#ifdef TESTE_REPIQUE

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

    pinMode(PIN_A5, OUTPUT);

    Serial1.begin(9600);
    Serial1.setTimeout(250);
    SerialDebug.begin(9600);

    SerialDebug.println("Start...");

    return;
}

void piscaLed(uint8_t ciclos, uint16_t t_on, uint16_t t_off)
{

    for (uint16_t k = 0; k < ciclos; k++)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        digitalWrite(PIN_A5, HIGH);
        delay(t_on);
        digitalWrite(LED_BUILTIN, LOW);
        digitalWrite(PIN_A5, LOW);
        delay(t_off);
    }
} //piscaLed(

//Habilitacao de uma nova UART, Serial3, nos pinos 0, PA22 (Tx) e 1, PA23 (Rx)
Uart Serial3(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM3_Handler()
{
    Serial3.IrqHandler();
}
#define SerialDebug SerialUSB

void setup()
{
    void initHW();
    piscaLed(5, 50, 100);
}

void loop()
{
    piscaLed();
}
#endif
