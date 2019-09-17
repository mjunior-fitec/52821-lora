
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

void trataRespABNT(void)
{
    abnt_resp_generic_t *buffRespRecebida = (abnt_resp_generic_t *)pBuffABNTrecv;

    switch (buffRespRecebida->id)
    {
    case ID_CMD14:
        abnt_resp_le_grand_instant_t *respLeGrandInst;
        respLeGrandInst = (abnt_resp_le_grand_instant_t *)buffRespRecebida;
        SerialDebug.println("Rec cmd 0x14");

        SerialDebug.println("Num. de serie medidor: " + String(abntBcdToByte(TOGGLEENDIAN32(respLeGrandInst->num_serie_registrador))));
        SerialDebug.println("Data: " + String(abntBcdToByte(respLeGrandInst->dia)) + "/" \
                                     + String(abntBcdToByte(respLeGrandInst->mes)) + "/" \
                                     + String(abntBcdToByte(respLeGrandInst->ano)));
        SerialDebug.println("Hora: " + String(abntBcdToByte(respLeGrandInst->hora)) + ":" \
                                     + String(abntBcdToByte(respLeGrandInst->min)) + ":" \
                                     + String(abntBcdToByte(respLeGrandInst->seg)));
        SerialDebug.println("Va: "   + String(respLeGrandInst->tensao_a) + \
                          "\tVb: "   + String(respLeGrandInst->tensao_b) + \
                          "\tVc: "   + String(respLeGrandInst->tensao_c) + \
                         "\tVab: "   + String(respLeGrandInst->tensao_ab) + \
                         "\tVbc: "   + String(respLeGrandInst->tensao_bc) + \
                         "\tVca: "   + String(respLeGrandInst->tensao_ca));

        SerialDebug.println("Ia: "   + String(respLeGrandInst->corrente_a) + \
                          "\tIb: "   + String(respLeGrandInst->corrente_b) + \
                          "\tIc: "   + String(respLeGrandInst->corrente_c) + \
                          "\tIn: "   + String(respLeGrandInst->corrente_n));

        SerialDebug.println("Pa: "   + String(respLeGrandInst->pot_ativa_a) + \
                          "\tPb: "   + String(respLeGrandInst->pot_ativa_b) + \
                          "\tPc: "   + String(respLeGrandInst->pot_ativa_c) + \
                          "\tP3: "   + String(respLeGrandInst->pot_ativa_tri));

        SerialDebug.println("PRa: "  + String(respLeGrandInst->pot_reativa_a) + \
                          "\tPRb: "  + String(respLeGrandInst->pot_reativa_b) + \
                          "\tPRc: "  + String(respLeGrandInst->pot_reativa_c) + \
                          "\tPR3: "  + String(respLeGrandInst->pot_reativa_tri));

        SerialDebug.println("CosFi_a: " + String(respLeGrandInst->cos_fi_a) + \
                          "\tCosFi_b: " + String(respLeGrandInst->cos_fi_b) + \
                          "\tCosFi_c: " + String(respLeGrandInst->cos_fi_c) + \
                          "\tCosFi_3: " + String(respLeGrandInst->cos_fi_tri));

        SerialDebug.println("Temp: " + String(respLeGrandInst->temperatura) + \
                          "\tFreq: " + String(respLeGrandInst->frequencia));
        break;

    case ID_CMD21:
        abnt_resp_leitura_param_t *respLeParam;

        respLeParam = (abnt_resp_leitura_param_t *)buffRespRecebida;

        SerialDebug.println("Rec cmd 0x21");
        SerialDebug.println("Num. de serie medidor: " + String(abntBcdToByte(TOGGLEENDIAN32(respLeParam->num_serie_registrador))));
        SerialDebug.println("Data: " + String(abntBcdToByte(respLeParam->dia)) + "/" \
                                     + String(abntBcdToByte(respLeParam->mes)) + "/" \
                                     + String(abntBcdToByte(respLeParam->ano)) + "\t\t" \
                                     + "Dia da semana: " + String(respLeParam->dia_semana));
        SerialDebug.println("Hora: " + String(abntBcdToByte(respLeParam->hora)) + ":" \
                                     + String(abntBcdToByte(respLeParam->min)) + ":" \
                                     + String(abntBcdToByte(respLeParam->seg)));

        SerialDebug.println("Ultimo intervalo de demanda: " + String(abntBcdToByte(respLeParam->intDemandas[3])) + "/" \
                                                            + String(abntBcdToByte(respLeParam->intDemandas[4])) + "/" \
                                                            + String(abntBcdToByte(respLeParam->intDemandas[5])) + "\t" \
                                                            + String(abntBcdToByte(respLeParam->intDemandas[0])) + ":" \
                                                            + String(abntBcdToByte(respLeParam->intDemandas[1])) + ":" \
                                                            + String(abntBcdToByte(respLeParam->intDemandas[2])));

        SerialDebug.println("Ultima reposicao de demanda: " + String(abntBcdToByte(respLeParam->intDemandas[9])) + "/" \
                                                            + String(abntBcdToByte(respLeParam->intDemandas[10])) + "/" \
                                                            + String(abntBcdToByte(respLeParam->intDemandas[11])) + "\t" \
                                                            + String(abntBcdToByte(respLeParam->intDemandas[6])) + ":" \
                                                            + String(abntBcdToByte(respLeParam->intDemandas[7])) + ":" \
                                                            + String(abntBcdToByte(respLeParam->intDemandas[8])));

        SerialDebug.println("Penultima reposicao de demanda: " + String(abntBcdToByte(respLeParam->intDemandas[15])) + "/" \
                                                               + String(abntBcdToByte(respLeParam->intDemandas[16])) + "/" \
                                                               + String(abntBcdToByte(respLeParam->intDemandas[17])) + "\t" \
                                                               + String(abntBcdToByte(respLeParam->intDemandas[12])) + ":" \
                                                               + String(abntBcdToByte(respLeParam->intDemandas[13])) + ":" \
                                                               + String(abntBcdToByte(respLeParam->intDemandas[14])));

        SerialDebug.println("Segmentos horarios: ");
        SerialDebug.println("Inicio Ponta " + String(abntBcdToByte(respLeParam->inicioPostosHorarios[PONTA].horaInicio1)) + ":" \
                                            + String(abntBcdToByte(respLeParam->inicioPostosHorarios[PONTA].minutoInicio1)) + "\t" \
                                            + String(abntBcdToByte(respLeParam->inicioPostosHorarios[PONTA].horaInicio2)) + ":" \
                                            + String(abntBcdToByte(respLeParam->inicioPostosHorarios[PONTA].minutoInicio2)) + "\t" \
                                            + String(abntBcdToByte(respLeParam->inicioPostosHorarios[PONTA].horaInicio3)) + ":" \
                                            + String(abntBcdToByte(respLeParam->inicioPostosHorarios[PONTA].minutoInicio3)) + "\t" \
                                            + String(abntBcdToByte(respLeParam->inicioPostosHorarios[PONTA].horaInicio4)) + ":" \
                                            + String(abntBcdToByte(respLeParam->inicioPostosHorarios[PONTA].minutoInicio4)));
        SerialDebug.println("Inicio Fora Ponta " + String(abntBcdToByte(respLeParam->inicioPostosHorarios[FORA_PONTA].horaInicio1)) + ":" \
                                                 + String(abntBcdToByte(respLeParam->inicioPostosHorarios[FORA_PONTA].minutoInicio1)) + "\t" \
                                                 + String(abntBcdToByte(respLeParam->inicioPostosHorarios[FORA_PONTA].horaInicio2)) + ":" \
                                                 + String(abntBcdToByte(respLeParam->inicioPostosHorarios[FORA_PONTA].minutoInicio2)) + "\t" \
                                                 + String(abntBcdToByte(respLeParam->inicioPostosHorarios[FORA_PONTA].horaInicio3)) + ":" \
                                                 + String(abntBcdToByte(respLeParam->inicioPostosHorarios[FORA_PONTA].minutoInicio3)) + "\t" \
                                                 + String(abntBcdToByte(respLeParam->inicioPostosHorarios[FORA_PONTA].horaInicio4)) + ":" \
                                                 + String(abntBcdToByte(respLeParam->inicioPostosHorarios[FORA_PONTA].minutoInicio4)));
        SerialDebug.println("Inicio Reservado "  + String(abntBcdToByte(respLeParam->inicioPostosHorarios[RESERVADO].horaInicio1)) + ":" \
                                                 + String(abntBcdToByte(respLeParam->inicioPostosHorarios[RESERVADO].minutoInicio1)) + "\t" \
                                                 + String(abntBcdToByte(respLeParam->inicioPostosHorarios[RESERVADO].horaInicio2)) + ":" \
                                                 + String(abntBcdToByte(respLeParam->inicioPostosHorarios[RESERVADO].minutoInicio2)) + "\t" \
                                                 + String(abntBcdToByte(respLeParam->inicioPostosHorarios[RESERVADO].horaInicio3)) + ":" \
                                                 + String(abntBcdToByte(respLeParam->inicioPostosHorarios[RESERVADO].minutoInicio3)) + "\t" \
                                                 + String(abntBcdToByte(respLeParam->inicioPostosHorarios[RESERVADO].horaInicio4)) + ":" \
                                                 + String(abntBcdToByte(respLeParam->inicioPostosHorarios[RESERVADO].minutoInicio4)));
        SerialDebug.println("Feriados nacionais: ");
        for (uint8_t i_fer = 0; i_fer < 15; ++i_fer)
        {
            SerialDebug.println(String(abntBcdToByte(respLeParam->feriados[i_fer].dia)) + "/" \
                              + String(abntBcdToByte(respLeParam->feriados[i_fer].mes)) + "/" \
                              + String(abntBcdToByte(respLeParam->feriados[i_fer].ano)));
        }

        SerialDebug.println("Estado da bateria " + String(respLeParam->estadoBateria));
        SerialDebug.println("Versao de SW " + String(abntBcdToByte(TOGGLEENDIAN16(respLeParam->versaoSWMedidor))));
        SerialDebug.println("Modelo do medidor " + String(abntBcdToByte(TOGGLEENDIAN16(respLeParam->modeloMedidor))));
        SerialDebug.print("Horario de verao: ");
        if (respLeParam->horVerao.ativar)
            SerialDebug.println("ATIVADO");
        else
            SerialDebug.println("DESATIVADO");
        SerialDebug.println("Fim do horario de inverno: " + String(abntBcdToByte(respLeParam->horVerao.diaFimHorInverno)) + "/" \
                                                          + String(abntBcdToByte(respLeParam->horVerao.mesFimHorInverno)));
        SerialDebug.println("Fim do horario de verao: "   + String(abntBcdToByte(respLeParam->horVerao.diaFimHorVerao)) + "/" \
                                                          + String(abntBcdToByte(respLeParam->horVerao.mesFimHorVerao)));
        break;

    case ID_CMD23:
        abnt_resp_leitura_regs_atuais_t *respRegsAtuais;
        uint64_t valorTotalizador;

        respRegsAtuais = (abnt_resp_leitura_regs_atuais_t *)buffRespRecebida;
        SerialDebug.println("Rec cmd 0x23");
        SerialDebug.println("Num. de serie medidor: " + String((uint32_t)abntBcdToInt((uint8_t *)&(respRegsAtuais->num_serie_registrador), 4)));

        SerialDebug.print("Totalizador geral 1o canal: 0x");
        valorTotalizador = abntBcdToInt(respRegsAtuais->totalGeralCanal1, 5);
        SerialDebug.println(String((uint32_t)(valorTotalizador >> 32), HEX) + String((uint32_t)(valorTotalizador & 0xFFFFFFFF), HEX));
        SerialDebug.print("Totalizador do 1o canal em ponta:");
        valorTotalizador = abntBcdToInt(respRegsAtuais->totalCanal1Ponta, 5);
        SerialDebug.println(String((uint32_t)(valorTotalizador >> 32), HEX) + String((uint32_t)(valorTotalizador & 0xFFFFFFFF), HEX));
        SerialDebug.print("Totalizador do 1o canal em fora ponta:");
        valorTotalizador = abntBcdToInt(respRegsAtuais->totalCanal1Fora, 5);
        SerialDebug.println(String((uint32_t)(valorTotalizador >> 32), HEX) + String((uint32_t)(valorTotalizador & 0xFFFFFFFF), HEX));
        SerialDebug.print("Totalizador do 1o canal em reservado:");
        valorTotalizador = abntBcdToInt(respRegsAtuais->totalCanal1Reserv, 5);
        SerialDebug.println(String((uint32_t)(valorTotalizador >> 32), HEX) + String((uint32_t)(valorTotalizador & 0xFFFFFFFF), HEX));
        SerialDebug.print("Totalizador do 1o canal em intermediario:");
        valorTotalizador = abntBcdToInt(respRegsAtuais->totalCanal1Interm, 5);
        SerialDebug.println(String((uint32_t)(valorTotalizador >> 32), HEX) + String((uint32_t)(valorTotalizador & 0xFFFFFFFF), HEX));
        SerialDebug.print("Totalizador geral 2o canal: 0x");
        valorTotalizador = abntBcdToInt(respRegsAtuais->totalGeralCanal2, 5);
        SerialDebug.println(String((uint32_t)(valorTotalizador >> 32), HEX) + String((uint32_t)(valorTotalizador & 0xFFFFFFFF), HEX));
        SerialDebug.print("Totalizador do 2o canal em ponta:");
        valorTotalizador = abntBcdToInt(respRegsAtuais->totalCanal2Ponta, 5);
        SerialDebug.println(String((uint32_t)(valorTotalizador >> 32), HEX) + String((uint32_t)(valorTotalizador & 0xFFFFFFFF), HEX));
        SerialDebug.print("Totalizador do 2o canal em fora ponta:");
        valorTotalizador = abntBcdToInt(respRegsAtuais->totalCanal2Fora, 5);
        SerialDebug.println(String((uint32_t)(valorTotalizador >> 32), HEX) + String((uint32_t)(valorTotalizador & 0xFFFFFFFF), HEX));
        SerialDebug.print("Totalizador do 2o canal em reservado:");
        valorTotalizador = abntBcdToInt(respRegsAtuais->totalCanal2Reserv, 5);
        SerialDebug.println(String((uint32_t)(valorTotalizador >> 32), HEX) + String((uint32_t)(valorTotalizador & 0xFFFFFFFF), HEX));
        SerialDebug.print("Totalizador geral 3o canal: 0x");
        valorTotalizador = abntBcdToInt(respRegsAtuais->totalGeralCanal3, 5);
        SerialDebug.println(String((uint32_t)(valorTotalizador >> 32), HEX) + String((uint32_t)(valorTotalizador & 0xFFFFFFFF), HEX));
        SerialDebug.print("Totalizador do 3o canal em ponta:");
        valorTotalizador = abntBcdToInt(respRegsAtuais->totalCanal3Ponta, 5);
        SerialDebug.println(String((uint32_t)(valorTotalizador >> 32), HEX) + String((uint32_t)(valorTotalizador & 0xFFFFFFFF), HEX));
        SerialDebug.print("Totalizador do 3o canal em fora ponta:");
        valorTotalizador = abntBcdToInt(respRegsAtuais->totalCanal3Fora, 5);
        SerialDebug.println(String((uint32_t)(valorTotalizador >> 32), HEX) + String((uint32_t)(valorTotalizador & 0xFFFFFFFF), HEX));
        SerialDebug.print("Totalizador do 3o canal em reservado:");
        valorTotalizador = abntBcdToInt(respRegsAtuais->totalCanal3Reserv, 5);
        SerialDebug.println(String((uint32_t)(valorTotalizador >> 32), HEX) + String((uint32_t)(valorTotalizador & 0xFFFFFFFF), HEX));
        SerialDebug.print("Demanda maxima do 1o canal em ponta: 0x");
        valorTotalizador = abntBcdToInt(respRegsAtuais->demMaxCanal1Ponta, 5);
        SerialDebug.println(String((uint32_t)(valorTotalizador >> 32), HEX) + String((uint32_t)(valorTotalizador & 0xFFFFFFFF), HEX));
        SerialDebug.print("Demanda maxima do 1o canal em fora ponta: 0x");
        valorTotalizador = abntBcdToInt(respRegsAtuais->demMaxCanal1Fora, 5);
        SerialDebug.println(String((uint32_t)(valorTotalizador >> 32), HEX) + String((uint32_t)(valorTotalizador & 0xFFFFFFFF), HEX));
        SerialDebug.print("Demanda maxima do 1o canal em reservado: 0x");
        valorTotalizador = abntBcdToInt(respRegsAtuais->demMaxCanal1Reserv, 5);
        SerialDebug.println(String((uint32_t)(valorTotalizador >> 32), HEX) + String((uint32_t)(valorTotalizador & 0xFFFFFFFF), HEX));
        SerialDebug.print("Demanda maxima do 1o canal em intermediario: 0x");
        valorTotalizador = abntBcdToInt(respRegsAtuais->demMaxCanal1Interm, 5);
        SerialDebug.println(String((uint32_t)(valorTotalizador >> 32), HEX) + String((uint32_t)(valorTotalizador & 0xFFFFFFFF), HEX));
        SerialDebug.print("Demanda maxima do 2o canal em ponta: 0x");
        valorTotalizador = abntBcdToInt(respRegsAtuais->demMaxCanal2Ponta, 5);
        SerialDebug.println(String((uint32_t)(valorTotalizador >> 32), HEX) + String((uint32_t)(valorTotalizador & 0xFFFFFFFF), HEX));
        SerialDebug.print("Demanda maxima do 2o canal em fora ponta: 0x");
        valorTotalizador = abntBcdToInt(respRegsAtuais->demMaxCanal2Fora, 5);
        SerialDebug.println(String((uint32_t)(valorTotalizador >> 32), HEX) + String((uint32_t)(valorTotalizador & 0xFFFFFFFF), HEX));
        SerialDebug.print("Demanda maxima do 2o canal em reservado: 0x");
        valorTotalizador = abntBcdToInt(respRegsAtuais->demMaxCanal2Reserv, 5);
        SerialDebug.println(String((uint32_t)(valorTotalizador >> 32), HEX) + String((uint32_t)(valorTotalizador & 0xFFFFFFFF), HEX));
        SerialDebug.print("Demanda maxima do 3o canal em ponta: 0x");
        valorTotalizador = abntBcdToInt(respRegsAtuais->demMaxCanal3Ponta, 5);
        SerialDebug.println(String((uint32_t)(valorTotalizador >> 32), HEX) + String((uint32_t)(valorTotalizador & 0xFFFFFFFF), HEX));
        SerialDebug.print("Demanda maxima do 3o canal em fora ponta: 0x");
        valorTotalizador = abntBcdToInt(respRegsAtuais->demMaxCanal3Fora, 5);
        SerialDebug.println(String((uint32_t)(valorTotalizador >> 32), HEX) + String((uint32_t)(valorTotalizador & 0xFFFFFFFF), HEX));
        SerialDebug.print("Demanda maxima do 3o canal em reservado: 0x");
        valorTotalizador = abntBcdToInt(respRegsAtuais->demMaxCanal3Reserv, 5);
        SerialDebug.println(String((uint32_t)(valorTotalizador >> 32), HEX) + String((uint32_t)(valorTotalizador & 0xFFFFFFFF), HEX));
        break;

    case ID_CMD25:
        abnt_resp_leitura_falta_energia_t *respFaltaEnergia;

        respFaltaEnergia = (abnt_resp_leitura_falta_energia_t *)buffRespRecebida;
        for (uint8_t i=0; i<20; ++i)
        {
            SerialDebug.println("Falta de energia " + String(1+i) + ":");
            SerialDebug.println("Data / hora da falta: " + String(abntBcdToByte(respFaltaEnergia->faltas[i].diaFalta)) + "/"  \
                                                         + String(abntBcdToByte(respFaltaEnergia->faltas[i].mesFalta)) + "/"  \
                                                         + String(abntBcdToByte(respFaltaEnergia->faltas[i].anoFalta)) + "\t" \
                                                         + String(abntBcdToByte(respFaltaEnergia->faltas[i].horaFalta)) + ":" \
                                                         + String(abntBcdToByte(respFaltaEnergia->faltas[i].minutoFalta)) + ":" \
                                                         + String(abntBcdToByte(respFaltaEnergia->faltas[i].segundoFalta)));
            SerialDebug.println("Data / hora do retorno: " + String(abntBcdToByte(respFaltaEnergia->faltas[i].diaRet)) + "/"  \
                                                           + String(abntBcdToByte(respFaltaEnergia->faltas[i].mesRet)) + "/"  \
                                                           + String(abntBcdToByte(respFaltaEnergia->faltas[i].anoRet)) + "\t" \
                                                           + String(abntBcdToByte(respFaltaEnergia->faltas[i].horaRet)) + ":" \
                                                           + String(abntBcdToByte(respFaltaEnergia->faltas[i].minutoRet)) + ":" \
                                                           + String(abntBcdToByte(respFaltaEnergia->faltas[i].segundoRet)));
        }
        SerialDebug.println("Ha QTD e DTD: " + String(respFaltaEnergia->temQTD_DTD));
        SerialDebug.print("Dados da reposicao: ");
        if (respFaltaEnergia->demandaAtual)
            SerialDebug.println ("ATUAL");
        else
            SerialDebug.println("ANTERIOR");
        SerialDebug.println("Total de segundos em falta de energia: " + String(respFaltaEnergia->totalSegundosFalta));
        SerialDebug.println("Numero de faltas de energia: " + String(respFaltaEnergia->numFaltas));
        SerialDebug.println("Total de segundos, da leitura, em falta de energia: " + String(respFaltaEnergia->segundosFaltaLeiura));
        SerialDebug.println("Numero de faltas de energia, da leitura: " + String(respFaltaEnergia->numFaltasLeiutra));
        break;

    default:
        break;
    }
    TEST_PASS();
}

void proxComando(void)
{
    uint32_t t_comandoAtual = millis();

    while ((millis() - t_comandoAtual) < MAX_T_UM_COMANDO)
    {
        maqEstAbnt();
        if (respABNTRecebida)
        {
            trataRespABNT();
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
        SerialDebug.println("test: " + String(i_test) + "\t\tcmd: " + \
                            String(listaCmdParaTestes[i_test], HEX) );
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
