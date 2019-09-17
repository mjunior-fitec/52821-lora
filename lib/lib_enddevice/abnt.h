/** @file abnt.h
 *
 * @brief Arquivo de cabecalho do protocolo ABNT
 *
 * @author FITec - Fundacao para Inovacoes Tecnologicas
 *         Marcio J Teixeira Jr. <mjunior@fitec.org.br>
 ************************************************************************/

#if !defined(__ABNT_H__)
#define __ABNT_H__

#include <RTCZero.h>
#include "util.h"

#define ABNT_BAUD       9600
#define LEN_CMD         66
#define LEN_RES         258
#define PAYLOAD_CMD     64
#define PAYLOAD_RES     256
#define TAM_LISTA_CMD   32

#define NUM_LEITOR      0x011100

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

#define LISTA_PT(pt)        ( (pt)      % TAM_LISTA_CMD)
#define LISTA_NEXT_PT(pt)   (((pt)+1)   % TAM_LISTA_CMD)
#define LISTA_INC_PT(pt)    (((pt)++)   % TAM_LISTA_CMD)

/**
 * SINALIZADORES do protocolo ABNT
 */
#define ABNT_ALO    0xFF
#define ABNT_ENQ    0x05
#define ABNT_ACK    0x06
#define ABNT_NACK   0x15
#define ABNT_WAIT   0x10

#define ABNT_OCORRENCIA43 (0x43)
#define TAM_COD_INSTALACAO  14

#define T_MAX_ENQ           600 //Tempo maximo entre ENQs consecutivos,
                                //em milisegundos(600). A norma define ~502ms
#define ABNT_MAX_RETRIES    5
#define T_MAX_AGUARDA_RESP  2500

#define ABNT_CRIPTO(A,B,C) (signed char)((char)A^(char)B^(char)C)
#define PRIM1 43
#define PRIM2 19

#define CMD_11   //{

#define CMD_12   {0x12, 0x00, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x77, 0x7C}
#define CMD_13   {0x13, 0x00, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0xB6, 0x8C}
#define CMD_14   {0x14, 0x00, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0xF7, 0xDE}
#define CMD_21   {0x21, 0x00, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x3B, 0x39}
#define CMD_23   {0x23, 0x00, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0xBA, 0x98}
#define CMD_25   {0x25, 0x00, 0x11, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x3B, 0x06}
#define CMD_28   {0x28, 0x00, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0xF8, 0xCF}
#define CMD_29   {0x29, 0x00, 0x11, 0x01, 0x02, 0x01, 0x13, 0x04, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                  0x00, 0x00, 0x00, 0x00, 0xA2, 0x7A} // 02/01/19 - Qua


#define ABNT_LE_GRAND_INST  CMD_14

#define LG_LEITURA  0x00
#define LG_CORTE    0xDE
#define LG_RELIGA   0xC0

typedef enum
{
    ID_CMD11 = 0x11,
    ID_CMD12,
    ID_CMD13,
    ID_CMD14,
    ID_CMD15,
    ID_CMD16,
    ID_CMD17,
    ID_CMD18,
    ID_CMD19,
    ID_CMD20 = 0x20,
    ID_CMD21,
    ID_CMD22,
    ID_CMD23,
    ID_CMD24,
    ID_CMD25,
    ID_CMD26,
    ID_CMD27,
    ID_CMD28,
    ID_CMD29,
    ID_CMD30 = 0x30,
    ID_CMD31,
    ID_CMD32,
    ID_CMD33,
    ID_CMD34,
    ID_CMD35,
    ID_CMD36,
    ID_CMD37,
    ID_CMD38,
    ID_CMD39,
    ID_CMD40 = 0x40,
    ID_CMD64 = 0x64,
    ID_CMD80 = 0x80,
    ID_CMD87 = 0x87,
    ID_CMDE2 = 0xE2,
} idCmd_t;

typedef enum
{
    CMD_LANDIS_LEITURA_PARAM =  0x2B,
    CMD_LANDIS_DRP_DRC =        0x41,
    CMD_LANDIS_ANTIFRAUDE =     0x50,
    CMD_LANDIS_CORTE_RELIGA =   0x90,
} cmdLandis_t;

/* Estruturas Genericas */

//Comando generico
typedef struct abnt_cmd_generic
{
    uint8_t id;
    uint32_t num_serie_leitor : 24;
    /////#### COLOCAR UM UNION COM TODOS OS COMANDOS NA MESMA STRUCT <<<<<<<---------
    uint8_t payload[60];
    uint16_t crc;
} __attribute__((packed)) abnt_cmd_generic_t;

//resposta generica
typedef struct abnt_resp_generic
{
    uint8_t id;
    uint32_t num_serie_registrador;
    /////#### COLOCAR UM UNION COM TODAS AS RESPOSTAS NA MESMA STRUCT <<<<<<<---------
    uint8_t payload[251];
    uint16_t crc;
} __attribute__((packed)) abnt_resp_generic_t;

typedef struct inicioPostosHorarios
{
    uint8_t horaInicio1;
    uint8_t minutoInicio1;
    uint8_t horaInicio2;
    uint8_t minutoInicio2;
    uint8_t horaInicio3;
    uint8_t minutoInicio3;
    uint8_t horaInicio4;
    uint8_t minutoInicio4;
} __attribute__((packed)) inicioPostosHorarios_t;

typedef struct configHorVer
{
    uint8_t ativar;
    uint8_t diaFimHorInverno;
    uint8_t mesFimHorInverno;
    uint8_t diaFimHorVerao;
    uint8_t mesFimHorVerao;
} __attribute__((packed)) configHorVer_t;

typedef struct monitor_fraude
/**
 * @attention quantidade esta codificada com LSB primeiro,
 * ao contrario dos dados ABNT
 */
{
    uint8_t id;
    uint8_t estado;
    uint16_t quantidade; // LSB first !!!
} __attribute__ ((packed)) monitor_fraude_t;

typedef struct leitura_drp_drc
{
    uint32_t    tInicio;
    uint32_t    tFim;
    uint16_t    drp0;
    uint16_t    drp1;
    uint16_t    drp2;
    uint16_t    drc0;
    uint16_t    drc1;
    uint16_t    drc2;
} __attribute__ ((packed)) leitura_drp_drc_t;


/**
 * Comandos especificos
 */
typedef struct abnt_cmd_altera_feriados
{
    uint8_t id;
    uint32_t num_serie_leitor : 24;
    feriado_t feriados[15];
    uint8_t nullpad[15];
    uint16_t crc;
} __attribute__((packed)) abnt_cmd_altera_feriados_t;

typedef struct abnt_cmd_altera_segmentos_horarios
{
    uint8_t id;
    uint32_t num_serie_leitor : 24;
    inicioPostosHorarios_t progPostoHor[4];
    uint8_t numSegmentos;
    uint8_t nullpad[27];
    uint16_t crc;
} __attribute__((packed)) abnt_cmd_altera_segmentos_horarios_t;

typedef struct abnt_cmd_altera_hor_verao
{
    uint8_t id;
    uint32_t num_serie_leitor : 24;
    configHorVer_t horarioVerao;
    uint8_t nullpad[55];
    uint16_t crc;
} __attribute__ ((packed)) abnt_cmd_altera_hor_verao_t;

typedef struct abnt_cmd_abertura_sessao
{
    uint8_t id;
    uint32_t num_serie_leitor : 24;
    uint8_t criptografia[10];
    uint8_t cripto_leitura[10];
    uint8_t num_usuario;
    uint8_t nullpad[39];
    uint16_t crc;
} __attribute__ ((packed)) abnt_cmd_abertura_sessao_t;

typedef struct abnt_cmd_corte_religa
{
    uint8_t id;
    uint32_t num_serie_leitor : 24;
    uint8_t idExt;
    uint8_t codAcao;
    uint8_t nullpad[58];
    uint16_t crc;
} __attribute__ ((packed)) abnt_cmd_corte_religa_t;

/**
 * Respostas aos comandos
 */

typedef struct abnt_resp_le_grand_instant
{
    uint8_t id;
    uint32_t num_serie_registrador;
    uint8_t hora;
    uint8_t min;
    uint8_t seg;
    uint8_t dia;
    uint8_t mes;
    uint8_t ano;
    float tensao_a;
    float tensao_b;
    float tensao_c;
    float tensao_ab;
    float tensao_bc;
    float tensao_ca;
    float corrente_a;
    float corrente_b;
    float corrente_c;
    float corrente_n;
    float pot_ativa_a;
    float pot_ativa_b;
    float pot_ativa_c;
    float pot_ativa_tri;
    float pot_reativa_a;
    float pot_reativa_b;
    float pot_reativa_c;
    float pot_reativa_tri;
    uint8_t notused_1[32]; // Pot aparente
    uint8_t notused_2[16];   // Pot distorsiva
    float cos_fi_a;
    float cos_fi_b;
    float cos_fi_c;
    float cos_fi_tri;
    uint8_t notused_3[4];  // caracteristica reativa
    uint8_t notused_4[16]; // Fator de potencia
    uint8_t notused_5[12]; // Defasagem
    float temperatura;
    float frequencia;
    uint8_t tipoLigacao;
    uint8_t notused_6[8];  // Latitude, longitude
    uint8_t versaoMedidor;
    uint8_t notused_7[24]; // Angulos
    uint8_t notused_8[24]; // Distorcao harmonica
    uint8_t notused_9;
    uint8_t null_pad[10];
    uint16_t crc;
} __attribute__((packed)) abnt_resp_le_grand_instant_t;

typedef struct abnt_resp_leitura_param
{
    uint8_t id;
    uint32_t num_serie_registrador;
    uint8_t hora;
    uint8_t min;
    uint8_t seg;
    uint8_t dia;
    uint8_t mes;
    uint8_t ano;
    uint8_t dia_semana;
    //
    uint8_t intDemandas[18];
    uint8_t nullpad1[20];
    //
    inicioPostosHorarios_t inicioPostosHorarios[3]; //Ponta, Fora, Reservado
    uint8_t naoUsado1[9];
    feriado_t feriados[15];
    uint8_t naoUsado2[18]; // <---- Constantes de multiplicacao
    uint8_t estadoBateria;
    uint16_t versaoSWMedidor;
    uint8_t naoUsado3[3];
    uint16_t modeloMedidor;
    uint8_t naoUsado4[3];
    configHorVer_t horVerao;
    uint8_t naoUsado5[94];
    uint16_t crc; 		//total
} __attribute__((packed)) abnt_resp_leitura_param_t;

typedef struct abnt_resp_leitura_regs_atuais
{
    uint8_t id;
    uint32_t num_serie_registrador;
    uint8_t totalGeralCanal1[5];
    uint8_t totalCanal1Ponta[5];
    uint8_t totalUFERPonta[5];
    uint8_t totalCanal1Fora[5];
    uint8_t totalUFERFora[5];
    uint8_t totalCanal1Reserv[5];
    uint8_t totalUFERReserv[5];
    uint8_t demUltimoIntCanal1[3];
    uint8_t demMaxCanal1Ponta[3];
    uint8_t DMCRPonta[3];
    uint8_t demMaxCanal1Fora[3];
    uint8_t DMCRFora[3];
    uint8_t demMaxCanal1Reserv[3];
    uint8_t DMCRReserv[3];
    uint8_t demAcumulada1[18]; //Nao usado
    uint8_t totalGeralCanal2[5];
    uint8_t totalCanal2Ponta[5];
    uint8_t totalCanal2RevPonta[5];
    uint8_t totalCanal2Fora[5];
    uint8_t totalCanal2RevFora[5];
    uint8_t totalCanal2Reserv[5];
    uint8_t totalCanal2RevReserv[5];
    uint8_t demUltimoIntCanal2[3];
    uint8_t demMaxCanal2Ponta[3];
    uint8_t demMaxCanal2RevPonta[3];
    uint8_t demMaxCanal2Fora[3];
    uint8_t demMaxCanal2RevFora[3];
    uint8_t demMaxCanal2Reserv[3];
    uint8_t demMaxCanal2RevReserv[3];
    uint8_t demAcumulada2[18]; //Nao usado
    uint8_t totalGeralCanal3[5];
    uint8_t totalCanal3Ponta[5];
    uint8_t totalCanal3RevPonta[5];
    uint8_t totalCanal3Fora[5];
    uint8_t totalCanal3RevFora[5];
    uint8_t totalCanal3Reserv[5];
    uint8_t totalCanal3RevReserv[5];
    uint8_t demUltimoIntCanal3[3];
    uint8_t demMaxCanal3Ponta[3];
    uint8_t demMaxCanal3RevPonta[3];
    uint8_t demMaxCanal3Fora[3];
    uint8_t demMaxCanal3RevFora[3];
    uint8_t demMaxCanal3Reserv[3];
    uint8_t demMaxCanal3RevReserv[3];
    uint8_t demAcumulada3[18]; //Nao usado
    uint8_t totalCanal1Interm[5];
    uint8_t totalUFERInterm[5];
    uint8_t demMaxCanal1Interm[3];
    uint8_t DMCRInterm[3];
    uint8_t demAcumulada4[6]; //Nao usado
    uint8_t nullpad1[7];
    uint16_t crc;
} __attribute__ ((packed)) abnt_resp_leitura_regs_atuais_t;

typedef struct abnt_resp_leitura_falta_energia
{
    uint8_t id;
    uint32_t num_serie_registrador;
    falta_t faltas[20]; //240 bytes
    uint32_t temQTD_DTD : 1;            ///// <<--- Conferir a froam correta de leitura destes campos, se necessario
    uint32_t demandaAtual : 1;          ///// <<---
    uint32_t totalSegundosFalta : 30;   ///// <<---
    uint16_t numFaltas;                 ///// <<---
    uint32_t segundosFaltaLeiura : 24;  ///// <<---
    uint16_t numFaltasLeiutra;          ///// <<---
    uint16_t crc;
} __attribute__ ((packed)) abnt_resp_leitura_falta_energia_t;

typedef struct abnt_resp_leitura_reg_alt
{
    uint8_t id;
    uint32_t num_serie_registrador;
    alteracao_t alteracoes[16];   //160
    alteracao_t alteracoesEst[9]; // 90
    uint8_t null;
    uint16_t crc;
} __attribute__ ((packed)) abnt_resp_leitura_reg_alt_t;

typedef struct abnt_resp_leitura_param_medicao
{
    uint8_t id;
    uint32_t num_serie_registrador;
    uint8_t modoApresentacao[7];
    uint8_t condicaoPostoUniv;
    uint8_t horaMinInicioPostosUniv[128];
    uint8_t constMultiplicacao[24];
    uint8_t modoOpAtiva;
    uint8_t modoOpReativa;
    uint8_t constMultiplicReat[6];
    uint8_t tMostrador;
    uint8_t null1;
    inicioPostosHorarios_t inicioPostoHorarioInter; //8bytes
    uint8_t condAtivPostosUnivDiaSemana[8];
    uint8_t postosUnivHorVer;
    uint8_t qualiQuantGrandFaturamento;
    uint8_t dispGrandInst;
    uint8_t tipoLigacao;
    uint8_t redefSaidaUsuario;
    uint8_t constKp[6];
    uint8_t codConsumidor[14];
    uint8_t null2[40];
    uint16_t crc;
} __attribute__ ((packed)) abnt_resp_leitura_param_medicao_t;

typedef struct abnt_resp_cod_instalacao
{
    uint8_t id;
    uint32_t num_serie_registrador;
    uint8_t  leituraEscrita;
    uint8_t  condInstalacao;
    uint8_t  codInstalacao[14];
    uint8_t null[235];
    uint16_t crc;
} __attribute__ ((packed)) abnt_resp_cod_instalacao_t;

typedef struct abnt_landis_resp_leitura_monitores
{
    uint8_t id;
    uint32_t num_serie_registrador;
    uint8_t cod_estendido;
    monitor_fraude_t monitores[18];
    uint8_t nullpad[178];
    uint16_t crc;
} __attribute__ ((packed)) abnt_landis_resp_leitura_monitores_t;

typedef struct abnt_landis_resp_drp_drc
{
    uint8_t id;
    uint32_t num_serie_registrador;
    uint8_t cod_estendido;
    leitura_drp_drc_t drp_drc[12];
    uint8_t nullpad[10];
    uint16_t crc;
} __attribute__ ((packed)) abnt_landis_resp_drp_drc_t;

typedef struct abnt_landis_resp_le_param
{
    uint8_t id;
    uint32_t num_serie_registrador;
    uint8_t cod_estendido;
    uint8_t postoAtual;
    uint8_t naousado[22];
    uint8_t nullpad[227];
    uint16_t crc;
} __attribute__ ((packed)) abnt_landis_resp_le_param_t;

typedef struct abnt_resp_pedido_string
{
    uint8_t id;
    uint32_t num_serie_registrador;
    uint8_t string_senha[10];
    uint8_t nullpad[241];
    uint16_t crc;
} __attribute__ ((packed)) abnt_resp_pedido_string_t;

typedef struct abnt_resp_abertura_sessao
{
    uint8_t id;
    uint32_t num_serie_registrador;
    uint8_t valid_senha;
    uint8_t valid_senha_leitura;
    uint8_t num_usuario;
    uint8_t nullpad[248];
    uint16_t crc;
} __attribute__ ((packed)) abnt_resp_abertura_sessao_t;

/**
 * Estados da maquina de estados ABNT
 * @enum abnt_state Conjunto de todos os estados da maquina de
 * estado de controle do protocolo ABNT
 */
typedef enum
{
    ABNT_STATE_DISCONNECTED,
    ABNT_STATE_IDLE,
    ABNT_STATE_ENVIA_CMD,
    ABNT_STATE_AGUARDA_RESP,
    ABNT_STATE_TRATA_RESP,
    ABNT_STATE_TRATA_ERR
} abnt_state_t;

typedef enum
{
    SERIAL_NULL,
    SERIAL_USB,
    SERIAL_RS232
} interfaceSerial_t;

typedef enum
{
    CANAL_123,
    CANAL_456,
    CANAL_789,
} visibilidade_canais_t;

typedef enum
{
    LISTA_NORMAL,
    LISTA_URGENTE,
} tipoLista_t;

typedef enum
{
    ANSI_PARAM_LEITURA,
    ANSI_PARAM_CORTE,
    ANSI_PARAM_RELIGA,
} coteReligaLG_t;

typedef struct itemCmdABNT
{
    uint8_t cmd;
    uint8_t codEstendido;
} __attribute__ ((packed)) itemCmdABNT_t;

typedef struct serialABNT
{
    interfaceSerial_t interface;
    uint8_t (*recMsgMedidor) (uint16_t *nBytes, uint8_t *buffPayload);
    uint8_t (*sendCmdMedidor) (uint32_t datasize, volatile uint8_t *databuf);
} __attribute__ ((packed)) serialABNT_t;

/*--------------------------------------------------*/
/*   Protótipos das funcoes exportadas              */
/*--------------------------------------------------*/
int16_t recvMsgMedidorABNT(abnt_resp_generic_t *resp);
int16_t recvMsgMedidorABNT(abnt_resp_generic_t *resp, uint16_t timeout);
void abntInit(void);
void maqEstAbnt(void);
void sinaliza_cmd_abnt(uint8_t IdCmd);
bool insereCmdABNT(tipoLista_t lista, uint8_t IdCmd, uint8_t estendido = 0);
void abntRespostaTratada();
itemCmdABNT_t buscaUltimoCmd(void);

/**
 * variáveis exportadas
 * */
extern volatile serialABNT_t portaSerial;
extern volatile uint8_t     stateABNT;
extern volatile uint8_t *   pBuffABNTrecv;
extern volatile uint8_t *   pBuffABNTsend;
extern volatile bool        bloqPtABNT;
extern volatile bool        respABNTRecebida;;
extern abnt_cmd_altera_hor_verao_t abntAlteraHorVer;
extern abnt_cmd_altera_segmentos_horarios_t abntProgSegmentosHorarios;
extern abnt_cmd_altera_feriados_t abntProgFeriados;
extern volatile uint8_t cmdEstendido;
extern itemCmdABNT_t cmdAtrasado;
extern uint8_t programacaoCorteReliga;
extern uint8_t abntOcorrencia;
extern RTCZero rtc;
extern uint8_t canaisVisiveis;
extern uint8_t sementeABNT[10];
#endif
