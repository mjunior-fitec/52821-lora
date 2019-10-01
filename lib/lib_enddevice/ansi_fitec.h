/** @file ansi-fitec.h
* @brief Cabeçalho para o módulo que implementa o protocolo ANSI-FITec
*
* Este arquivbo contém os tipos de dados (tabelas), variáveis
* exportadas e protótipos de funções referentes ao módulo ANSI-FITec
*
* @date Jan/2019
* @author FITec - Fundacao para Inovacoes Tecnologicas
*         Marcio J Teixeira Jr. <mjunior@fitec.org.br>
*
************************************************************************/


#if !defined(__ANSI_FITEC_H__)
#define __ANSI_FITEC_H__

#include <stdint.h>
#include <time.h>
#include "util.h"
#include "abnt.h"

#define NUM_TABELAS_UP  9
#define TMIN_TRY_LORA   130000

//Funcoes exportadas
void ansiFitecInit(void);
int8_t loRaSetup(void);
bool trataLoRa(void);
uint8_t calcSum(uint8_t *buffer, size_t bufSize);
bool insereTabelaANSI(uint8_t IdTabela, uint8_t tam = 0);
void updateTLoRaSend(void);

enum nomePostos {
    PONTA,
    FORA_PONTA,
    RESERVADO,
    INTERMEDIARIO,
    CONT_POSTOS
};

typedef struct regAlteracao
{
    uint8_t     codAlteracao;
    uint32_t    numSerieLeitor: 24;
    uint32_t    dataHoraAlteracao;
} __attribute__ ((packed)) regAlteracao_t;

typedef union postoHor {
    uint16_t HoraMinInicio;
    struct {
        uint8_t HoraInicio;
        uint8_t MinInicio;
    } camposHoraMinuto;
} postoHorario_t;

typedef struct alarmes
{
    uint32_t falha_comunicacao: 1;
    uint32_t senha_abnt: 1;
    uint32_t medidor_bloq: 1;
    uint32_t relogio_medidor: 1;
    uint32_t relogio_dispositivo: 1;
    uint32_t temperatura_dev: 1;
    uint32_t temperatura_med: 1;
    uint32_t aberura_principal: 1;
    uint32_t abertura_bloco: 1;
    uint32_t movimento: 1;
    uint32_t i_sem_v: 1;
    uint32_t potencia_reversa: 1;
    uint32_t vcarga_corte: 1;
    uint32_t dif_i_neutro: 1;
    uint32_t sobretensao: 1;
    uint32_t subtensao: 1;
    uint32_t vmax: 1;
    uint32_t vmin: 1;
    uint32_t sobrecorrente: 1;
    uint32_t imin: 1;
    uint32_t ineutro: 1;
    uint32_t fator_pot: 1;
    uint32_t i_reversa: 1;
    uint32_t bateria_baixa: 1;
    uint32_t :0; //garante um novo uint32
    uint32_t reservado;
} __attribute__ ((packed)) alarmes_t;

typedef struct tabela01SemDemanda
{
    uint32_t    timestamp;
    uint8_t     posto_horario;
    uint32_t    temperatura: 24;
    uint32_t    enAtivaTotDireto;
    uint32_t    enAtivaPontaDireto;
    uint32_t    enAtivaForaDireto;
    uint32_t    enAtivaIntermDireto;
    uint32_t    enAtivaReservDireto;
    uint32_t    enReativaTotDireto;
} __attribute__ ((packed)) tabela01SemDemanda_t;
#define SIZE_TABELA1_SEMDEMANDA 32

typedef enum
{
     TAB_ANSI01 = 1,
     TAB_ANSI02,
     TAB_ANSI03,
     TAB_ANSI04,
     TAB_ANSI05,
     TAB_ANSI06,
     TAB_ANSI07,
     TAB_ANSI08,
     TAB_ANSI09,
     TAB_ANSI10,
     TAB_ANSI11,
     TAB_ANSI12,
     TAB_ANSI13,
     TAB_ANSI14,
} tiposTabANSI_t;

typedef struct tabela01
{
    uint32_t    timestamp;
    uint8_t     posto_horario;
    uint32_t    temperatura: 24;
    uint32_t    enAtivaTotDireto;
    uint32_t    enAtivaPontaDireto;
    uint32_t    enAtivaForaDireto;
    uint32_t    enAtivaIntermDireto;
    uint32_t    enAtivaReservDireto;
    uint32_t    enReativaTotDireto;
    uint16_t    demAtivaPonta;
    uint16_t    demAtivaFora;
    uint16_t    demAtivaInterm;
    uint16_t    demAtivaReserv;
} __attribute__ ((packed)) tabela01_t;
#define SIZE_TABELA1_COMPLETA   40

typedef struct tabela02
{
    uint32_t    enAtivaTotReverso;
    uint32_t    enAtivaPontaReverso;
    uint32_t    enAtivaForaReverso;
    uint32_t    enAtivaIntermReverso;
    uint32_t    enAtivaReservReverso;
} __attribute__ ((packed)) tabela02_t;
#define SIZE_TABELA2    20

typedef struct tabela03
{
    uint32_t    enReativaCapTot;
    uint32_t    tensao_a: 24;
    uint32_t    tensao_b: 24;
    uint32_t    tensao_c: 24;
    uint32_t    corrente_a: 24;
    uint32_t    corrente_b: 24;
    uint32_t    corrente_c: 24;
    uint16_t    drp;
    uint16_t    drc;
} __attribute__ ((packed)) tabela03_t;
#define SIZE_TABELA3    26

typedef struct tabela04
{
    uint32_t dataHoraIniFalta[5];
    uint32_t dataHoraFimFalta[5];
} __attribute__ ((packed)) tabela04_t;
#define SIZE_TABELA4    40

typedef struct tabela05
{
    alarmes_t alarmes;
    uint8_t     cod_ocorrencia;
    uint8_t     subcod_ocorrencia;
    uint8_t     solicitaRTC;
    uint32_t    enReativaIndPonta;
    uint32_t    enReativaIndFora;
    uint32_t    enReativaIndReserv;
    uint32_t    enReativaCapPonta;
    uint32_t    enReativaCapFora;
    uint32_t    enReativaCapReserv;
} __attribute__ ((packed)) tabela05_t;
#define SIZE_TABELA5                35
#define SIZE_TABELA5_SO_ALARMES     8
#define SIZE_TABELA5_CMD40          10
#define SIZE_TABELA5_SOLICITA_RTC   11

typedef struct tabela06
{
    uint32_t numSerieMedidor;
    uint8_t  codInstalacao[14];
    uint16_t demReativaIndPonta;
    uint16_t demReativaIndFora;
    uint16_t demReativaIndReserv;
    uint16_t demReativaCapPonta;
    uint16_t demReativaCapFora;
    uint16_t demReativaCapReserv;
} __attribute__ ((packed)) tabela06_t;
#define SIZE_TABELA6    34

typedef struct tabela07
{
    inicioPostosHorarios_t postoHorario[4];
    configHorVer_t horVerao;
} __attribute__ ((packed)) tabela07_t;
#define SIZE_TABELA7    37

typedef struct tabela08
{
    feriado_t feriados[15];
} __attribute__ ((packed)) tabela08_t;
#define SIZE_TABELA8    45

typedef struct tabela09
{
    regAlteracao_t regAlteracoes[5];
} __attribute__ ((packed)) tabela09_t;
#define SIZE_TABELA9    40

typedef struct tabela10
{
    uint32_t dataHoraCFG;
    configHorVer_t horVeraoCFG;
    inicioPostosHorarios_t postoHorarioCFG[4];
    uint8_t numSegHorarios;
} __attribute__ ((packed)) tabela10_t;
#define SIZE_TABELA10   42

typedef struct tabela10_so_relogio
{
    uint32_t dataHoraCFG;
} __attribute__ ((packed)) tabela10_so_relogio_t;
#define SIZE_TABELA10_SO_RELOGIO   4

typedef struct tabela11
{
    feriado_t feriadosCFG[15];
} __attribute__ ((packed)) tabela11_t;
#define SIZE_TABELA11   45

typedef struct tabela12
{
    uint8_t corteReliga;
    uint8_t intervalo;
    uint8_t solicitaTabela[9];
    uint8_t senhaABNT[10];
} __attribute__ ((packed)) tabela12_t;
#define SIZE_TABELA12   21

typedef struct tabelasAnsi
{
    uint8_t id;
    uint8_t size;
    uint8_t *tabela;
} tabelasAnsi_t;

typedef struct itemListaTabAnsi
{
    uint8_t idTabela;
    uint8_t tamEspecial;
} __attribute__ ((packed)) itemListaTabAnsi_t;

#endif

// variaveis exportada
extern volatile uint8_t *   loRaBuffPt;
extern volatile uint8_t     loRaBuffSize;
extern volatile bool        bloqPtANSI;
extern volatile uint32_t    t0_LoRa, t0_ABNT, novoIntervaloLoRa;
extern FITecModem modem; //LoRaModem modem;
extern volatile bool        salvarNaoVolatil;
/* Variaveis globais com as tabelas a serem enviadas via LoRa */
extern tabela01_t ansi_tab1;
extern tabela02_t ansi_tab2;
extern tabela03_t ansi_tab3;
extern tabela04_t ansi_tab4;
extern tabela05_t ansi_tab5;
extern tabela06_t ansi_tab6;
extern tabela07_t ansi_tab7;
extern tabela08_t ansi_tab8;
extern tabela09_t ansi_tab9;
