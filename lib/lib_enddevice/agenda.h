/** @file agenda.h
* @brief Arquivo de cabecalho do modulo agenda
*
* Este e o arquivo de cabecalho do modulo agenda, contendo as variaveis
* exportadas e prototipos das funcoes deste modulo.
*
* @date Jun/2019
* @author FITec - Fundacao para Inovacoes Tecnologicas
*         Marcio J Teixeira Jr. <mjunior@fitec.org.br>
*
************************************************************************/
#if !defined(__AGENDA_H__)
#define __AGENDA_H__

#include <RTCZero.h>
#include "util.h"

/* Variaveis exportadas */
extern itemAgenda_t *listaAgenda, *itemAtual;
extern RTCZero rtc;
extern uint8_t intervaloLoRa;
extern int32_t localMaxTemp;
extern bool relogioValido;

/* Prototipos das funcoes exportadas */
void initAgenda(void);
void iniciaAlarmes(void);
void limpaLista(itemAgenda_t * head);
void programaAlarmeAgenda(void);
itemAgenda_t * criaItemAgenda(uint32_t new_tAgenda, void (* new_fAgenda) (void));
void insereAgenda(itemAgenda_t **head, itemAgenda_t *itemAInserir);
void montaAgenda(void);
void imprimeAgenda(void);
void transmiteLoRaFrequente(void);
void transmiteLoRaEventual(void);
void preparaABNTFrequente(void);
void preparaABNTEventual(void);
void atualizaMaxUptime(void);
void verificaUptime(void);

#endif
