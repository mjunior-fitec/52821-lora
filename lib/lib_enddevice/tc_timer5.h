/** @file tc_timer5.h
* @brief Arquivo de cabecalho do modulo timer5
*
* Este e o arquivo de cabecalho do modulo timer5, contendo as variaveis
* exportadas e prototipos das funcoes deste modulo.
*
* @date Jun/2019
* @author FITec - Fundacao para Inovacoes Tecnologicas
*         Marcio J Teixeira Jr. <mjunior@fitec.org.br>
*
************************************************************************/
#if !defined(__TC_TIMER5_H__)
#define __TC_TIMER5_H__

typedef enum
{
    TIMER_HERTZ,
    TIMER_MICROS,
    TIMER_MILLIS
} tipoTimer_t;

//####################
//Prototipo das funcoes exportadas
void tcConfigure(uint32_t sampleRate, tipoTimer_t tipo = TIMER_HERTZ);
void tcStartCounter(void);
void tcReset(void);
void tcStopCounter(void);

#endif
