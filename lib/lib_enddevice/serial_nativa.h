/** @file serial_nativa.h
* @brief Cabecalho para modulo serial nativa
*
* Este arquivo contem prototipos das funcoes do modulo
* serial nativa e as referencias para as variaveis exportadas.
*
* @date Feb/2019
* @author FITec - Fundacao para Inovacoes Tecnologicas
*         Marcio J Teixeira Jr. <mjunior@fitec.org.br>
*
************************************************************************/
#if !defined(__SERIAL_NATIVA_H__)
#define __SERIAL_NATIVA_H__

#define SERIAL_RTS  4
#define SERIAL_CTS  5

/******* Funções exportadas *****/
bool temMedidorSerial(void);
uint8_t recv_dataSerial(uint16_t *nBytes, uint8_t *buffPayload);
uint8_t send_dataSerial(uint32_t datasize, volatile uint8_t *databuf);
void initSerialNativa(void);

#endif
