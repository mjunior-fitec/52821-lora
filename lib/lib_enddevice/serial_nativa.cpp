/** @file serial_nativa.cpp
* @brief Modulo com a implementacao das funcoes de transmissao
* e recepcao de buffer de bytes na serial RS-232 nativa do processador.
*
* Este modulo prove as funcoes de transmissao e recepcao de um stream de
* bytes na porta serial nativa do processador. Estas funcoes deverao ser
* apontadas pelo elemento de comunicacao quando for identificada a
* comunicacao com um medidor nesta interface.
*
* @date Feb/2019
* @author FITec - Fundacao para Inovacoes Tecnologicas
*         Marcio J Teixeira Jr. <mjunior@fitec.org.br>
*
************************************************************************/

#include "util.h"
#include "abnt.h"

#include <stdint.h>
#include <Arduino.h>

/**
* @brief Funcao para identificar se ha um medidor ABNT conectado a
* porta serial nativa do end device.
*
* Esta funcao identifica se ha um medidor ABNT conectado a porta serial
* nativa. Para isso, verifica se esta recebendo ENQ pela porta serial.
*
* @return retrun TRUE para conexao existente, FALSE caso contrario
*
************************************************************************/
bool temMedidorSerial(void)
{
    uint8_t bytes;
    bytes = Serial1.available();
    if (bytes)
    {
        if (ABNT_ENQ == Serial1.peek())
        {
            return true;
        }
        else
        {
            uint8_t dummybuff[64];
            Serial1.readBytes(dummybuff, bytes);
        }
    }
    return false;
} //temMedidorSerial(

/**
* @brief Funcao para receber um buffer de bytes pela serial nativa
*
* Esta funcao recebe um buffer de bytes pela porta serial nativa do
* processador. Sua assinatura é identica a do modulo otica_ftdi para
* que sejam intercambeaveis e associadas a um ponteiro para funcao
* usando-se a mesma chamada.
*
* @param[in] *nBytes Numero de bytes a ser recebido
* @param[out] *buffPayload Buffer para receber os bytes
* @return Código de erro, 0 para sucesso
*
************************************************************************/
uint8_t recv_dataSerial(uint16_t *nBytes, uint8_t *buffPayload)
{
    uint16_t bytesRec = 0;

    *nBytes = Serial1.available(); //Forca a receber apenas o que tem disponivel
    if (*nBytes)
    {
        bytesRec = Serial1.readBytes(buffPayload, (size_t)(*nBytes));
        if (bytesRec)
        {
            *nBytes = bytesRec;
        }
        else
            return 0xFF;
    }
    return 0;
} //recv_dataSerial(

/**
* @brief Funcao para enviar um buffer de bytes pela serial nativa
*
* Esta funcao recebe um buffer de bytes e o envia pela porta serial nativa
* do processador. Sua assinatura é identica a do modulo otica_ftdi para
* que sejam intercambeaveis e associadas a um ponteiro para funcao
* usando-se a mesma chamada.
*
* @param[in] datasize Tamanho do buffer a ser enviado
* @param[in] databuf Buffer com os dados a serem enviados
* @return Código de erro, 0 para sucesso
*
************************************************************************/
uint8_t send_dataSerial(uint32_t datasize, volatile uint8_t *databuf)
{
    if (Serial1.write((uint8_t *)databuf, (size_t)datasize))
        return 0;
    return 0xFF;
} //send_dataSerial(

/**
* @brief Inicializa a intterface fisica da serial nativa para comunicacao
* com o medidor ABNT via RS-232
*
* Funcao para inicializar a interface fisica da serial nativa para comunicacao
* com o medidor ABNT via RS-232. O timeout esta fixo em 250milissegundos que
* foi o valor determinado na pratica com bom resultado para o medidor Landis Gyr.
*
************************************************************************/
void initSerialNativa(void)
{
    Serial1.begin(ABNT_BAUD);
    Serial1.setTimeout(250);
}
