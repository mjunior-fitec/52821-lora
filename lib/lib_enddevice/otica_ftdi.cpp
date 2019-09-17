#include <cdcftdi.h>
#include <usbhub.h>

#include "abnt.h"
#include "otica_ftdi.h"
//#include "pgmstrings.h"

/*-----------------------------------------*/
/*  Variáveis globais do módulo            */
/*-----------------------------------------*/
USBHost UsbH;
FTDIAsync FtdiAsync;
FTDI Ftdi(&UsbH, &FtdiAsync);

uint8_t FTDIAsync::OnInit(FTDI *pftdi)
{
    uint8_t rcode = 0;

    rcode = pftdi->SetBaudRate(ABNT_BAUD);
    if (rcode)
    {
        ErrorMessage<uint8_t>(PSTR("SetBaudRate"), rcode);
        return rcode;
    }

    rcode = pftdi->SetFlowControl(FTDI_SIO_DISABLE_FLOW_CTRL);
    if (rcode)
        ErrorMessage<uint8_t>(PSTR("SetFlowControl"), rcode);

    return rcode;
}

/**
 * @brief Envia um buffer para o dispositivo USB FTDI
 *
 * Esta função envia um buffer de bytes para o dispositivo
 * FTDI, considerando que este já foi devidamente inciado
 * e está pronto para receber. Esta função pode receber qualquer
 * quantidade de bytes e trata a limitação do driver de enviar
 * apenas 64 bytes de cada vez.
 *
 * @param [in] *pftdi Ponteiro para o objeto FTDI já inciado
 * @param [in] datasize Quantidade de bytes a ser enviada
 * @param [in] *databuf Ponteiro para o início do buffer de dados
 * @return Código de erro, 0 para sucesso
 *
 */
uint8_t send_dataFTDI(uint32_t datasize, volatile uint8_t *databuf)
{
    uint32_t bytes_left = datasize;
    uint8_t bytes_tosend, rcode = 0;
    volatile uint8_t *curr_buf = databuf;

    if (datasize)
    {
        while (bytes_left)
        {
            bytes_tosend = (bytes_left >= MAX_SEND) ? MAX_SEND : bytes_left;
            rcode = Ftdi.SndData(bytes_tosend, (uint8_t *)curr_buf);
            if (rcode)
                break;
            bytes_left -= bytes_tosend;
            curr_buf += bytes_tosend;
        }
        //NOTE: É necessário fazer este envio com zero bytes para garantir
        //      que o envio finalizará corretamente. O cabo USB / porta
        //      ótica ABNT Landis gyr, não funciona sem isso.
        Ftdi.SndData(bytes_left, (uint8_t *)curr_buf);
    }
    return (rcode);
} //send_dataFTDI(

/**
 * @brief Recebe um buffer de dados do dispositivo FTDI
 *
 * Esta função recebe um buffer de dados do dispositivo
 * FTDI, considerando que este já foi devidamente inciado
 * e está pronto para enviar. Esta função utiliza a função
 * do driver FTDI de baixo nível que bufferiza até 64 bytes,
 * não sendo garantido o instante de recebimento efetivo de
 * cada byte.
 *
 * @param [in]  *pftdi Ponteiro para o objeto FTDI já iniciado
 * @param [out] *nBytes Quantidade de bytes recebidos
 * @param [out] *buff Ponteiro para o início do buffer de dados
 *              a ser preenchido
 * @return Código de erro, 0 para sucesso
 *
 */
uint8_t recv_dataFTDI(uint16_t *nBytes, uint8_t *buffPayload)
{
    uint8_t rcode = 0;
    uint8_t buff [MAX_RECV];

    rcode = Ftdi.RcvData(nBytes, buff);

    if (rcode && rcode != USB_ERRORFLOW)
    {
        ErrorMessage<uint8_t>(PSTR("Ret"), rcode);
        return rcode;
    }

    // NOTE: O driver FTDI retorna informações de
    // status (modem e line status registers) nos 2 primeiros
    // bytes, portanto eles não contém dados do payload.
    if (*nBytes > 2)
    {
        (*nBytes) -= 2;
        //Copia o pyaload para a saída
        memcpy(buffPayload, buff+2, *nBytes);
        return 0;
    }
    *nBytes = 0;
    return 0;
} //recv_dataFTDI(
