
#if !defined(__OTICA_FTDI_H__)
#define __OTICA_FTDI_H__

#include <cdcftdi.h>
#include <usbhub.h>
#define DEBUG_FTDI

/******* Funções exportadas *****/

uint8_t send_dataFTDI(uint32_t datasize,  volatile uint8_t *databuf);
//uint8_t send_dataFTDI(uint32_t datasize, uint8_t *databuf);
uint8_t recv_dataFTDI(uint16_t *nBytes, uint8_t *buffPayload);

class FTDIAsync : public FTDIAsyncOper
{
  public:
    uint8_t OnInit(FTDI *pftdi);
};

/********* Variáveis exportadas ********/
extern USBHost UsbH;
extern FTDIAsync FtdiAsync;
extern FTDI Ftdi; /*(&UsbH, &FtdiAsync);*/

#endif
