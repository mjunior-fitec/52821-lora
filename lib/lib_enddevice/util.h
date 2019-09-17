/** @file util.h
  * @brief Arquivo de cabeçalho do modulo util. Definicoes e funcoes de uso
  * geral
  *
  * Este arquivo contem definicoes e funcoes de uso geral, compartilhadas por
  * todos os modulos do projeto.
  *
  * @date Jan/2019
  * @author FITec - Fundacao para Inovacoes Tecnologicas
  *         Marcio J Teixeira Jr. <mjunior@fitec.org.br>
  *
  ************************************************************************/

#if !defined(__UTIL_H__)
#define __UTIL_H__

#include <stdint.h>
#include <RTCZero.h>

#undef min
#undef max

#include <string>

#include "../../include/MKRWAN.h"

#ifdef DEBUG_SERIAL3
#  define SerialDebug Serial3
#endif

#define MAX_SEND    64
#define MAX_RECV    64
#define SIZE_RECV   64

#define NUM_SEG_DIA             (86400)    //(60*60*24)
#define NUM_SEG_HORA            (3600)     //(60*60)
#define OFFSET_LORA_ABNT        (1800)     //(30*60)  //30m
#define OFFSET_LORA_EVENTUAL    (2700)     //(45*60)  //45m
#define OFFSET_ABNT_EVENTUAL    (1200)     //(20*60)  //20m

#define MILLIS_5MIN             (300000L)
#define MILLIS_10MIN            (600000L)
#define MILLIS_15MIN            (900000L)
#define MILLIS_2HORAS           (7200000L)

#define MAX_SEND_RETRY      5
#define T_WAIT_ABP          5000

#define FLASH_VALID     0xA5

extern Uart Serial3;

class FITecModem : public LoRaModem
{
    using LoRaModem::LoRaModem;

public:

    bool isJoined()
    {
        return getJoinStatus();
    } //isJoined()

/**
* @brief Funcao para fazer todo o processo de join, incluindo retentativas
* e reset do modem.
*
* Esta funcao executa o processo completo de join OTAA. Inicialmente verifica
* se ja existe uma sessao ativa e retorna TRUE neste caso, sem iniciar um novo
* processo de Join. Caso contrario reseta o modem e inicia o processo de Join.
* Uma vez iniciado aguarda por tEsperaAccept milisegundos pela resposta de
* Join com sucesso. Caso nao receba a resposta neste tempo, reseta o modem
* novamente e inicia um novo processo de Join, repetindo este procedimento por
* ate nTentativas. Se apos todas as tentativas, nao for recebida uma resposta
* positiva do Modem, retorna FALSE.
*
* @param[in] tEsperaAccept  Tempo de espera pela resposta de JoinAccept (ms)
* @param[in] nTentativas    Numero de tentativas de processos de Join
* @param[in] appEui         Application EUI para validar o Join OTAA no NS
* @param[in] appKey         Application Key para validar o Join OTAA no NS
* @return retrun TRUE indicando que ha uma sessao estabelecida, FALSE caso
* contratio
*
************************************************************************/
    bool fitecJoinOTAA(uint32_t tEsperaAccept, uint8_t nTentativas,\
                       const char *appEui, const char *appKey)
    {
        uint8_t ret;
        auto tStart = millis();

        if (tEsperaAccept < 30000)
            tEsperaAccept = 30000;
        if (nTentativas == 0)
            nTentativas = 1;

        while (nTentativas--)
        {
            // Verifica se ja existe uma sessao aberta com sucesso
            ret = getJoinStatus();
            if (ret)
                return true;
            restart();
            delay(500);
            begin(AU915);
            delay(500);
            publicNetwork(false);
            delay(500);
            configureClass(CLASS_C);
            delay(500);
            changeMode(OTAA);
            delay(1000);
            set(APP_EUI, appEui);
            set(APP_KEY, appKey);
            sendAT(GF("+JOIN"));
            while ((millis() - tStart) < tEsperaAccept)
            {
                delay(500);
                if (getJoinStatus())
                    return true;
            }
            delay(1000);
            tStart = millis();
        }
        return false;
    } //fitecJoinOTAA(

    bool fitecJoinABP(const char *devAddr, const char *nwkSKey, const char *appSKey)
    {
        // Verifica se ja existe uma sessao aberta com sucesso
        auto ret = getJoinStatus();
        if (ret)
            return true;

        restart();
        delay(500);
        begin(AU915);
        delay(500);
        publicNetwork(false);
        delay(300);
        rx.clear();
        changeMode(ABP);
        set(DEV_ADDR, devAddr);
        set(NWKS_KEY, nwkSKey);
        set(APPS_KEY, appSKey);
        sendAT(GF("+JOIN"));
        auto tStart = millis();
        while ((millis() - tStart) < T_WAIT_ABP)
        {
            ret = getJoinStatus();
            if (ret)
                return true;
            delay(200);
        }
        return false;
    } //fitecJoinABP(

    bool initTransmit(void)
    {
        int getACK = -1;
        uint8_t dummy;
        uint8_t maxRetry = MAX_SEND_RETRY;

        if (!isJoined())
            return false;
        while (getACK < 0)
        {
            getACK = modemSend(&dummy, 1, true);
            if (!(maxRetry--))
                return false;
            delay(1000);
        }
        return true;
    }

    template <typename T>
    size_t fitStreamPrint(T param)
    {
        return stream.print(param);
    }

    int fitStreamRead(void)
    {
        return stream.read();
    }

    int fitStreamAvailable(void)
    {
        return stream.available();
    }

    void fitStreamFlush(void)
    {
        return stream.flush();
    }

//private:

};

typedef struct feriado
{
    uint8_t dia;
    uint8_t mes;
    uint8_t ano;
} __attribute__ ((packed)) feriado_t;

typedef struct data
{
    uint8_t dia;
    uint8_t mes;
} __attribute__ ((packed)) data_t;

typedef struct falta
{
    uint8_t horaFalta;
    uint8_t minutoFalta;
    uint8_t segundoFalta;
    uint8_t diaFalta;
    uint8_t mesFalta;
    uint8_t anoFalta;
    uint8_t horaRet;
    uint8_t minutoRet;
    uint8_t segundoRet;
    uint8_t diaRet;
    uint8_t mesRet;
    uint8_t anoRet;
} __attribute__ ((packed)) falta_t;

typedef struct alteracao
{
    uint8_t     codigo;
    uint32_t    numLeitor: 24;
    uint8_t     hora;
    uint8_t     minuto;
    uint8_t     segundo;
    uint8_t     dia;
    uint8_t     mes;
    uint8_t     ano;
} __attribute__ ((packed)) alteracao_t;

typedef enum
{
    COLD_START,
    WARM_START,
} startType_t;

typedef enum
{
    ST_NENHUM,
    ST_NCOMISSIONADO,
    ST_AGUARDA_ACK,
    ST_SEMCOMUNIC,
    ST_SEMABNT,
    ST_SEMLORA,
    ST_NORMAL
} sinalizaStatus_t;

typedef struct itemAgenda
{
    uint32_t tAgenda;         //Instante do alarme (numero de segundos do dia)
    void (* fAgenda) (void);  //Funcao a ser chamada
    struct itemAgenda *next;
} __attribute__ ((packed)) itemAgenda_t;

#define TOGGLEENDIAN16(s)    ((uint16_t)((uint16_t)((s & 0xff00) >> 8) | \
                              (uint16_t)(((s & 0x00ff) << 8) & 0xff00)))
#define TOGGLEENDIAN32(l)    ((uint32_t)((uint32_t)((l & 0x000000ffU) << 24) | \
                              (uint32_t)((l & 0x0000ff00U) <<  8) | \
                              (uint32_t)((l & 0x00ff0000U) >>  8) | \
                              (uint32_t)((l & 0xff000000U) >> 24)))

// Prototipos das funcoes exportadas
uint16_t CalcWeekDayNumFromDate(uint16_t y, uint16_t m, uint16_t d);
uint8_t abntByteToBcd(uint8_t byte);
uint8_t abntBcdToByte(uint8_t bcd);
uint16_t abntBcdToByte(uint16_t bcd);
uint32_t abntBcdToByte(uint32_t bcd);
uint64_t abntBcdToInt(uint8_t *buf, uint8_t nBytes);
uint8_t abntIntToBcd(uint64_t valor, uint8_t *buf);
void stringToBytes (uint8_t *pStream, std::string strIn);
void piscaLed(uint8_t ciclos = 1, uint16_t t_on = 250, uint16_t t_off = 250);
void segundosParaHHMMSS(uint32_t seg, uint8_t *hh, uint8_t *mm, uint8_t *ss);
uint32_t hhmmssParaSegundos (uint8_t hh, uint8_t mm, uint8_t ss);
uint16_t max3 (uint16_t n1, uint16_t n2, uint16_t n3);

#endif
