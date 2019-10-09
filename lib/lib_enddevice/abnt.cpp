/** @file abnt.c
*
* @brief Arquivo contendo o tratamento do protocolo ABNT
*
* Este arquivo contém a função com o tratamento da maquina de estados do
* protocolo ABNT, além das demais funções auxiliares necessarias para
* o tratamento deste protcolo.
* @author FITec - Fundacao para Inovacoes Tecnologicas
*         Marcio J Teixeira Jr. <mjunior@fitec.org.br>
*
************************************************************************/

#include <stdint.h>
#include <RTCZero.h>

#define DEBUG_PROTO

#ifdef DEBUG_PROTO
# include "enddevice.h"
#endif

#include "abnt.h"
#include "otica_ftdi.h"
#include "serial_nativa.h"
#include "ansi_fitec.h"
#include "util.h"
#include "cripto.h"

/**********************************
 * Variaveis do modulo, exportadas
 **********************************/
volatile uint8_t *  pBuffABNTrecv;
volatile uint8_t *  pBuffABNTsend;
volatile uint8_t    stateABNT;
volatile bool       bloqPtABNT = false;
volatile bool       respABNTRecebida;
volatile serialABNT_t portaSerial;
abnt_cmd_altera_hor_verao_t abntAlteraHorVer;
abnt_cmd_altera_segmentos_horarios_t abntProgSegmentosHorarios;
abnt_cmd_altera_feriados_t abntProgFeriados;
volatile uint8_t cmdEstendido;
itemCmdABNT_t cmdAtrasado;
uint8_t programacaoCorteReliga;
uint8_t abntOcorrencia;
uint8_t canaisVisiveis;
uint8_t sementeABNT[10];

/**********************************
 * Variaveis internas ao modulo
 **********************************/
static uint8_t  buffABNTrecv[LEN_RES];
static uint8_t  buffABNTsend[LEN_CMD];

static itemCmdABNT_t listaCmdAEnviar[TAM_LISTA_CMD];
volatile static uint8_t ptEscritaCmd;
volatile static uint8_t ptLeituraCmd;

static itemCmdABNT_t listaCmdAEnviarUrgent[TAM_LISTA_CMD];
volatile static uint8_t ptEscritaCmdUrgent;
volatile static uint8_t ptLeituraCmdUrgent;

static uint8_t  temComando = false;
static uint16_t bytesRecSerialABNT;

static uint32_t tLastENQ;
static uint32_t tAguardaResp;
static uint8_t  abntRetries;

/*
 * Tabela pre-calculada para o polinomio CRC16 (0xA001)
 */
static unsigned short crc16Table[256] = {
    0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241,
    0xc601, 0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440,
    0xcc01, 0x0cc0, 0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40,
    0x0a00, 0xcac1, 0xcb81, 0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841,
    0xd801, 0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40,
    0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01, 0x1dc0, 0x1c80, 0xdc41,
    0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680, 0xd641,
    0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081, 0x1040,
    0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240,
    0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441,
    0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41,
    0xfa01, 0x3ac0, 0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840,
    0x2800, 0xe8c1, 0xe981, 0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41,
    0xee01, 0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40,
    0xe401, 0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640,
    0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0, 0x2080, 0xe041,
    0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281, 0x6240,
    0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480, 0xa441,
    0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41,
    0xaa01, 0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840,
    0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41,
    0xbe01, 0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40,
    0xb401, 0x74c0, 0x7580, 0xb541, 0x7700, 0xb7c1, 0xb681, 0x7640,
    0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080, 0xb041,
    0x5000, 0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0, 0x5280, 0x9241,
    0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440,
    0x9c01, 0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40,
    0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841,
    0x8801, 0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40,
    0x4e00, 0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41,
    0x4400, 0x84c1, 0x8581, 0x4540, 0x8701, 0x47c0, 0x4680, 0x8641,
    0x8201, 0x42c0, 0x4380, 0x8341, 0x4100, 0x81c1, 0x8081, 0x4040};

/**********************************************
 * Prototipos das funcoes internas ao modulo
 **********************************************
*/
uint16_t calcCRC16(unsigned char *buffer, int len);
uint8_t trataEstadoDisconnected();
uint8_t trataEstadoIdle();
uint8_t trataEstadoEnviaCmd();
uint8_t trataEstadoAguardaResp();
uint8_t trataEstadoTrataResp();
uint8_t trataEstadoTrataErr();

void repeteCmdAbnt(void);
void montaCmdAbnt(void);
uint8_t enviaCmdAbnt(void);
void identificaSerial(void);
void TrataOcorrenciaMedidor(uint8_t cod);

uint8_t buscaCmdABNT(void);

//############ Test
extern feriado_t feriadosProgramados[15];
extern inicioPostosHorarios_t segmentosProgramados[4];
//extern uint8_t stringLeitura[10];
//////////////////////////////////////////

/**
 * @brief Funcao para calcular CRC16 do protocolo ABNT
 *
 * Calcula o CRC16 de uma sequencia de bytes passada como parametro, baseado
 * na tabela pre-calculada para o polinomio CRC16 (0xA001)
 *
 * @param[in] buffer vetor de bytes com o conteudo a ser lido
 * @param[in] len tamanho de buffer
 * @return o CRC16 do buffer
*/
uint16_t calcCRC16(unsigned char *buffer, int len)
{
    uint16_t crc = 0;
    int i;

    for (i = 0; i < len; i++)
    {
        crc = (crc >> 8) ^ \
              crc16Table[(crc ^ ((uint16_t)(buffer[i]))) & 0xff];
    }
    return crc;
}

/**
 * @brief Funcao para iniciar o protocolo ABNT
 *
 * Esta funcao inicia o protocolo ABNT. Aqui a maquina de estados e colocada
 * no seu estado inicial - ABNT_STATE_DISCONNECTED e todas as variaveis
 * recebem seus respectivos valores iniciais.
 */
void abntInit(void)
{
    stateABNT = ABNT_STATE_DISCONNECTED;
    abntRetries = 0;
    tLastENQ = 0; //@attention Eh necessario ???
    temComando = false;
    respABNTRecebida = false;
    pBuffABNTrecv = buffABNTrecv;
    pBuffABNTsend = buffABNTsend;
    portaSerial.interface = SERIAL_NULL;
    ptEscritaCmd = ptLeituraCmd = 0;
    ptEscritaCmdUrgent = ptLeituraCmdUrgent = 0;
    cmdEstendido = 0;
} //abntInit(

/**
 * @brief Funcao para tratar a maquina de estados ABNT
 *
 * Esta funcao implementa a maquina de estados do protocolo ABNT, sem se
 * preocupar com a interface de comunicacao. Cada estado tem uma funcao de
 * tratamento que retorna o proximo estado. A variavel de controle do estado
 * atual #stateABNT e exportada para visibilidade por outros modulos.
 */
void maqEstAbnt(void)
{
    //SerialDebug.println("maqEstAbnt()\r\n st= " + String(stateABNT));
    switch(stateABNT)
    {
        case ABNT_STATE_DISCONNECTED:
            stateABNT = trataEstadoDisconnected();
        break;

        case ABNT_STATE_IDLE:
            stateABNT = trataEstadoIdle();
        break;

        case ABNT_STATE_ENVIA_CMD:
            stateABNT = trataEstadoEnviaCmd();
        break;

        case ABNT_STATE_AGUARDA_RESP:
            stateABNT = trataEstadoAguardaResp();
        break;

        case ABNT_STATE_TRATA_RESP:
            stateABNT = trataEstadoTrataResp();
        break;

        case ABNT_STATE_TRATA_ERR:
            stateABNT = trataEstadoTrataErr();
        break;

        default:
            // Se a maquina ficar perdida, volta para Disconnected
            stateABNT = ABNT_STATE_DISCONNECTED;
        break;
    }
} //maqEstAbnt(

/**
 * @brief Funcao para tratar o estado ABNT_STATE_DISCONNECTED
 *
 * Esta funcao verfiica se recebeu um ENQ como inicio de sincronismo, se sim
 * muda para o estado ABNT_STATE_IDLE. Senao, permanece no estado
 * ABNT_STATE_DISCONNECTED
 *
 * * @return Próximo estado
 */
uint8_t trataEstadoDisconnected(void)
{
    //SerialDebug.println("EstadoDisconnected()");
    uint8_t ret = ABNT_STATE_DISCONNECTED;
    identificaSerial();
    if (portaSerial.interface != SERIAL_NULL)
    {
        bytesRecSerialABNT = recvMsgMedidorABNT((abnt_resp_generic_t *) \
                                                buffABNTrecv);
        if (1 == bytesRecSerialABNT)
        {
            if (ABNT_ENQ == buffABNTrecv[0])
            {
                tLastENQ = millis();
                ret = ABNT_STATE_IDLE;
            }
        }
    }
    return (ret);
} //trataEstadoDisconnected(

/**
 * @brief Funcao para tratar o estado ABNT_STATE_IDLE
 *
 * Esta funcao verfiica se recebeu um ENQ, se sim marca o instante para medir
 * o tempo sem ENQ. Se já ficou mais tempo que o maximo permitido sem ENQ,
 * desconecta mudando para o estado ABNT_STATE_DISCONNECTED.
 * Se existe algum comando a ser enviado, muda para o estado
 * ABNT_STATE_ENVIA_CMD.
 *
 * * @return Próximo estado
 */
uint8_t trataEstadoIdle(void)
{
    //SerialDebug.println("EstadoIdle()");
    uint8_t ret = ABNT_STATE_IDLE;

    bytesRecSerialABNT = recvMsgMedidorABNT((abnt_resp_generic_t *) \
                                             buffABNTrecv);

    //SerialDebug.println("Aft recvMsgMedidorABNT)");
    if (1 == bytesRecSerialABNT)
    {
        if (ABNT_ENQ == buffABNTrecv[0])
        {
            tLastENQ = millis();
        }
    }
    if ((millis() - tLastENQ) > T_MAX_ENQ)
        ret = ABNT_STATE_DISCONNECTED;

    temComando = buscaCmdABNT();
    if (temComando)
    {
        abntRetries = 0;
        ret = ABNT_STATE_ENVIA_CMD;
    }

    //SerialDebug.println("Fim EstadoIdle()");
    return (ret);
} //trataEstadoIdle(

/**
 * @brief Funcao para tratar o estado ABNT_STATE_ENVIA_CMD
 *
 * Esta funcao trata o estado ABNT_STATE_ENVIA_CMD. Neste estado o comando
 * e montado e enviado/reenviado. Caso nao seja possivel concluir o envio
 * com sucesso ou seja ultrapassado o maximo de retentativas, muda para o
 * estado ABNT_STATE_TRATA_IDLE.
 * Se o comando for enviado com sucesso, muda para o estado
 * ABNT_STATE_AGUARDA_RESP.
 *
 * @return Próximo estado
 */
uint8_t trataEstadoEnviaCmd(void)
{
    uint8_t ret = ABNT_STATE_ENVIA_CMD;
    uint8_t err_envio = 0;

    if (abntRetries)
        repeteCmdAbnt();
    else
        montaCmdAbnt();

    err_envio = enviaCmdAbnt();

    if (err_envio)
        if (++abntRetries > ABNT_MAX_RETRIES)
            ret = ABNT_STATE_IDLE;
        else
            ret = ABNT_STATE_ENVIA_CMD;
    else
    {
        tAguardaResp = millis();
        abntRetries = 0;
        ret = ABNT_STATE_AGUARDA_RESP;
    }

    return (ret);
} //trataEstadoEnviaCmd(

/**
 * @brief Funcao para tratar o estado ABNT_STATE_AGUARDA_RESP
 *
 * Esta funcao trata o estado ABNT_STATE_AGUARDA_RESP.
 * Neste estado é verificado o recebimento de um bloco de resposta
 * ABNT completo. Caso seja recebido um bloco, sua integridade eh
 * verificada pelo CRC.
 * Se a resposta for integra, muda para o estado ABNT_STATE_TRATA_RESP,
 * senao muda para o estado ABNT_STATE_TRATA_ERR.
 *
 * @todo Verificar necessidade de disparar reenvio de cmd neste estado
 *
 * * @return Próximo estado
 */
uint8_t trataEstadoAguardaResp(void)
{
    uint8_t ret = ABNT_STATE_AGUARDA_RESP;

    if ((millis() - tAguardaResp) > T_MAX_AGUARDA_RESP)
    {
        temComando = 0;
        tLastENQ = millis();
        ret = ABNT_STATE_IDLE; //###Verificar se eh necessario
    }
    bytesRecSerialABNT = recvMsgMedidorABNT((abnt_resp_generic_t *) \
                                            buffABNTrecv);

    if (1 == bytesRecSerialABNT)
    {
        if (ABNT_NACK == buffABNTrecv[0])
        {
            enviaCmdAbnt(); //Se recebeu NACK, repete comando
            return (ret);
        }
    }
    // verifica tamanho do bloco de bytes recebidos.
    // Se já recebeu o tamanho de uma resposta, valida CRC.
    if (LEN_RES == bytesRecSerialABNT)
    {
        if (calcCRC16(buffABNTrecv, LEN_RES))
        {
            if (++abntRetries < ABNT_MAX_RETRIES)
            {
                buffABNTsend[0] = ABNT_NACK;
                if (portaSerial.sendCmdMedidor(1, pBuffABNTsend))
                    ret = ABNT_STATE_AGUARDA_RESP;
                else
                    ret = ABNT_STATE_TRATA_ERR;
            }
            else
                ret = ABNT_STATE_TRATA_ERR;
        }
        else
            ret = ABNT_STATE_TRATA_RESP;
    }
    return (ret);
} //trataEstadoAguardaResp(

/**
 * @brief Funcao para tratar o estado ABNT_STATE_TRATA_RESP
 *
 * Esta funcao trata o estado ABNT_STATE_TRATA_RESP.
 * Neste estado os dados recebido sao interpretados e despachados
 * para os modulos correspondentes.
 * Se a resposta for integra, muda para o estado ABNT_STATE_TRATA_RESP,
 * senao muda para o estado ABNT_STATE_TRATA_ERR.
 *
 * * @return Próximo estado
 */
uint8_t trataEstadoTrataResp(void)
{
    uint8_t ret = ABNT_STATE_TRATA_RESP;

    respABNTRecebida = true;
    ret = ABNT_STATE_TRATA_RESP;

    return (ret);
} //trataEstadoTrataResp(

/**
 * @brief Funcao para tratar o estado ABNT_STATE_TRATA_ERR
 *
 * Esta funcao trata o estado ABNT_STATE_TRATA_ERR.
 * Neste estado sao tratadas as situacoes de erro
 * para os modulos correspondentes.
 * Se a resposta for integra, muda para o estado ABNT_STATE_TRATA_RESP,
 * senao muda para o estado ABNT_STATE_TRATA_ERR.
 *
 * @todo Implementar tratamento de erro
 *
 * @return Próximo estado
 */
uint8_t trataEstadoTrataErr(void)
{
    //uint8_t ret = ABNT_STATE_TRATA_ERR;

    ////////
    abntInit();

    return (ABNT_STATE_DISCONNECTED);
} //trataEstadoTrataErr(

/**
 * @brief Função para receber uma resposta ou sinalizador do medidor ABNT
 *
 * Esta função recebe uma resposta ou um sinalizador do medidor ABNT,
 * tratando o envio de ACKs e NACKs diretamente para o medidor, timeout com
 * resposta incompleta além de remover sequência de ENQs acumulados.
 * @param [in] timeout Tempo maximo de espera, em milissegundos
 * @param [out] resp Estrutura genérica para qualquer resposta do protocolo
 * @return -1 para erro na recepção, 0 para timeout, positivo para o tamanho
 * do pacote de resposta
*/

int16_t recvMsgMedidorABNT(abnt_resp_generic_t *resp)
{
    // se for chamada sem timeout, usa default de 1,5s
    return recvMsgMedidorABNT (resp, 1500);
} //recvMsgMedidorABNT(

int16_t recvMsgMedidorABNT(abnt_resp_generic_t *resp, uint16_t timeout)
{
    //SerialDebug.println("recvMsgMedidorABNT()");
    uint8_t err_rec;
    uint8_t bufferTmp[MAX_RECV];
    uint8_t *buffResp;
    uint16_t bytesRec, i;
    uint32_t tStart;
    bool recebendo = false;
    bool respOk = false;

    buffResp = (uint8_t *)resp;
    tStart = millis();
    bytesRecSerialABNT = 0;

    //Recebe cada bloco de dados num buffer temporário
    memset(bufferTmp, 0, MAX_RECV);
    bytesRec = MAX_RECV;
    err_rec = portaSerial.recMsgMedidor(&bytesRec, bufferTmp);

    //###
    //SerialDebug.println("Init err_rec= " + String(err_rec) + " bytes: " + String(bytesRec));
    // SerialDebug.println("Buffer recebido:");
    // for (uint16_t iBuf = 0; iBuf<bytesRec; iBuf++)
    //     SerialDebug.println("  [" + String(iBuf) + "]: " + String(bufferTmp[iBuf], HEX));
    // SerialDebug.println();

    while (!respOk)
    {
        if (((millis() - tStart)) > timeout)
        {
            return 0;
        }
        if (err_rec)
            return -1;

        if (bytesRec)
        {
            //SerialDebug.println("bRec= " + String(bytesRec));
            i = 0;
            if (!recebendo)
            {
                if (1 == bytesRec) //Msg de 1 byte
                {
                    //Trata a recepção de sinalizadores (msg de 1 byte)
                    if ((bufferTmp[0] == ABNT_ACK) ||  \
                        (bufferTmp[0] == ABNT_NACK) || \
                        (bufferTmp[0] == ABNT_ENQ) ||  \
                        (bufferTmp[0] == ABNT_WAIT) )
                    {
                        buffResp[0] = bufferTmp[0];
                        bytesRecSerialABNT = 1;
                        return bytesRecSerialABNT; //Retorna o tamanho
                    }
                }
                recebendo = true;
                //SerialDebug.println("RECEBENDO !!! <-----");

                //Remove possiveis ENQs recebidos em sequência
                if (ABNT_ENQ == bufferTmp[0])
                {
                    //SerialDebug.println("Removendo ENQs...");
                    for (i = 0; (i + 1) < bytesRec;)
                    {
                        if (ABNT_ENQ != bufferTmp[++i])
                            break;
                    }
                }
            }
            //Aqui, i contem o índice para o primeiro byte do payload.
            //Se recebeu apenas uma sequência de ENQs, i apontará para o
            //último ENQ.
            //Se recebeu ENQs seguido de um bloco de dados, i apontará para
            //o início do bloco sem ENQs. Assim o valor de i representa o
            //número de bytes a ser desprezado. Nos casos normais, sem
            //recebimento de ENQs, i será sempre 0.
            bytesRec -= i;

            //###
            //SerialDebug.println("By real: " + String(bytesRec));

            memcpy(buffResp, bufferTmp + i, bytesRec);
            //atualiza o ponteiro para receber o restante
            buffResp += bytesRec;
            //atualiza a quantidade recebida
            bytesRecSerialABNT += bytesRec;

            if (bytesRecSerialABNT < LEN_RES)
            {
                bytesRec = MAX_RECV;
                delay(8);
                err_rec = portaSerial.recMsgMedidor(&bytesRec, bufferTmp);

                // //####
                // SerialDebug.println("N err_rec= " + String(err_rec) + " bytes: " + String(bytesRec));
                // SerialDebug.println("Buffer recebido:");
                // for (uint16_t iBuf = 0; iBuf < bytesRec; iBuf++)
                //     SerialDebug.println("  [" + String(iBuf) + "]: " + String(bufferTmp[iBuf], HEX));
                // SerialDebug.println();
                /////
            }
            else
            {
                recebendo = false;
                respOk = true;
                // Garante que sai do while com a resposta completa
            }
        }
        else
        {
            bytesRec = MAX_RECV;
            delay(10);
            err_rec = portaSerial.recMsgMedidor(&bytesRec, bufferTmp);
            //###
            // SerialDebug.println("M err_rec= " + String(err_rec) + " bytes: " + String(bytesRec));
            // SerialDebug.println("Buffer recebido:");
            // for (uint16_t iBuf = 0; iBuf < bytesRec; iBuf++)
            //     SerialDebug.println("  [" + String(iBuf) + "]: " + String(bufferTmp[iBuf], HEX));
            // SerialDebug.println();
            //////////
        }
    }
    return bytesRecSerialABNT;
} //recvMsgMedidorABNT()

void repeteCmdAbnt(void)
{
    return;
} //repeteCmdAbnt(

void montaCmdAbnt(void)
{
    abnt_cmd_generic_t cmdABNT;

    //Inicia a estrutura de comnado com os valores comuns a todos comandos
    //e restante do payload zerado.
    cmdABNT.id = temComando;
    cmdABNT.num_serie_leitor = NUM_LEITOR;
    memset(&(cmdABNT.payload), 0, sizeof(cmdABNT.payload));

    //Preenche o payload com dados especificos para cada comando que
    //posui dados no payload
    switch (temComando)
    {
    case ID_CMD11:
    {
        abnt_cmd_abertura_sessao_t *cmd_abertura_sessao;
        cmd_abertura_sessao = (abnt_cmd_abertura_sessao_t *)&cmdABNT;

        uint8_t token[10];
        uint8_t inputHash[24];
        std::string inputHashStr;

        if (EH_MEDIDOR_RESIDENCIAL(localKeys.modeloMedidor))
        {
            //DEBUG!
            SerialDebug.println(" -- Identificado residencial!");

            static uint8_t numSerieMedidor[10];
            //Num serie em BCD para montar o hash
            abntIntToBcd(localKeys.numSerieMedidor, numSerieMedidor);
            //Input para o MD5 (E430 e E450) = NumSerie | seed | key
            memcpy(inputHash, (void *)numSerieMedidor, 4);
            memcpy(inputHash + 4, (void *)sementeABNT, 10);
            memcpy(inputHash + 14, (void *)localKeys.senhaABNT, 10);

            SerialDebug.println("Input: ");
            for (uint8_t i = 0; i < 24; i++)
                SerialDebug.print(String(inputHash[i], HEX) + " ");
            SerialDebug.println("");
            SerialDebug.flush();

            //A funcao MD5 so recebe String como entrada, assim e necessario
            //copiar o vetor de bytes numa variavel string sem converter para ASCII
            inputHashStr.clear();
            inputHashStr.append((char *)inputHash, 24);
            auto calcMD5 = md5(inputHashStr);

            //*** No algoritmo para modelo residencial, o input sao 24 bytes e
            // o hash calculado tem 16 bytes!
            //****************************************************************

            //O resultado da funcao MD5 e uma string contendo a representacao
            //hexa do hash calculado, assim e necessario converter para bytes
            static uint8_t calcMD5_[16];
            stringToBytes(calcMD5_, calcMD5);

            //O token, a ser usado no comando de abertura de sessao, e
            //constituido apenas dos primeiros 10 bytes do MD5 calculado
            for (uint8_t i = 0; i < 10; i++)
                token[i] = calcMD5_[i];

            SerialDebug.println("Hash: ");
            for (uint8_t i = 0; i < 16; i++)
                SerialDebug.print(String(calcMD5_[i], HEX) + " ");
            SerialDebug.println();
            SerialDebug.flush();

            SerialDebug.println("Token: ");
            for (uint8_t i = 0; i < 10; i++)
                SerialDebug.print(String(token[i], HEX) + " ");
            SerialDebug.println();
            SerialDebug.flush();
        }

        //-----
        // ICG (E650 e E750) - SHA-256
        //-----
        else if (EH_MEDIDOR_ICG(localKeys.modeloMedidor))
        {
            //DEBUG!
            SerialDebug.println(" -- Identificado ICG!");
            memcpy(inputHash, (void *)localKeys.senhaABNT, 10);
            memcpy(inputHash + 10, (void *)sementeABNT, 10);

            SerialDebug.println("Input: ");
            for (uint8_t i = 0; i < 20; i++)
                SerialDebug.print(String(inputHash[i], HEX) + " ");
            SerialDebug.println("");
            SerialDebug.flush();

            inputHashStr.clear();
            inputHashStr.append((char *)inputHash, 20);
            auto calcSHA = sha256(inputHashStr);

            //*** No algoritmo para modelos ICG, o input sao 20 bytes e
            // o hash calculado tem 32 bytes!
            //****************************************************************

            //O resultado da funcao SHA256 eh uma string contendo a representacao
            //hexa do hash calculado, assim e necessario converter para bytes
            static uint8_t calcSHA_[32];
            stringToBytes(calcSHA_, calcSHA);

            SerialDebug.println("Hash: ");
            for (uint8_t i = 0; i < 32; i++)
                SerialDebug.print(String(calcSHA_[i], HEX) + " ");
            SerialDebug.println();
            SerialDebug.flush();

            for (uint8_t i = 31, j = 0; j < 10; i--, j++)
                token[j] = calcSHA_[i];

            SerialDebug.println("Token: ");
            for (uint8_t i = 0; i < 10; i++)
                SerialDebug.print(String(token[i], HEX) + " ");
            SerialDebug.println();
            SerialDebug.flush();

        }
        else
        {
            //Erro modelo de medidor invalido
            SerialDebug.println("Modelo nao identificado!!!");
            SerialDebug.flush();
            ansi_tab5.alarmes.senha_abnt = 1;
            insereTabelaANSI(TAB_ANSI05, SIZE_TABELA5_CMD40);
            localKeys.senhaABNT_ok = false;
            salvarNaoVolatil = true;
            return;
        }

        memcpy(cmd_abertura_sessao->criptografia, token, sizeof(token));
        // SerialDebug.println("Payload do cmd 11:");
        // for (uint8_t i = 0; i < 30; i++)
        //     SerialDebug.print(String(cmdABNT.payload[i], HEX) + " ");
        // SerialDebug.println();
        // SerialDebug.flush();

        break;
    }

    case ID_CMD21:
        cmdABNT.payload[1] = cmdEstendido;
        canaisVisiveis = cmdEstendido;
        break;

    case ID_CMD25:
        cmdABNT.payload[0] = 1; //0: anteriores; 1: atuais
        break;

    case ID_CMD29:
        cmdABNT.payload[0] = abntByteToBcd(rtc.getDay());
        cmdABNT.payload[1] = abntByteToBcd(rtc.getMonth());
        cmdABNT.payload[2] = abntByteToBcd(rtc.getYear());
        cmdABNT.payload[3] = CalcWeekDayNumFromDate(rtc.getYear(),
                                    rtc.getMonth(), rtc.getDay());
        break;

    case ID_CMD30:
        cmdABNT.payload[0] = abntByteToBcd(rtc.getHours());
        cmdABNT.payload[1] = abntByteToBcd(rtc.getMinutes());
        cmdABNT.payload[2] = abntByteToBcd(rtc.getSeconds());
        break;

    case ID_CMD32:
    {
        abnt_cmd_altera_feriados_t *cmdABNTFeriados;
        cmdABNTFeriados = (abnt_cmd_altera_feriados_t *)&cmdABNT;
        memcpy(cmdABNTFeriados->feriados, &(abntProgFeriados.feriados),
               sizeof(abntProgFeriados.feriados));
        break;
    }
    case ID_CMD35:
    {
        abnt_cmd_altera_segmentos_horarios_t *cmdABNTSegHorarios;
        cmdABNTSegHorarios = (abnt_cmd_altera_segmentos_horarios_t *)&cmdABNT;
        memcpy(cmdABNTSegHorarios->progPostoHor,
               &(abntProgSegmentosHorarios.progPostoHor),
               sizeof(abntProgSegmentosHorarios.progPostoHor));
        cmdABNTSegHorarios->numSegmentos =
            abntProgSegmentosHorarios.numSegmentos;
        break;
    }
    case ID_CMD37:
        cmdABNT.payload[0] = abntOcorrencia;
        break;

    case ID_CMD64:
    {
        abnt_cmd_altera_hor_verao_t *cmdABNTHorVerao;
        cmdABNTHorVerao = (abnt_cmd_altera_hor_verao_t *)&cmdABNT;
        memcpy(&(cmdABNTHorVerao->horarioVerao),
               &(abntAlteraHorVer.horarioVerao),
               sizeof(abntAlteraHorVer.horarioVerao));
        break;
    }
    case ID_CMD87:
        cmdABNT.payload[0] = 1; // 0: Alteracao, 1: Leitura
        break;

    case ID_CMDE2:
        cmdABNT.payload[0] = cmdEstendido;
        if (cmdEstendido == CMD_LANDIS_CORTE_RELIGA)
        {
            switch (programacaoCorteReliga)
            {
            case ANSI_PARAM_LEITURA:
                cmdABNT.payload[1] = LG_LEITURA;
                break;

            case ANSI_PARAM_CORTE:
                cmdABNT.payload[1] = LG_CORTE;
                break;

            case ANSI_PARAM_RELIGA:
                cmdABNT.payload[1] = LG_RELIGA;
                break;

            default:
                break;
            }
        }
        break;

    default:
        break;
    }

    cmdABNT.crc = calcCRC16((unsigned char *)&cmdABNT, PAYLOAD_CMD);
    memcpy(buffABNTsend, &cmdABNT, LEN_CMD);

    /// --- DEBUG
    // SerialDebug.println("\r\n Buffer a ser enviado (ABNT):");
    // for (uint8_t i = 0; i < 66; i++)
    // {
    //     SerialDebug.print(String(pBuffABNTsend[i], HEX) + " ");
    // }
    // SerialDebug.println();
    ///

    return;
} //montaCmdAbnt(

void sinaliza_cmd_abnt(uint8_t IdCmd)
{
    temComando = IdCmd;
    return;
} //sinaliza_cmd_abnt(

uint8_t enviaCmdAbnt(void)
{
    uint8_t rcode;
    rcode = portaSerial.sendCmdMedidor(LEN_CMD, pBuffABNTsend);
    return rcode;
} //enviaCmdAbnt(

void identificaSerial(void)
{
    #ifndef USB_DEBUG
    UsbH.Task();
    if (USB_STATE_RUNNING == UsbH.getUsbTaskState())
    {
        portaSerial.interface = SERIAL_USB;
        portaSerial.recMsgMedidor = recv_dataFTDI;
        portaSerial.sendCmdMedidor = send_dataFTDI;
    }
    else
    #endif
    if (temMedidorSerial())
    {
        portaSerial.interface = SERIAL_RS232;
        portaSerial.recMsgMedidor = recv_dataSerial;
        portaSerial.sendCmdMedidor = send_dataSerial;
        Serial1.end();
        Serial1.begin(ABNT_BAUD);
        Serial1.setTimeout(250); /////// <--VERIFICAR ESTES VALORES !!!
    }
    else
        portaSerial.interface = SERIAL_NULL;
} //identificaSerial(

void abntRespostaTratada()
{
    respABNTRecebida = false;
    stateABNT = ABNT_STATE_IDLE;
} //abntRespostaTratada(

/**
* @brief Funcao para tratamento de ocorrencia recebida do medidor ABNT
*
* Esta funcao trata o recebimento de uma ocorrencia recebida do medidor ABNT
* colocando esta informacao na respectiva tabela ANSI-FITec a ser enviada.
*
* @param[in] cod codigo da ocorrencia recebida
*
************************************************************************/
void TrataOcorrenciaMedidor(uint8_t cod)
{
    switch (cod)
    {
        default:
            break;
    }
} //TrataOcorrenciaMedidor(

/**
* @brief Funcao para inserir um comando ABNT a ser enviado para o medidor
*
* Esta funcao recebe o codigo de um comando ABNT a ser enviado para o medidor
* e o inclui na lista de comandos que sera enviado no tratamento do protocolo.
* O comando pode ser colocado na lista normal ou na lista urgente, e o
* parametro para comando estendido tambem e passado, opcionalmente.
*
* @param[in] lista tipo da lista que recebe o comando (Normal ou Urgente)
* @param[in] IdCmd Numero do comando a ser inserido na lista
* @param[in] estendido parametro para comando estendido (opcional)
* @return true caso o comando seja inserido corretamente, false em caso de erro
*
************************************************************************/
bool insereCmdABNT(tipoLista_t lista, uint8_t IdCmd, uint8_t estendido)
{
    volatile uint8_t    *ptEscrita,
                        *ptLeitura;
    itemCmdABNT_t       *listaCmd;

    if (LISTA_URGENTE == lista)
    {
        ptEscrita = &ptEscritaCmdUrgent;
        ptLeitura = &ptLeituraCmdUrgent;
        listaCmd = listaCmdAEnviarUrgent;
    }
    else
    {
        ptEscrita = &ptEscritaCmd;
        ptLeitura = &ptLeituraCmd;
        listaCmd = listaCmdAEnviar;
    }

    if (LISTA_NEXT_PT(*ptEscrita) == LISTA_PT(*ptLeitura))
        return false; //lista cheia

    bloqPtABNT = true; //Bloqueia acesso a lista para evitar inconsistencia
    itemCmdABNT_t rec;
    rec.cmd = IdCmd;
    rec.codEstendido = estendido;
    listaCmd[LISTA_INC_PT(*ptEscrita)] = rec;
    bloqPtABNT = false; //Libera acesso a lista
    return true;
} //insereCmdABNT(

/**
* @brief Funcao para recuperar um comando ABNT da lista de comandos a ser
* enviada
*
* Esta funcao retorna o codigo do proximo comando da lista a ser enviado para
* o medidor. A lista de comandos e um buffer circular, assim caso os ponteiros
* de escrita e leitura sejam coincidentes, retorna zero indicando que a lista
* esta vazia.
* A funcao verifica primeiro se ha algum comando na lista urgente, so
* verificando a lista normal quando esta esta vazia.
*
* @return Codigo do proximo comando, zero para lista vazia
*
************************************************************************/
uint8_t buscaCmdABNT(void)
{
    uint8_t ret;
    volatile uint8_t    *ptEscrita,
                        *ptLeitura;
    itemCmdABNT_t       *listaCmd;

    //Se ha algo na lista urgente, usa esta
    if (LISTA_PT(ptLeituraCmdUrgent) != LISTA_PT(ptEscritaCmdUrgent))
    {
        ptEscrita = &ptEscritaCmdUrgent;
        ptLeitura = &ptLeituraCmdUrgent;
        listaCmd = listaCmdAEnviarUrgent;
    }
    else
    {
        if (LISTA_PT(ptLeituraCmd) == LISTA_PT(ptEscritaCmd))
            return 0;
        ptEscrita = &ptEscritaCmd;
        ptLeitura = &ptLeituraCmd;
        listaCmd = listaCmdAEnviar;
    }
    bloqPtABNT = true; //Bloqueia acesso a lista para evitar inconsistencia
  //DEBUG###
    SerialDebug.println("\r\nABNT - Wr: " + String(LISTA_PT(*ptEscrita))
                        + "\tRd: " + String(LISTA_PT(*ptLeitura)));
    SerialDebug.println("A enviar:\tcmd= "
                    + String(listaCmd[LISTA_PT(*ptLeitura)].cmd, HEX)
                    + "\testend= "
        + String(listaCmd[LISTA_PT(*ptLeitura)].codEstendido, HEX));
    SerialDebug.println("visib = " + String(canaisVisiveis));
  ////
    cmdEstendido = listaCmd[LISTA_PT(*ptLeitura)].codEstendido;
    ret = listaCmd[LISTA_INC_PT(*ptLeitura)].cmd;
    bloqPtABNT = false; //Libera acesso a lista

    // //--- log de sanidade
    // localKeys.log_sanidade.ptEscritaABNT = ptEscritaCmd;
    // localKeys.log_sanidade.ptLeituraABNT = ptLeituraCmd;
    // localKeys.log_sanidade.ptEscritaABNTUrgente = ptEscritaCmdUrgent;
    // localKeys.log_sanidade.ptLeituraABNTUrgente = ptLeituraCmdUrgent;
    return ret;
} //buscaCmdABNT(

itemCmdABNT_t buscaUltimoCmd(void)
{
    itemCmdABNT_t ret;
    abnt_cmd_generic_t *cmd;
    cmd = (abnt_cmd_generic_t *)buffABNTsend;
    ret.cmd = cmd->id;
    ret.codEstendido = cmd->payload[0];

    return ret;
}

/**************************
 *  os seguintes comandos ABNT devem ser implementados no end device:
/ 11 – Pedido de abertura de sessão com senha
/ 13 – Pedido de string para cálculo de senha
/ 14 – Leitura das grandezas instantâneas - cmd vazio
/ 21 – Leitura dos parâmetros atuais      - cmd vazio*
/ 23 – Leitura dos registradores atuais   - cmd vazio
/ 25 – Leitura das faltas de energia      - 1 byte, sem estrutura
/ 28 – Leitura dos registros de alteração - cmd vazio
/ 29 – Alteração de data                  - dados sem estrutura
/ 30 – Alteração de hora                  - dados sem estrutura
/ 32 – Alteração de feriados              - cmd com struct
/ 35 – Alteração de postos horários       - cmd com struct
/ 64 – Alteração do horário de verão      - cmd com struct
/ 80 - Leitura de parametros de medicao   - cmd vazio (necessario para ler posto horario INTERMEDIARIO)
/ 87 - Leitura cód instalacao             - cmd vazio
E2 41 - Leitura DRP e DRC                 - cmd vazio
E2 90 - Corte / religa                    - cmd com dado simples
E2 2B - Le parametros atuais, estendido   - cmd vazio
E2 50 - Leitura de monitores antifraude (alarmes)
40 - Condicao de ocorrecia
37 - Alteracao da condicao de ocorrencia (ALARME) - 1 byte, sem estrutura
Leitura de DRP (Duração Relativa de Transgressão da Tensão Precária) e DRC (Duração Relativa da Transgressão de Tensão Crítica).

***********************************
Senhas Landis+Gyr
E-430: TAETEEEUGT
E-450: EUEEEETUGB
E-650: EBJEEEEYGA
***********************************/
