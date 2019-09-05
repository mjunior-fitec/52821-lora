/** @file util.cpp
  * @brief Funcoes de utilidade geral
  *
  * Este arquivo contém a implementacao de funcoes de uso geral
  * a serem usadas por todos os modulos do sistema
  *
  * @date Jan/2019
  * @author FITec - Fundacao para Inovacoes Tecnologicas
  *         Marcio J Teixeira Jr. <mjunior@fitec.org.br>
  *
  ************************************************************************/
#include <Arduino.h>
#include <stdint.h>
#include <RTCZero.h>

#define DEBUG_UTIL

#ifdef DEBUG_UTIL
# include "enddevice.h"
#endif

#include "util.h"

/**
* @brief Calcula dia da semana
*
* Funcao para calcular dia da semana a partir dos parametros da data
* (dia, mes, ano). Retorna o dia da semana no formato ABNT, DOM=1, SAB=7
*
* @param[in] y ano
* @param[in] m mes
* @param[in] d dia
* @return dia da semana
*
************************************************************************/
uint16_t CalcWeekDayNumFromDate(uint16_t y, uint16_t m, uint16_t d)
{
    m = (m + 9) % 12;
    y -= m / 10;
    uint16_t dn = 365 * y + y / 4 - y / 100 + y / 400 + (m * 306 + 5) / 10 + (d - 1);

    return (((dn + 3) % 7) + 1); //Para retornar DOM=1, SEG=2... SAB=7
} //CalcWeekDayNumFromDate(

/**
* @brief Converte numero para BCD
*
* Funcao que converte um valor para um byte em notacao BCD.
*
* @param[in] byte_in valor a ser convertido
* @return representacao em BCD, 0x99 para valor invalido
*
************************************************************************/
uint8_t abntByteToBcd(uint8_t byte_in)
{
    uint8_t bcd;

    if (byte_in > 99)
    {
        return 0x99;
    }
    bcd = (byte_in % 10) | ((byte_in / 10) << 4);
    return bcd;
} //abntByteToBcd(

/**
* @brief Converte BCD para valor inteiro
*
* Funcao que converte um byte em notacao BCD para seu valor inteiro.
*
* @param[in] bcd representacao em BCD do valor a ser convertido
* @return valor inteiro
*
************************************************************************/
uint8_t abntBcdToByte(uint8_t bcd)
{
    return (((bcd >> 4)*10) + (bcd & 0xF));
} //abntBcdToByte(

uint16_t abntBcdToByte(uint16_t bcd)
{
    return (((abntBcdToByte((uint8_t)(bcd >> 8)))*100) + (abntBcdToByte((uint8_t)(bcd & 0xFF))));
} //abntBcdToByte(

uint32_t abntBcdToByte(uint32_t bcd)
{
    return (((abntBcdToByte((uint16_t)(bcd >> 16)))*10000) + (abntBcdToByte((uint16_t)(bcd & 0xFFFF))));
} //abntBcdToByte(

uint64_t abntBcdToInt(uint8_t *buf, uint8_t nBytes)
{
    uint64_t result = 0;

    for (uint8_t i=0; i<nBytes; ++i)
    {
        result *= 100;
        result += abntBcdToByte(buf[i]);
    }
    return result;
}
// abntBcdToInt(

uint8_t abntIntToBcd(uint64_t valor, uint8_t *buf)
{
    uint8_t i = 0;

    while (valor)
    {
        buf[0] = abntByteToBcd((uint8_t)(valor % 100));
        valor /= 100;
        i++;
        if (valor)
        {
            for (uint8_t j = i; j > 0; j--)
                buf[j] = buf[j - 1];
        }
    }
    return i;
}
// abntIntToBcd(

/**
* @brief Funcao para converter uma string contendo a representacao de bytes
* em ASCII para os bytes em hexa propriamente ditos
*
* Esta funcao converte uma string que contem a representacao de bytes em ASCII
* para os bytes em hexa propriamente ditos. E especialmente util para converter
* o resultado das funcoes de criptografia MD5 e SHA256, que sao Strings, para
* vetores de bytes.
*
* @param[in] strIn String contendo o valor a ser convertido
* @param[out] pStream vetor de bytes que vai receber os dados convertidos
*
************************************************************************/
void stringToBytes (uint8_t *pStream, std::string strIn)
{
    for (uint8_t i = 0; i < strIn.length(); i++)
    {
        uint8_t msb = tolower(strIn.c_str()[2*i]);
        uint8_t lsb = tolower(strIn.c_str()[(2*i)+1]);
        msb -= isdigit(msb) ? 0x30 : 0x57;
        lsb -= isdigit(lsb) ? 0x30 : 0x57;

        *pStream = (lsb | (msb << 4));
        pStream++;
    }
} //stringToBytes()


//*******-------------------------------------------------------------
#if 0
/**
* @brief Funcao para converter um array de bytes em hexa para uma string
* contendo a representacao de bytes em ASCII.
*
* Esta funcao converte um array de bytes em hexa para uma string que contem
* a representacao de bytes em ASCII. E especialmente util para se enviar um
* array de bytes para a funcao que calcula o hash (MD5 ou SHA-256) que so
* trabalha com parametro em string.
* @param[in] strIn  vetor de bytes contendo o valor a ser convertido
* @return String contendo a representacao ASCII
*
************************************************************************/
std::string bytesToString(uint8_t *pStream, uint8_t len)
{
    uint8_t i = 0;
    std::string ret;
    for (uint8_t l = 0; l < len ; l++)
    {
        uint8_t msb = pStream[i] >> 4;
        uint8_t lsb = pStream[i++] & 0x0F;
        msb += (msb <= 0x09) ? 0x30 : 0x57;
        lsb += (lsb <= 0x09) ? 0x30 : 0x57;
        ret.push_back((char)msb);
        ret.push_back((char)lsb);
    }
    return ret;
} //bytesToString(
#endif
//*******-------------------------------------------------------------


/**
* @brief Funcao para piscar o LED interno
*
* Esta funcao pisca o LED interno de acordo com os parametros passados. Pode-se
* definir o numero de ciclos, e os tempos de ligado e desligado de cada ciclo.
* ATENCAO: usa delay para temporizar.
*
* @param[in] ciclos numero de ciclos de ligado / desligado
* @param[in] t_on tempo de LED ligado a cada ciclo (milissegundos)
* @param[in] t_off tempo de LED desligado a cada ciclo (milissegundos)
*
************************************************************************/
void piscaLed(uint8_t ciclos, uint16_t t_on, uint16_t t_off)
{

    for (uint16_t k = 0; k < ciclos; k++)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(t_on);
        digitalWrite(LED_BUILTIN, LOW);
        delay(t_off);
    }
} //piscaLed(

/**
* @brief Converte numero de segundos no dia para horas, minutos e segundos
*
* Esta funcao recebe um numero inteiro representando a quantidade de segundos
* a partir de 00h00m00s e retorna HH, MM e SS.
*
* @param[in] seg numero de segundos desde 00h00m00s
* @param[out] hh numero de horas
* @param[out] mm numero de minutos
* @param[out] ss numero de segundos
*
************************************************************************/
void segundosParaHHMMSS(uint32_t seg, uint8_t *hh, uint8_t *mm, uint8_t *ss)
{
    uint32_t tm = (seg / 60);
    *hh = tm / 60;
    *mm = tm % 60;
    *ss = seg % 60;

    return;
} //segundosParaHHMMSS(

/**
* @brief Converte um horario em HH:MM:SS para segundos a partir de 00h00m00
*
* Esta funcao recebe um horario no formato HH:MM:SS e retorna a quantidade de
* segundos a partir de 00h00m00s que este horario representa.
*
* @param[in] hh horas
* @param[in] mm minutos
* @param[in] ss segundos
* @return numero de segundos desde 00h00m00s
*
************************************************************************/
uint32_t hhmmssParaSegundos(uint8_t hh, uint8_t mm, uint8_t ss)
{
    return ((hh*3600)+(mm*60)+ss);
} //hhmmssParaSegundos(

/**
* @brief Funcao para definir o maior entre 3 numeros inteiros
*
* Esta funcao recebe 3 numeros inteiros (uint8) e retorna o maior deles
*
* @param[in] n1 primeiro numero
* @param[in] n2 segundo numero
* @param[in] n3 terceiro numero
* @return maior numero
*
************************************************************************/
uint8_t max3 (uint8_t n1, uint8_t n2, uint8_t n3)
{
    uint8_t max = n1 > n2 ? (n1 > n3 ? n1 : n3) : (n2 > n3 ? n2 : n3);
    return max;
} //max3 (
