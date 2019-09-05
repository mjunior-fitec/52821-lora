/** @file tc_timer5.cpp
* @brief Modulo para configuracao e gerenciamento do timer interno para
* gerar interrupcoes temporizadas.
*
* Este modulo contem funcoes para configurar o timer5, permitindo a
* utilizacao para geracao de interrupcoes temporizadas com alta precisao.
*
* @date Jun/2019
* @author FITec - Fundacao para Inovacoes Tecnologicas
*         Marcio J Teixeira Jr. <mjunior@fitec.org.br>
*
************************************************************************/

#include <Arduino.h>
#include <stdint.h>

#include "tc_timer5.h"

//Prototipo da funcao da ISR
void maqEstSinalizacao(void);

//Prototipos das funcoes locais
bool tcIsSyncing(void);

//variaveis exportadas
static bool ledState = false;

//this function gets called by the interrupt at <sampleRate> Hertz
void TC5_Handler(void)
{
    maqEstSinalizacao();

    TC5->COUNT16.INTFLAG.bit.MC0 = 1; //Permite que nova interrupcao ocorra
}

//Configures the TC to generate output events at the sample frequency.
//Configures the TC in Frequency Generation mode, with an event output once
//each time the audio sample frequency period expires.
void tcConfigure(uint32_t sampleRate, tipoTimer_t tipo)
{
    // Enable GCLK for TCC2 and TC5 (timer counter input clock)
    GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
    while (GCLK->STATUS.bit.SYNCBUSY)
        ;

    tcReset(); //reset TC5

    // Set Timer counter Mode to 16 bits
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
    // Set TC5 mode as match frequency
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
    //set prescaler and enable TC5
    //////TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024 | TC_CTRLA_ENABLE;

    //set TC5 timer counter based off of the system clock and the user defined sample rate or waveform
    if (tipo == TIMER_HERTZ)
        TC5->COUNT16.CC[0].reg = (uint16_t)((SystemCoreClock / (sampleRate*1024)) - 1);
    else if (tipo == TIMER_MICROS)
        TC5->COUNT16.CC[0].reg = (uint16_t) (sampleRate * 0.046875);
    else if (tipo == TIMER_MILLIS)
        TC5->COUNT16.CC[0].reg = (uint16_t) (sampleRate * 46.875);

    while (tcIsSyncing())
        ;

    // Configure interrupt request
    NVIC_DisableIRQ(TC5_IRQn);
    NVIC_ClearPendingIRQ(TC5_IRQn);
    NVIC_SetPriority(TC5_IRQn, 0);
    NVIC_EnableIRQ(TC5_IRQn);

    // Enable the TC5 interrupt request
    TC5->COUNT16.INTENSET.bit.MC0 = 1;
    while (tcIsSyncing())
        ; //wait until TC5 is done syncing
}

//Function that is used to check if TC5 is done syncing
//returns true when it is done syncing
bool tcIsSyncing()
{
    return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

//Reset TC5
void tcReset()
{
    TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
    while (tcIsSyncing())
        ;
    while (TC5->COUNT16.CTRLA.bit.SWRST)
        ;
}

//This function enables TC5 and waits for it to be ready
void tcStartCounter()
{
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
    while (tcIsSyncing())
        ; //wait until snyc'd
}

//Stops TC5
void tcStopCounter()
{
    TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
    while (tcIsSyncing())
        ;
}
