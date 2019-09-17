#if !defined(__TEST_H__)
#define __TEST_H__

#include <stdint.h>

//#define TEST1CONFIG
//#define SEND_RECEIVE
//#define TEST_FTDI
//#define TEST_LORA_SERIALDEBUG
//#define OLD_TEST_FTDI

//#define TESTE_FIMAFIM
//#define TESTE_ABNT
//#define TEST_SEMRTOS
//#define TEST_SERIALNATIVA
//#define TESTE_RS232
//#define USB_DEBUG
//#define TESTE_SERIAL3
//#define TEST_CONSUMO
//#define TESTE_PWRFAIL
//#define TEST_LORA_CLASS_C

#define SEM_CHMASK

//#define SEMLORA
#define DEBUG_SERIAL3
#define DEBUG_LORA
#define TESTE_INTEGRACAO
//#define TESTE_JOIN
//#define TEST_TERMINAL_AT
//#define TEST_TIMER5
//#define TESTE_CRIPTO
//#define TESTE_MODELO
//#define TEST_INITXMIT

#define SECRET_APP_EUI "70B3D57ED0012777"
//#define SECRET_APP_KEY "AECBAEA6EB06A316413E7B2244DFBF00" //AppKey para servidor interno Multitech / Node-RED
//#define SECRET_APP_KEY "0e4c5961ba6e2b116fda7dd14382246f" //Appkey para servidor local, placa avulsa (EUI :7a0b), LoRaServer
//#define SECRET_APP_KEY "7818c354f78b690d30145963e87b2938" //Chave para o EUI: a8610a3432266f05 (servidor interno LoRaServer)
//#define SECRET_APP_KEY "27ca801eadbd2b30b52c64581b4b37a4" // chave para o EUI: a8610a3039476906 (placa com interface debugger ATMEL-ICE)
//#define SECRET_APP_KEY "c11b6c1d71a26eca4dbb1cf7e0722882" // chave para o EUI: a8610a303923750b (placa nao montada na placa mae)
//#define SECRET_APP_KEY "f4092d86e075ad28fc6c04e0973e8d4d"    //end device 0115
#define SECRET_APP_KEY "2ca6c21ec44c45984ee111534f5f355c"    //end device 0105

#define ABP_DEVADDR     "000f780f" //"0039561c"
#define ABP_NWKSKEY     "8105c97abfe43b24f378376f7a1b8132"  //"a7cfa33b3d57d5524eefa45582d69915"
#define ABP_APPSKEY     "91f5160e510f18500927b43883b96269"  //"3c005aa77471e1ea3376fa657c09dfe0"

#define MIN_LORA_INTERVAL       250000L //milissegundos
#define LORA_RESP_INTERVAL      1000
#define TEMPO_MORTO             5000
#define TMIN_SEM_CONECTALORA    20000

#define DEBUG_BAUD 9600

#define PIN_TESTE_PWRFAIL   PIN_A1
#define PIN_TEST_ISR        PIN_A2
#define PIN_LEITURA_VIN     3
#define PIN_POWER_FAILURE   7

#define MIN_ABNT_SEND_INTERVAL  (12000L)
#define T_COMANDO               (10000L)
#define T_WAITUSB               (5000L)
#define SLEEP_PERIOD            (5000)
#define T_MIN_MEDE_TEMP         (10000) //########(2000)
#define TMAX_SEM_ABNT           (9000000L)

#define TIMER_STSEMCOMUNIC      (1000)
#define TIMER_STNCOMISSIONADO   (200)
#define TIMER_STSEMACK          (150)
#define TIMER_STSEMLORA         (150)
#define TIMER_STSEMABNT         (150)
#define TIMER_STNORMAL          (150)

//Sinalizacao de instalacao com sucesso
#define NUM_INV_INTERMITENTE    8
#define NUM_INV_SEMACK          4
#define NUM_SEG_SINALIZANORMAL  5

//Quantidade de tentativas para Join e Init na rede LoRa
#define MAX_TENTATIVAS_JOIN     2
#define MAX_TENTATIVAS_INIT     30

typedef struct secret_keys
{
    uint8_t   comissionado;
    uint8_t   sessao_ok;
    uint32_t  seed;
    uint8_t   intervalo;
    uint32_t  numSerieMedidor;
    uint16_t  modeloMedidor;
    char      appeui[17];
    char      appkey[33];
    char      appskey[33];
    char      nwkskey[33];
    char      devaddr[9];
    uint8_t senhaABNT[10];
} __attribute__ ((packed)) secret_keys_t;

extern secret_keys_t localKeys;

#endif
