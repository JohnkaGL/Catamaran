/*! 
 *  \brief     Libreria NRF-MEMORY MAP
 *  \details   Distribución de memoria NRF224L01
 *  \author    John Camilo Giraldo López
 *  \version   1.0
 *  \date      Agust 1st 2022
 *  \copyright Universidad de Antioquia, Facultad de Ingeniería, Departamento de ingeniería electrónica y de telecomunicaciones
 */
#ifndef _NRF24L01_LIBRARY
#define _NRF24L01_LIBRARY

/*******************************************************************/
/****************Start Commands Enumerate******************/
/*******************************************************************/
enum{
    R_REGISTERm=0b00011111, /// mask for reading a FIFO register 
    W_REGISTERm=0b00100000, /// mask for writing a FIFO register 
    R_RX_PAYLOAD=0b01100001, /// command to read all FIFO
    W_TX_PAYLOAD=0b10100000, /// command to write all FIFO
    FLUSH_TX=0b11100001, /// clears the FIFO used
    FLUSH_RX=0b11100010, /// clears the FIFO used
    REUSE_TX_PL=0b11100011, /// stand last transmited data
    ACTIVATE=0b01010000, /// activate functions RX_PL_WID,TX_PAYLOADm,TX_PAYLOAD_NO_ACK if followed by data 0x73
    R_RX_PL_WIDa=0b01100000, /// command to measure RX FIFO registers used
    W_TX_PAYLOADm=0b10101111, /// command to
    W_TX_PAYLOAD_NO_ACKa=0b10110000, /// 
    NOP=0xFF /// no operation
};//listado de comandos SPI
/*******************************************************************/
/****************End Commands Enumerate******************/
/*******************************************************************/

/*******************************************************************/
/****************Start Bits Fields Type Definition******************/
/*******************************************************************/
/*
typedef union{
    uint8_t W;
    typedef struct{
        uint8_t PRIM_RX:1;
        uint8_t PWR_UP:1;
        uint8_t CRCO:1;
        uint8_t EN_CRC:1;
        uint8_t MASK_MAX_RT:1;
        uint8_t MASK_TX_DS:1;
        uint8_t MASK_RX_DR:1;
        uint8_t reserved:1;
    }B;
}_NRF24L01_CONFIG_t;
typedef union{
    uint8_t W;
    typedef struct{
        uint8_t ENAA_P0:1;
        uint8_t ENAA_P1:1;
        uint8_t ENAA_P2:1;
        uint8_t ENAA_P3:1;
        uint8_t ENAA_P4:1;
        uint8_t ENAA_P5:1;
        uint8_t reserved:2;
    }B;
}_NRF24L01_EN_AA_t;
typedef union{
    uint8_t W;
    typedef struct{
        uint8_t ERX_P0:1;
        uint8_t ERX_P1:1;
        uint8_t ERX_P2:1;
        uint8_t ERX_P3:1;
        uint8_t ERX_P4:1;
        uint8_t ERX_P5:1;
        uint8_t reserved:2;
    }B;
}_NRF24L01_EN_RXADDR_t;
typedef union{
    uint8_t W;
    typedef struct{
        uint8_t AW:2;
        uint8_t reserved:6;
    }B;
}_NRF24L01_SETUP_AW_t;
typedef union{
    uint8_t W;
    typedef struct{
        uint8_t ARC:4;
        uint8_t ARD:4;
    }B;
}_NRF24L01_SETUP_RETR_t;
typedef union{
    uint8_t W;
    typedef struct{
        uint8_t RF_CH:6;
        uint8_t reserved:1;
    }B;
}_NRF24L01_RF_CH_t;
typedef union{
    uint8_t W;
    typedef struct{
        uint8_t LNA_HCURR:1;
        uint8_t RF_PWR:2;
        uint8_t RF_DR:1;
        uint8_t PLL_LOCK:1;
        uint8_t reserved:3;
    }B;
}_NRF24L01_RF_SETUP_t;
typedef union{
    uint8_t W;
    typedef struct{
        uint8_t TX_FULL:1;
        uint8_t RX_P_NO:3;
        uint8_t MAX_RT:1;
        uint8_t TX_DS:1;
        uint8_t RX_DR:1;
        uint8_t reserved:1;
    }B;
}_NRF24L01_STATUS_t;
typedef union{
    uint8_t W;
    typedef struct{
        uint8_t ARC_CNT:4;
        uint8_t PLOS_CNT:4;
    }B;
}_NRF24L01_OBSERVE_TX_t;
typedef union{
    uint8_t W;
    typedef struct{
        uint8_t CD:1;
        uint8_t Reserved:7;
    }B;
}_NRF24L01_CD_t;
typedef struct{
    uint RX_ADDR_P0:40;
}_NRF24L01_RX_ADDR_P0_t;
typedef struct{
    uint RX_ADDR_P1:40;
}_NRF24L01_RX_ADDR_P1_t;

typedef struct{
    uint8_t RX_ADDR_Pn:8;
}_NRF24L01_RX_ADDR_Pn_t;
typedef struct{
    uint TX_ADDR:40;
}_NRF24L01_TX_ADDR_t;
typedef struct{
    uint TX_ADDR:40;
}_NRF24L01_TX_ADDR_t;
typedef union{
    uint8_t W;
    typedef struct{
        uint8_t RX_PW_Pn:6;
        uint8_t Reserved:2;
    }B;
}_NRF24L01_RX_PW_Pn_t;
typedef union{
    uint8_t W;
    typedef struct{
        uint8_t RX_EMPTY:1;
        uint8_t RX_FULL:1;
        uint8_t Reserved:2;
        uint8_t TX_EMPTY:1;
        uint8_t TX_FULL:1;
        uint8_t TX_REUSE:1;
        uint8_t Reserved2:1;
    }B;
}_NRF24L01_FIFO_STATUS_t;
typedef struct{
    _NRF24L01_CONFIG_t CONFIG;
    _NRF24L01_EN_AA_t EN_AA;
    _NRF24L01_EN_RXADDR_t EN_RXADDR;
    _NRF24L01_SETUP_AW_t SETUP_AW;
    _NRF24L01_SETUP_RETR_t SETUP_RETR;
    _NRF24L01_RF_CH_t RF_CH;
    _NRF24L01_RF_SETUP_t RF_SETUP;
    _NRF24L01_STATUS_t STATUS;
    _NRF24L01_OBSERVE_TX_t OBSERVE_TX;
    _NRF24L01_CD_t CD;
    _NRF24L01_RX_ADDR_P0_t RX_ADDR_P0;
    _NRF24L01_RX_ADDR_P1_t RX_ADDR_P1;
    _NRF24L01_RX_ADDR_Pn_t RX_ADDR_P2;
    _NRF24L01_RX_ADDR_Pn_t RX_ADDR_P3;
    _NRF24L01_RX_ADDR_Pn_t RX_ADDR_P4;
    _NRF24L01_RX_ADDR_Pn_t RX_ADDR_P5;
    _NRF24L01_TX_ADDR_t TX_ADDR;
    _NRF24L01_RX_PW_Pn_t RX_PW_P0;
    _NRF24L01_RX_PW_Pn_t RX_PW_P1;
    _NRF24L01_RX_PW_Pn_t RX_PW_P2;
    _NRF24L01_RX_PW_Pn_t RX_PW_P3;
    _NRF24L01_RX_PW_Pn_t RX_PW_P4;
    _NRF24L01_RX_PW_Pn_t RX_PW_P5;
    _NRF24L01_FIFO_STATUS_t FIFO_STATUS;
}_NRF24L01_t;
*/
///*******************************************************************/
///****************Start MACROs per Register**************************/
///*******************************************************************/
// #define sNRF24L01 (*((_NRF24L01_t *)(0x00)))                     /// Apuntador a Registro base de NRF24L01
// Esto significa que cualquier variable de este tipo (estructura) 
// que se declare de este tipo en un código aparte, será apuntada 
// al registro base 0x00? O que la estructura se fijará en este registro para apuntarse?
/// R/ Las direcciones que genera el compilador son apuntadores a bytes,
/// por tanto para tener las direcciones reales no es util tener la distribución de memoria

enum{
    CONFIG,
    EN_AA,
    EN_RXADDR,
    SETUP_AW, 
    SETUP_RETR,
    RF_CH,
    RF_SETUP,
    STATUS,
    OBSERVE_TX,
    CD,
    RX_ADDR_P0,
    RX_ADDR_P1,
    RX_ADDR_P2,
    RX_ADDR_P3,
    RX_ADDR_P4,
    RX_ADDR_P5,
    TX_ADDR,
    RX_PW_P0,
    RX_PW_P1,
    RX_PW_P2,
    RX_PW_P3,
    RX_PW_P4,
    RX_PW_P5,
    FIFO_STATUS,
    DYNPDc=0x1C,
    FEATUREc=0x1D
};
/*******************************************************************/
/****************End MACROs per Register****************************/
/*******************************************************************/
#endif