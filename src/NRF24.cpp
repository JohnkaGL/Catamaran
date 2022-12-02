#include "NRF24.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include <string.h>
#include "NRF24L01_LIBRARY.h"

NRF24::NRF24(spi_inst_t *port, uint16_t csn, uint16_t ce)
{
    this->port = port; // Puerto de SPI
    this->csn = csn; // chip select
    this->ce = ce; //chip enable

    spi_init(this->port, 1000000);//puerto y velocidad
    /// INICIALIZACION DE PUERTO SP1 EN LOS GPIO 10-12

    gpio_set_function(10, GPIO_FUNC_SPI); // SCK
    gpio_set_function(11, GPIO_FUNC_SPI); // MOSI
    gpio_set_function(12, GPIO_FUNC_SPI); // MISO

    gpio_init(csn);//CHIP SELECT PIN avaliable
    gpio_init(ce);// CHIP ENABLE PIN avaliable

    gpio_set_dir(csn, 1);// CHIP SELECT AS OUTPUT
    gpio_set_dir(ce, 1); // CHIP ENABLE AS OUTPUT

    ceLow(); 
    csnHigh();

}

NRF24::~NRF24()
{
}

uint8_t NRF24::readReg(uint8_t reg){
    csnHigh();
    uint8_t result = 0;
    reg = ( R_REGISTERm & reg);
    csnLow();
    spi_write_blocking(port, &reg,1);
    spi_read_blocking(port, NOP,&result, 1);
    csnHigh();
    return result;
}

void NRF24::writeReg(uint8_t reg, uint8_t data){
    writeReg(reg, &data, 1);
}
void NRF24::writeReg(uint8_t reg, uint8_t *data, uint8_t size){
    reg = W_REGISTERm | ( R_REGISTERm & reg);
    csnLow();
    spi_write_blocking(port, &reg, 1);
    spi_write_blocking(port, (uint8_t*)data, size);
    csnHigh();
} // write registers on TX FIFO

void NRF24::config(){
    ceLow();
    sleep_ms(11);//wait 10.3ms power on reset
    writeReg(CONFIG, 0b00001010); // config. PWR_UP=1 ; EN_CRC=1 bit6=
    sleep_us(1500);//Start up 1.5ms
    //STAND By I
    writeReg(EN_AA,0); // no auto ack.
    writeReg(RF_CH,80); // channel. 
    writeReg(RX_ADDR_P0, (uint8_t*)"gyroc",5);//direccion de 5 bytes
    writeReg(TX_ADDR, (uint8_t*)"gyroc",5);//direccion de 5 bytes
    writeReg(RX_PW_P0, 32); // tamaño de las tramas en la linea0 = 32 bytes
    //gpio_put(25,0);
}
void NRF24::modeTX(){
    uint8_t reg = readReg(CONFIG);
    reg &= 0b11111110;// TODOS IGUALES MENOS EL BIT DE PRIM_RX=0
    writeReg(CONFIG, reg); 
    ceHigh();
    sleep_us(11);
    ceLow(); 
    sleep_us(130);//Wait 130us TX settling
}
void NRF24::modeRX(){
    uint8_t reg = readReg(CONFIG);
    reg |= 0b00000001;
    writeReg(CONFIG, reg);
    ceHigh(); 
    sleep_us(130);//wait 130us RX settling
}

void NRF24::sendMessage(char *msg){
    uint8_t reg = W_TX_PAYLOAD;
    csnLow();
    spi_write_blocking(port, &reg,1);
    spi_write_blocking(port, (uint8_t*)msg, 32);
    csnHigh();
    ceHigh();
    sleep_us(300);
    ceLow();
}
void NRF24::receiveMessage(char *msg){
    uint8_t cmd = R_RX_PAYLOAD;   
    csnLow();
    spi_write_blocking(port, &cmd, 1);
    spi_read_blocking(port,NOP, (uint8_t*)msg,32);
    csnHigh();

}

uint8_t NRF24::newMessage(){
    uint8_t fifo_status = readReg(FIFO_STATUS);
    return !(0b00000001 & fifo_status);
}

void NRF24::setChannel(uint8_t ch){
    writeReg(RF_CH, ch); // channel.
}

void NRF24::setRXName(char *name){
    if( strlen(name) != 5) return;
    writeReg(RX_ADDR_P0, (uint8_t*)name,5);
}
void NRF24::setRXName(char *name, uint8_t pipe){
    switch (pipe)
    {
    case 1:
        if( strlen(name) != 5) break;
        writeReg(RX_ADDR_P1, (uint8_t*)name,5);
        break;
    case 2:
        if( strlen(name) != 1) break;
        writeReg(RX_ADDR_P2, (uint8_t*)name,1);
        break;
    case 3:
        if( strlen(name) != 1) break;
        writeReg(RX_ADDR_P3, (uint8_t*)name,1);
        break;
    case 4:
        if( strlen(name) != 1) break;
        writeReg(RX_ADDR_P4, (uint8_t*)name,1);
        break;
    case 5:
        if( strlen(name) != 1) break;
        writeReg(RX_ADDR_P5, (uint8_t*)name,1);
        break;
    default:
        if( strlen(name) != 5) break;
        writeReg(RX_ADDR_P0, (uint8_t*)name,5);
        break;
    }
}

void NRF24::setTXName(char *name){
    if( strlen(name) != 5) return;
    writeReg(TX_ADDR, (uint8_t*)name,5);
}
void NRF24::flushTX(){
    uint8_t reg = (FLUSH_TX);//Cleaning TX FIFO
    csnLow();
    spi_write_blocking(port, &reg,1);
    csnHigh();
}
void NRF24::flushRX(){
    uint8_t reg = (FLUSH_RX);//Cleaning RX FIFO
    csnLow();
    spi_write_blocking(port, &reg,1);
    csnHigh();
}
void NRF24::clearIRQ(){
    uint8_t status = readReg(STATUS);
    writeReg(STATUS,0x40|status);
}