#include "MPU9250.h"
#include "mpu9250_library.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h" 

MPU9250::MPU9250(i2c_inst_t *p, uint16_t data, uint16_t clk)
{
    this->port=p;
    this->sda=data;
    this->scl=clk;
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(scl);//PICO_DEFAULT_I2C_SDA_PIN 
    //bi_decl(bi_2pins_with_func(sda, scl, GPIO_FUNC_I2C));
}

void MPU9250::init(){
    uint8_t buf00[2] = {PWR_MGMNT_1,0x00};
    i2c_write_blocking(i2c_default, 0x68, buf00,2, false);
    uint8_t buf0[2] = {PWR_MGMNT_1,CLOCK_SEL_PLL};
    i2c_write_blocking(i2c_default, 0x68, buf0,2, false);
    // enable I2C master mode
    uint8_t buf[2] = {USER_CTRL,I2C_MST_EN};
    i2c_write_blocking(i2c_default, 0x68, buf,2, false);
    // set the I2C bus speed to 400 kHz
    uint8_t buf2[2] = {I2C_MST_CTRL,I2C_MST_CLK};
    i2c_write_blocking(i2c_default, 0x68, buf2, 2, false);
    // set AK8963 to Power Down ¬¬
    writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN); //Se puede dejar así, se ha modificado abajo
    // reset the MPU9250
    uint8_t buf3[2]={PWR_MGMNT_1,PWR_RESET};
    i2c_write_blocking(i2c_default, 0x68, buf3, 2, false);
    // wait for MPU-9250 to come back up
    sleep_ms(1);
    // reset the AK8963 ¬¬
    writeAK8963Register(AK8963_CNTL2,AK8963_RESET);
    /* get the magnetometer calibration */
    sleep_ms(100);
    uint8_t buf10[2]={PWR_MGMNT_1,CLOCK_SEL_PLL};
    i2c_write_blocking(i2c_default, 0x68, buf10, 2, false);
    uint8_t buf11[2]={PWR_MGMNT_2,0x00};
    i2c_write_blocking(i2c_default, 0x68, buf11, 2, false);
    // accel full scale select
    uint8_t buf7[2]={ACCEL_CONFIG, 0x18};///16G
    i2c_write_blocking(i2c_default, 0x68, buf7, 2, false);
    // gyro full scale select
    uint8_t buf6[2]={GYRO_CONFIG, 0x18};//2000DPS
    i2c_write_blocking(i2c_default, 0x68, buf6, 2, false);
    // A_DLPFCFG
    uint8_t buf8[2]={ACCEL_CONFIG2, 0x01};//184
    i2c_write_blocking(i2c_default, 0x68, buf8, 2, false);
    //GYRO DLPFCFG
    uint8_t buf4[2]={ MPU_CONFIG, 0x01};//184
    i2c_write_blocking(i2c_default, 0x68, buf4, 2, false);
    // sample rate divider
    uint8_t buf5[2]={SMPLRT_DIV, 0x00};// muestras a la tasa de muestreo
    i2c_write_blocking(i2c_default, 0x68, buf5, 2, false);
    
    // enable I2C master mode
    i2c_write_blocking(i2c_default, 0x68, buf,2, false);
    // set the I2C bus speed to 400 kHz
    i2c_write_blocking(i2c_default, 0x68, buf2, 2, false);
    
    // set AK8963 to Power Down
    writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
    sleep_ms(100); // long wait between AK8963 mode changes
    // set AK8963 to FUSE ROM access
    writeAK8963Register(AK8963_CNTL1,AK8963_FUSE_ROM);
    sleep_ms(100); // long wait between AK8963 mode changes
    // read the AK8963 ASA registers and compute magnetometer scale factors
    readAK8963Registers(AK8963_ASA,3,_buffer);
    _magScaleX = ((((float)_buffer[0]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
    _magScaleY = ((((float)_buffer[1]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
    _magScaleZ = ((((float)_buffer[2]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla 
    // set AK8963 to Power Down
    writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
    sleep_ms(100); // long wait between AK8963 mode changes  
    // set AK8963 to 16 bit resolution, 100 Hz update rate
    writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2);
    sleep_ms(100); // long wait between AK8963 mode changes 
    i2c_write_blocking(i2c_default, 0x68, buf10, 2, false);      
    uint8_t b[3];
    readAK8963Registers(0x00,3,b);
    /*if(b[0] != 72){
        printf("\nLa lectura es: %d , %d ,%d",b[0],b[1],b[2]);
    }*/
    enableDataReadyInterrupt();
    readAK8963Registers(AK8963_HXL,7,_buffer);
    setMagCalX(0.0,1);
    setMagCalY(0.0,1);
    setMagCalZ(0.0,1);
}

MPU9250::~MPU9250()
{
}

float MPU9250::getMagX_uT(){
    return _hx;
}
float MPU9250::getMagY_uT(){
    return _hy;
}
float MPU9250::getMagZ_uT(){
    return _hz;
}
void MPU9250::setMagCalX(float bias,float scaleFactor){
    _hxb=bias;
    _hxs=scaleFactor;
}
void MPU9250::setMagCalY(float bias,float scaleFactor){
    _hyb=bias;
    _hys=scaleFactor;
}
void MPU9250::setMagCalZ(float bias,float scaleFactor){
    _hzb=bias;
    _hzs=scaleFactor;
}

int MPU9250::writeAK8963Register(uint8_t subAddress, uint8_t data){
    //Dispositivo a escribir
    uint8_t buf0[2] = {I2C_SLV0_ADDR,AK8963_I2C_ADDR};
    i2c_write_blocking(i2c_default, 0x68, buf0, 2, false);
    // Direccion de destino de la info
    uint8_t buf[2] = {I2C_SLV0_REG, subAddress};
    i2c_write_blocking(i2c_default, 0x68, buf, 2, false);
    //Informacion
    uint8_t buf1[2] = {I2C_SLV0_DO, data};
    i2c_write_blocking(i2c_default, 0x68, buf1, 2, false);
    //Transferencia de informacion
    uint8_t buf2[2] = {I2C_SLV0_CTRL, I2C_SLV0_EN};
    i2c_write_blocking(i2c_default, 0x68, buf2, 2, false);
    return 0;
}
int MPU9250::readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest){
    //Dispositivo a leer + Bandera de lectura
    uint8_t a=AK8963_I2C_ADDR|0x80;
    uint8_t buf0[2] = {I2C_SLV0_ADDR,a};
    i2c_write_blocking(i2c_default, 0x68, buf0, 2, false);
    //Direccion a leer
    uint8_t buf[2] = {I2C_SLV0_REG, subAddress};
    i2c_write_blocking(i2c_default, 0x68, buf, 2, false);
    //Trasferencia de informacion+tamaño
    a=I2C_SLV0_EN|(count&0xF);
    uint8_t buf2[2] = {I2C_SLV0_CTRL,a};
    i2c_write_blocking(i2c_default, 0x68, buf2, 2, false);
    sleep_ms(10);
    return 0;
}
int MPU9250::readSensor(){
/*
    uint8_t val=EXT_SENS_DATA_00;//ACCEL_XOUT_H
    i2c_write_blocking(i2c_default, 0x68, &val, 1, true);
    i2c_read_blocking(i2c_default, 0x68,_buffer, 7, false);
*/
    uint8_t val=ACCEL_XOUT_H;
    i2c_write_blocking(i2c_default, 0x68, &val, 1, true);
    i2c_read_blocking(i2c_default, 0x68,_buffer, 21, false);

    _axcounts = (((int16_t)_buffer[0]) << 8) | _buffer[1];  
    _aycounts = (((int16_t)_buffer[2]) << 8) | _buffer[3];
    _azcounts = (((int16_t)_buffer[4]) << 8) | _buffer[5];
    //_tcounts = (((int16_t)_buffer[6]) << 8) | _buffer[7];
    _gxcounts = (((int16_t)_buffer[8]) << 8) | _buffer[9];
    _gycounts = (((int16_t)_buffer[10]) << 8) | _buffer[11];
    _gzcounts = (((int16_t)_buffer[12]) << 8) | _buffer[13];
    _hxcounts = (((int16_t)_buffer[15]) << 8) | _buffer[14];
    _hycounts = (((int16_t)_buffer[17]) << 8) | _buffer[16];
    _hzcounts = (((int16_t)_buffer[19]) << 8) | _buffer[18];
/*
    _hxcounts = (((int16_t)_buffer[1]) << 8) | _buffer[0];
    _hycounts = (((int16_t)_buffer[3]) << 8) | _buffer[2];
    _hzcounts = (((int16_t)_buffer[5]) << 8) | _buffer[4];  
*/
    _hx = (((float)(_hxcounts) * _magScaleX) - _hxb)*_hxs;
    _hy = (((float)(_hycounts) * _magScaleY) - _hyb)*_hys;
    _hz = (((float)(_hzcounts) * _magScaleZ) - _hzb)*_hzs;
    return 0;
}
/* enables the data ready interrupt */
int MPU9250::enableDataReadyInterrupt() {
  /* setting the interrupt */
  uint8_t buf[2]={INT_PIN_CFG,0x00} ; // 50ms 
  i2c_write_blocking(i2c_default, 0x68,buf, 2,false);
  uint8_t buf1[2]={INT_ENABLE,0x01} ; //RAW_DATA_READY
  i2c_write_blocking(i2c_default, 0x68,buf1, 2,false);
  return 0;
}
uint8_t MPU9250::getMagX_H(){
    return ((_hxcounts&0xff00)>>8);
}
uint8_t MPU9250::getMagY_H(){
    return ((_hycounts&0xff00)>>8);
}
uint8_t MPU9250::getMagZ_H(){
    return ((_hzcounts&0xff00)>>8);
}
uint8_t MPU9250::getMagX_L(){
    return (_hxcounts&0x00ff);
}
uint8_t MPU9250::getMagY_L(){
    return (_hycounts&0x00ff);
}
uint8_t MPU9250::getMagZ_L(){
    return (_hzcounts&0x00ff);
}

