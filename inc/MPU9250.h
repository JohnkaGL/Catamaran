#ifndef __MPU9250_H_
#define __MPU9250_H_
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

class MPU9250
{
    private: // Vars.
        i2c_inst_t *port;
        uint16_t sda;
        uint16_t scl;
    public:
        float getMagX_uT(); //Obtener el valor en x del magnetometro
        float getMagY_uT(); //Obtener el valor en y del magnetometro
        float getMagZ_uT(); //Obtener el valor en z del magnetometro
        uint8_t getMagX_H();
        uint8_t getMagX_L();
        uint8_t getMagY_H();
        uint8_t getMagY_L();
        uint8_t getMagZ_H();
        uint8_t getMagZ_L();
        void setMagCalX(float bias,float scaleFactor); //Dar los valores de calibración {offset, escala} del eje x
        void setMagCalY(float bias,float scaleFactor); //Dar los valores de calibración {offset, escala} del eje y
        void setMagCalZ(float bias,float scaleFactor); //Dar los valores de calibración {offset, escala} del eje z
        int readSensor(); // Leer los registros de resultados
        void init();
    protected:
        int writeAK8963Register(uint8_t subAddress, uint8_t data); //Escribir en registro del magnetómetro(AK8963)
        int readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest); //Leer registros del magnetómetro(AK8963)
        int enableDataReadyInterrupt();
        float _hx,_hy,_hz; // Resultado de medidas
        int16_t _hxcounts,_hycounts,_hzcounts,_axcounts,_aycounts,_azcounts,_gxcounts,_gycounts,_gzcounts; //Lectura de registros
        float _magScaleX,_magScaleY,_magScaleZ; //Escala magnetómetro
        float _hxb,_hyb,_hzb; //Offset
        float _hxs,_hys,_hzs; //escala secundaria
        uint8_t _buffer[21]; //buffer de salidas
    public:
        MPU9250(i2c_inst_t *p, uint16_t data, uint16_t clk); // Constructor
        ~MPU9250(); //Destructor


};


#endif