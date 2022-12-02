#ifndef _SERVO_H_
#define _SERVO_H_
#include "hardware/pwm.h"
#include "hardware/gpio.h"
class SERVO
{
private:// variables privadas
    uint16_t Pin;
    uint grados;
public:
    //Variables publicas
private:// funciones
    //Funciones privadas
public: //metodos
    void setSpeed(float new_speed);
    void degrees(uint deg);
public://constructor
    SERVO(uint16_t Pin,uint grados);
    ~SERVO();
};
#endif