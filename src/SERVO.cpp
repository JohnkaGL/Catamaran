#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "SERVO.h"

SERVO::SERVO(uint16_t Pin,uint grados=0)
{
    this->Pin = Pin;
    this->grados = grados;
    gpio_set_function(Pin,GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(Pin);
    pwm_config config= pwm_get_default_config();
    pwm_config_set_clkdiv(&config,64.f);//divisor de reloj
    pwm_config_set_wrap(&config,39062.f);
    pwm_init(slice_num,&config,true);
    degrees(grados);
}
SERVO::~SERVO()
{
}
void SERVO::setSpeed(float new_speed){
    if (new_speed>2400) new_speed=2400;// el rango no debe sobre pasar el mayor
    if (new_speed<400) new_speed=400; // ni debe ser inferior al menor
    pwm_set_gpio_level(Pin,(new_speed/20000.f)*39062.f); /* el ciclo es de 20000us==20ms y 
    el divisor de reloj es la cantidad de ciclos de 125MHz necesarios para tener un periodo de 20ms=50Hz
    (125Mhz/64*50Hz)=39062.5*/
}
void SERVO::degrees(uint deg){
    float ns=(float)(deg)*(2000.f/180.f)+400.f;//2400 es el angulo mayor(180°), 400 el menor(0°)
    setSpeed(ns);// 
}