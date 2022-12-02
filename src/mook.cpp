#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "NRF24.h"
#include "NRF24L01_LIBRARY.h"
#include "sd_card.h"
#include "ff.h"
using namespace std;

/// GPIO AND PORTS
#define Vela_Izquierda 2
#define Vela_Derecha 3
#define automatic 4
#define LED_PIN 25
#define ADC_S 0

//#define fileClose 5

#define MAXDIR 3556.0f
#define ScaleFx 1.0f
#define ScaleFy 1.0f
#define ScaleFz 1.0f
#define biasX 0.0f
#define biasY 0.0f
#define biasZ 0.0f

/*Connections on Raspberry Pi Pico board, other boards may vary.

   GND   (pin 3,8,13,18,23,28,33,38)-> GND (general)
   GPIO 2 (pin 4)-> Vela Izquierda
   GPIO 3 (pin 5)-> Vela Derecha
   GPIO 4 (pin 6)-> N.A.
   
   GPIO 8 (pin 11)-> CE(NRF)
   GPIO 9 (pin 12)-> CSn(NRF)
   GPIO 10 (pin 14)-> SCK(NRF)
   GPIO 11 (pin 15)-> MOSI(NRF)
   GPIO 12 (pin 16)-> MISO(NRF)

   GPIO 16 (pin 21)-> MISO(SD)
   GPIO 17 (pin 22)-> CSn(SD)
   GPIO 18 (pin 24)-> SCK(SD)
   GPIO 19 (pin 25)-> MOSI(SD)

   GPIO 26 (pin 31)-> JOYSTICK

   3.3v (pin 36) -> VCC (Or Logic Converter)
   5v (pin 40)-> VCC USB
*/
NRF24 nrf(spi1, 9, 8);// inicializacion de objeto de transmisor GLOBAL
char bufferout[32] {0};//buffers
char bufferin[32] {0};
bool flag=0;// bandera de interrupciones GPIO
char filebuf[250];///
FRESULT fr;
FATFS fs;
FIL fil;
int ret;
char filename[] = "Info.txt";

int contador(){
    static uint count_1=0;
    static uint c2_1=0;
    count_1++;
    if(count_1==15000){
        c2_1++;
        //printf("Boton_1 %d veces!\n",c2_1);
        count_1=0;
        return 1;
    }else{
        return 0;
    }
}
int contador2(){
    static uint count_3=0;
    static uint c2_3=0;
    count_3++;
    if(count_3==15000){
        c2_3++;
        //printf("Boton_2 %d veces! \n",c2);
        count_3=0;
        return 1;
    }else{
        return 0;
    }
}
int contador3(){
    static uint count=0;
    static uint c2=0;
    count++;
    if(count==15000){
        c2++;
        //printf("Boton_2 %d veces! \n",c2);
        count=0;
        return 1;
    }else{
        return 0;
    }
}
void gpio_callback(uint gpio, uint32_t events) {
    uint8_t abajo=0;
    uint8_t abajo2=0;
    if(gpio==Vela_Derecha){
        if(contador())abajo=1;
        if(abajo){
            for(int i=0;i<1200;i++){
                //x=0; //Gasto de ciclos para evitar leer el estado de GPIO
            }
            abajo=0;
            bufferout[2]='C';
            flag=1;
        }
    }
    if(gpio==Vela_Izquierda){
        if(contador2())abajo2=1;
        if(abajo2){
            for(int i=0;i<1200;i++){
                //x=0; //Gasto de ciclos para evitar leer el estado de GPIO
            }
            abajo2=0;
            bufferout[3]='B';
            flag=1;
        }
    }
}
void ControlInit(){
    gpio_init(LED_PIN);
    gpio_init(Vela_Derecha);
    gpio_init(Vela_Izquierda);
    //gpio_init(automatic);
    //gpio_init(fileClose);
    stdio_init_all();
    gpio_set_dir(LED_PIN,1);//Definir como salida
    gpio_set_dir(Vela_Derecha,0); //Definir como entrada
    gpio_set_dir(Vela_Izquierda,0); 
    gpio_set_dir(automatic,0);
    //gpio_set_dir(fileClose,0);
    gpio_pull_up(Vela_Izquierda);
    gpio_pull_up(Vela_Derecha);
    //gpio_pull_up(automatic);
    //gpio_pull_up(fileClose);
    adc_init();    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26+ADC_S);    // Select ADC input 0 (GPIO26)
    adc_select_input(ADC_S);
    nrf.config();
    nrf.modeRX();
    if (!sd_init_driver()) {
        printf("ERROR: Could not initialize SD card\r\n");
        while (true);
    }
    // Mount drive
    fr = f_mount(&fs, "0:", 1);
    if (fr != FR_OK) {
        printf("ERROR: Could not mount filesystem (%d)\r\n", fr);
        while (true);
    }    
    gpio_set_irq_enabled_with_callback(Vela_Izquierda,GPIO_IRQ_LEVEL_LOW, true, &gpio_callback);// GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL
    while(!stdio_usb_connected());
    gpio_put(LED_PIN,1);
}

void Telemetry(){
    int ret2=0;
    float dir;
    float vel;
    float Magx_uT,Magy_uT,Magz_uT;
    nrf.receiveMessage(bufferin);
    //Procesar datos recibidos
    dir=(float)((bufferin[0]<<8)|bufferin[1])*360.0f/(MAXDIR);
    vel=(float)(bufferin[2])/10.0f;
    Magx_uT=((float)((int16_t)((bufferin[3]<<8)|bufferin[4]))*ScaleFx)-biasX; 
    Magy_uT=((float)((int16_t)((bufferin[5]<<8)|bufferin[6]))*ScaleFy)-biasY;
    Magz_uT=((float)((int16_t)((bufferin[7]<<8)|bufferin[8]))*ScaleFz)-biasZ;
    ret2=sprintf(filebuf,"%.2f,%.2f,%.2f,%.2f,%.2f,%c.%c%c%c",dir,vel,Magx_uT,Magy_uT,Magz_uT, bufferin[9], bufferin[10], bufferin[11], bufferin[12], bufferin[13]);
    printf(filebuf);
    //printf("%.2f,%.2f,%.2f,%.2f,%.2f,%c.%c%c%c%c%c%c%c,%c%c.%c%c%c%c%c%c%c\r\n",dir,vel,Magx_uT,Magy_uT,Magz_uT, bufferin[9], bufferin[10], bufferin[11], bufferin[12], bufferin[13], bufferin[14], bufferin[15], bufferin[16], bufferin[17], bufferin[18], bufferin[19], bufferin[20], bufferin[21], bufferin[22], bufferin[23], bufferin[24], bufferin[25], bufferin[26]);
    //Escribir sobre archivo
    ret = f_printf(&fil,filebuf);
    while (ret < 0) {
        printf("ERROR: Could not write to file (%d)\r\n", ret);
        ret = f_printf(&fil,filebuf);
    }
    bzero(filebuf,250);
    ///Aqui se partió la escritura
    ret2=sprintf(filebuf,"%c%c%c%c,%c%c.%c%c%c%c%c%c%c,%i,%i\n", bufferin[14], bufferin[15], bufferin[16], bufferin[17], bufferin[18], bufferin[19], bufferin[20], bufferin[21], bufferin[22], bufferin[23], bufferin[24], bufferin[25], bufferin[26],(int)bufferin[27],(int)bufferin[28]);
    printf(filebuf);
    //printf("%.2f,%.2f,%.2f,%.2f,%.2f,%c.%c%c%c%c%c%c%c,%c%c.%c%c%c%c%c%c%c\r\n",dir,vel,Magx_uT,Magy_uT,Magz_uT, bufferin[9], bufferin[10], bufferin[11], bufferin[12], bufferin[13], bufferin[14], bufferin[15], bufferin[16], bufferin[17], bufferin[18], bufferin[19], bufferin[20], bufferin[21], bufferin[22], bufferin[23], bufferin[24], bufferin[25], bufferin[26]);
    //Escribir sobre archivo
    while (ret < 0) {
        printf("ERROR: Could not write to file (%d)\r\n", ret);
        ret = f_printf(&fil,filebuf);
    }
    bzero(filebuf,250);
}

int main(){
    ControlInit();
    uint16_t timon;
    uint16_t temp;//Variable para almacenar temporalmente
    //gpio_set_irq_enabled_with_callback(automatic,GPIO_IRQ_LEVEL_LOW, true, &gpio_callback);
    while(1){
        tight_loop_contents();
        timon = adc_read();
        if(flag||temp!=timon){//si hay una interrupcion de botones o hay una diferencia en el adc
            nrf.modeTX();
            flag=0;
            bufferout[0]='M';
            bufferout[4]=(char)((int)(timon*0.043956)); //       180/4095=0.043956044
            temp=timon;
            nrf.sendMessage(bufferout);
            bzero(bufferout,32);//limpiar bufferout
            nrf.modeRX(); //la mayoria del tiempo en RX solo cuando haya eventos TX
        }
        if(nrf.newMessage()){
            Telemetry();
        }
    }
    // Unmount drive
    f_unmount("0:");
    return 0;  
    
}

