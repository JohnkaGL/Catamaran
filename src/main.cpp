#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hardware/uart.h"
#include "pico/binary_info.h"
#include "event_groups.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
//#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "NRF24.h"
#include "NRF24L01_LIBRARY.h"
#include "SERVO.h" 
#include "MPU9250.h"
#include "mpu9250_library.h"

//Timers del FreeRTOS
#include "timers.h"

//Grupo de eventos
#define BIT_0 (1UL << 0UL)
#define BIT_1 (1UL << 1UL)
#define BIT_2 (1UL << 2UL)
#define BIT_3 (1UL << 3UL)
#define BIT_4 (1UL << 4UL)

#define CONTROLTRIGGER (1UL << 3UL)|(1 << 2UL)|(1 << 1UL)|(1 << 0UL)
#define LED_PIN 25
#define SERVO1_PIN 2 //timon
#define SERVO2_PIN 0 //vela

//Wind
#define HOLE_PIN 18

//GPS
#define UART_ID uart0
#define BAUD_RATE 9600
#define DATA_BITS 8
#define STOP_BITS 1
#define UART_TX_PIN 16
#define UART_RX_PIN 17
#define PARITY    UART_PARITY_NONE

//IMU
#define SDA 4
#define SCL 5

//Variables
int dir=0;
int cont=0;
int cont2=0;
long long time=0;
int label1=0;
float v=0;
float v_mean=0;
float v_total=0;
char GPS1[100]={0};
char OUT[32]={0};
int i=0;
int m=0;
bool GPSOK=0;
//Perifericos

NRF24 nrf(spi1, 9, 8);// inicializacion de objeto de transmisor
MPU9250 IMU(i2c_default,4,5);//IMU object
SERVO servo1(SERVO1_PIN,90);//Servo Vela
SERVO servo2(SERVO2_PIN,90);//Servo timon

//uint8_t Globalbuffer[32];
char bufferin[32] {0};

void on_uart_rx() {
    char ch;
    ch = uart_getc(UART_ID);
    if(ch=='\n'){
        GPSOK=1;
    }else{
        GPS1[m]=ch;
        m++;
    }
}
void hole(uint gpio,uint32_t events)
{
    cont++;
    if(cont==1 && label1==1)
    {
        time=time_us_64();
        label1=0;
    }
    if(cont>=20)
    {
        cont2++;//contador de vueltas
        time=time_us_64()-time;
        float t=(float)time;
        t=t*0.000001;//Transformacion de unidades temporales u segundos a segundos
        v=(2*3.14159265*0.04)/t;//2*pi*r/t=theta/t
        v=v*3.6; //Factor de forma y posicion de cucharas
        v_mean=v_mean+v;
        cont=0;
        label1=1;
        if(cont2==2){
            v_total=v_mean/2;
            cont2=0;
            v=0;
            v_mean=0;
        }
    }
}
// funciones de inicialización
void UARTinit(){
    // Set up our UART with a basic baud rate.
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    int __unused actual = uart_set_baudrate(UART_ID, BAUD_RATE);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_ID, false);
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);
    //uart_puts(UART_ID, "\nHello, uart interrupts\n");
}
void InitHardware(){
    stdio_init_all();
    adc_init();
    adc_gpio_init(26+0);
    adc_select_input(0);
    gpio_init(HOLE_PIN);
    gpio_set_dir(HOLE_PIN, GPIO_IN);
    gpio_pull_down(HOLE_PIN);
    gpio_set_irq_enabled_with_callback(HOLE_PIN,0x04,1,hole);
    //gpio_init(LED_PIN);
    //gpio_set_dir(LED_PIN,1);//Definir como salida
    nrf.config();
    nrf.modeRX();//     NRF
    UARTinit(); //      ESP/GPS
    IMU.init(); //      IMU
    //while(!stdio_usb_connected());
}
//Event group de medida
EventGroupHandle_t xMeasureEventGroup;
EventGroupHandle_t xProcEventGroup;
EventGroupHandle_t xControlEventGroup;
EventGroupHandle_t xReceivingEventGroup;
// Tarea de revision de buffer de datos
void messageTask(void *pvParameters);
//Tarea de recepcion de ordenes
//void RxTask(void *pvParameters);
//Tareas de loop de entrega
void readIMUTask(void *pvParameters);//Leer magnetometro, giroscopio y acelerometro
void readWindDirTask(void *pvParameters);//Leer direccion del viento
void readWindSpeedTask(void *pvParameters);//Velocidad del viento
void readWiFi(void *pvParameters);

void processIMUTask(void *pvParameters);
void processWindDirTask(void *pvParameters);
void processWiFiTask(void *pvParameters);

void controlActionTask(void *pvParameters);
void sendPayloadTask(void *pvParameters);
//Tarea
void settingTask(void *pvParameters);

int main()
{
    InitHardware();
    printf("Iniciando...\r\n");

    //Creación del grupo de eventos
    printf("Creando grupos de eventos...\r\n");
    xMeasureEventGroup = xEventGroupCreate(); //Grupo de eventos de medida
    xProcEventGroup = xEventGroupCreate(); //Grupo de eventos de procesamiento 
    xControlEventGroup = xEventGroupCreate();//Grupo de eventos de control
    xReceivingEventGroup = xEventGroupCreate();//Grupo de eventos de recepcion

    //Creación de tareas
    printf("Creando tareas...\r\n");

    //Tarea trigger (Cada segundo)
    xTaskCreate(settingTask, "BitSetter", 1000, NULL, 1, NULL);
    xTaskCreate(messageTask, "BitSetter", 1000, NULL, 1, NULL);

    //Tareas de medición
    xTaskCreate(readIMUTask, "IMUReader", 1000, NULL, 2, NULL);
    xTaskCreate(readWindDirTask, "WindDirReader", 1000, NULL, 2, NULL);
    xTaskCreate(readWindSpeedTask, "WindSpeedReader", 1000, NULL, 2, NULL);
    xTaskCreate(readWiFi, "WiFiReader", 1000, NULL, 2, NULL);

    //xTaskCreate(RxTask, "ControlRead", 1000, NULL, 1, NULL);

    //Tareas de procesamiento
    xTaskCreate(processIMUTask, "IMUProces", 1000, NULL, 2, NULL);
    xTaskCreate(processWindDirTask, "WindProces", 1000, NULL, 2, NULL);
    xTaskCreate(processWiFiTask, "WiFiProces", 1000, NULL, 2, NULL);
    
    //Tarea de control
    xTaskCreate(controlActionTask, "ControlAction", 1000, NULL, 2, NULL);

    //Tarea de comunicacion
    xTaskCreate(sendPayloadTask, "Send", 1000, NULL, 2, NULL);

    //Iniciar el scheduler
    printf("Iniciando scheduler...\r\n");
    vTaskStartScheduler();

    for(;;);
    return 0;
}
void messageTask(void *pvParamters){
    const TickType_t xDelay1sec = pdMS_TO_TICKS(200UL), xDontBlock = 0;
    while(true){
        vTaskDelay(xDelay1sec);
        if (nrf.newMessage()==1){
            nrf.receiveMessage(bufferin);
            servo2.degrees((int) (bufferin[4]+3));//Servo Analogo, servo Timón
            if(bufferin[3]=='B'){
                if(i<=178){
                    i=i+2;
                    servo1.degrees(i);
                }
            }
            else if(bufferin[2]=='C'){
                if(i>=2){
                    i=i-2;
                    servo1.degrees(i);
                }
            }
            xEventGroupSetBits(xControlEventGroup, BIT_4);
            //xEventGroupSetBits(xReceivingEventGroup, BIT_0);
        }
    }
}
/*void RxTask(void *pvParameters){
    EventBits_t xEventGroupValue;
    const EventBits_t xBitsToWaitFor = BIT_0;
    while(true){
        xEventGroupValue = xEventGroupWaitBits(xReceivingEventGroup, xBitsToWaitFor, pdTRUE, pdTRUE, portMAX_DELAY);
    }
}*/
void settingTask(void *pvParamters){
    const TickType_t xDelay1sec = pdMS_TO_TICKS(4000UL), xDontBlock = 0;
    while(true){
        vTaskDelay(xDelay1sec);

        printf("---- < LANZANDO ETAPA DE MEDIDA > ----\r\n");

        //Disparando grupos de eventos.
        xEventGroupSetBits(xMeasureEventGroup, BIT_0);
        xEventGroupSetBits(xMeasureEventGroup, BIT_1);
        xEventGroupSetBits(xMeasureEventGroup, BIT_2);
        xEventGroupSetBits(xMeasureEventGroup, BIT_3);
    }
}

void readIMUTask(void *pvParameters){
    //Valor del grupo de eventos
    EventBits_t xEventGroupValue;
    //Bits del grupo de eventos por los que se va a esperar
    const EventBits_t xBitsToWaitFor = BIT_0;
    while(true){
        xEventGroupValue = xEventGroupWaitBits(xMeasureEventGroup, xBitsToWaitFor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("Iniciando lectura de IMU...\r\n");
        IMU.readSensor();
        xEventGroupSetBits(xProcEventGroup, BIT_0);
    }
}

void readWindDirTask(void *pvParameters){
    EventBits_t xEventGroupValue;
    const EventBits_t xBitsToWaitFor = BIT_1;

    while(true){
        xEventGroupValue = xEventGroupWaitBits(xMeasureEventGroup, xBitsToWaitFor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("Iniciando lectura de direccion del viento...\r\n");
        dir=adc_read();
        xEventGroupSetBits(xProcEventGroup, BIT_1);
    }
}

void readWindSpeedTask(void *pvParameters){
    EventBits_t xEventGroupValue;

    const EventBits_t xBitsToWaitFor = BIT_2;

    while(true){
        xEventGroupValue = xEventGroupWaitBits(xMeasureEventGroup, xBitsToWaitFor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("Iniciando lectura de velocidad del viento...\r\n");
        OUT[2]= (uint8_t) v_total*10;//Ajuste para reducir bits de envio
        xEventGroupSetBits(xControlEventGroup, BIT_2);
    }
}

void readWiFi(void *pvParameters){
    EventBits_t xEventGroupValue;

    const EventBits_t xBitsToWaitFor = BIT_3;

    while(true){
        xEventGroupValue = xEventGroupWaitBits(xMeasureEventGroup, xBitsToWaitFor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("Iniciando lectura de módulos WiFi...\r\n");
        if(GPSOK){
            xEventGroupSetBits(xProcEventGroup, BIT_2);
            GPSOK=0;
        }
    }
}

void processIMUTask(void *pvParameters){
    //Valor del grupo de eventos
    EventBits_t xEventGroupValue;

    //Bits del grupod e eventos por lo que se va a esperar
    const EventBits_t xBitsToWaitfor = BIT_0;

    while(true){
        xEventGroupValue = xEventGroupWaitBits(xProcEventGroup, xBitsToWaitfor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("Iniciando procesamiento de la IMU...\r\n");
        OUT[3]=IMU.getMagX_H();
        OUT[4]=IMU.getMagX_L();
        OUT[5]=IMU.getMagY_H();
        OUT[6]=IMU.getMagY_L();
        OUT[7]=IMU.getMagZ_H();
        OUT[8]=IMU.getMagZ_L();
        xEventGroupSetBits(xControlEventGroup, BIT_0);
    }
}

void processWindDirTask(void *pvParameters){
    //Valor del grupo de eventos
    EventBits_t xEventGroupValue;

    //Bits del grupod e eventos por lo que se va a esperar
    const EventBits_t xBitsToWaitfor = BIT_1;

    while(true){
        xEventGroupValue = xEventGroupWaitBits(xProcEventGroup, xBitsToWaitfor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("Iniciando procesamiento de la dirección del viento...\r\n");
        OUT[0]=(uint8_t)((dir&0xff00)>>8);
        OUT[1]=(uint8_t)((dir&0x00ff));
        xEventGroupSetBits(xControlEventGroup, BIT_1);
    }
}

void processWiFiTask(void *pvParameters){
    //Valor del grupo de eventos
    EventBits_t xEventGroupValue;

    //Bits del grupo de eventos por lo que se va a esperar
    const EventBits_t xBitsToWaitfor = BIT_2;

    while(true){
        xEventGroupValue = xEventGroupWaitBits(xProcEventGroup, xBitsToWaitfor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("Iniciando procesamiento del WiFi...\r\n");
        if(GPS1[0]=='$' && GPS1[1]=='G' && GPS1[2]=='P' && GPS1[3]=='R' && GPS1[4]=='M' && GPS1[5]=='C'){
            int l=0;
            GPSOK=0;//si se identicica una trama de GPS aun no está listo
            for(int r=6;r<m;r++){
                if(GPS1[r]==','){
                    l++;
                    if (l==3){//Confirmar posición de la primera coma de interés (Latitud)
                        if(GPS1[r+1]==','){    //No se engancha                                        
                            OUT[9]= '0';
                            OUT[10]= '0';
                            OUT[11]= '0';
                            OUT[12]= '0';
                            OUT[14]= '0';
                            OUT[15]= '0';
                            OUT[16]= '0';
                            OUT[17]= '0';
                        }else{
                            OUT[9]=  GPS1[r+2];
                            OUT[10]= GPS1[r+3];
                            OUT[11]= GPS1[r+4];
                            OUT[12]= GPS1[r+6];
                            OUT[14]= GPS1[r+7];
                            OUT[15]= GPS1[r+8];
                            OUT[16]= GPS1[r+9];
                            OUT[17]= GPS1[r+10];

                        }
                    }
                    if (l==5){//Confirmar posicion de la segunda coma de interés (Longitud)
                        if(GPS1[r+1]==','){                                            
                            OUT[18]= '0';
                            OUT[19]= '0';
                            OUT[20]= '0';
                            OUT[21]= '0';
                            OUT[22]= '0';
                            OUT[23]= '0';
                            OUT[24]= '0';
                            OUT[25]= '0';
                            OUT[26]= '0';
                        }else{
                            OUT[18]= GPS1[r+2];
                            OUT[19]= GPS1[r+3];
                            OUT[20]= GPS1[r+4];
                            OUT[21]= GPS1[r+5];
                            OUT[22]= GPS1[r+7];
                            OUT[23]= GPS1[r+8];
                            OUT[24]= GPS1[r+9];
                            OUT[25]= GPS1[r+10];
                            OUT[26]= GPS1[r+11];
                        }
                        break;
                    }
                    
                }
                
            }
            xEventGroupSetBits(xControlEventGroup, BIT_3);
        }
        bzero(GPS1,100);
        m=0;
       //xEventGroupSetBits(xControlEventGroup, BIT_4);
    }
}

void controlActionTask(void *pvParameters){
    //Valor del grupo de eventos
    EventBits_t xEventGroupValue;

    //Bits del grupo de eventos por lo que se va a esperar
    const EventBits_t xBitsToWaitfor = BIT_4;

    while(true){
        xEventGroupValue = xEventGroupWaitBits(xControlEventGroup, xBitsToWaitfor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("---- < INICIANDO CONTROL > ----\r\n");
        
    }
}

void sendPayloadTask(void *pvParameters){
    //Valor del grupo de eventos
    EventBits_t xEventGroupValue;

    //Bits del grupo de eventos por lo que se va a esperar
   const EventBits_t xBitsToWaitfor = (BIT_0 | BIT_1 | BIT_2 | BIT_3);

    while(true){
        xEventGroupValue = xEventGroupWaitBits(xControlEventGroup, xBitsToWaitfor, pdTRUE, pdTRUE, portMAX_DELAY);
        printf("---- < ENVIANDO PAYLOAD > ----\r\n");
        nrf.modeTX();
        nrf.sendMessage(OUT);
        sleep_ms(1);
        nrf.modeRX();
        bzero(OUT,32);
    }
}
