#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "lib/mi2c.h"
#include "lib/uart.h"
#include "estimation/estimation.h"
#include "control/control.h"
#include "sensors/sensors.h"
#include "rc/rc.h"
#include "pwm/pwm.h"
#include "led/led.h"



#define FOSC 16000000 // Clock Speed
#define BAUD 57600

#define MYUBRR FOSC/16/BAUD-1

float anglef[3];
float rv[2];

float erroa[2];
float errog[2];

float angle[2];

float calibra_giro[3];
float calibra_giro_i[3];

uint8_t armed=0;  //Armado-desarmado
uint8_t carmed=0; //Contador para armar

int tempo=0;  //Tempo usado no timer
uint8_t roda=0; //Se 1 roda loop se zero espera
int ocioso=0; //Tempo ocioso entre loops

int pr=0;

int motor[4];
int ref[4]={0,0,0,0};
int pino[4]={0,0,0,0};
int pid[3]={0,0,0};

//float erro1[3]={0,0,0};
//float erro2[3]={0,0,0};
//float erro3[3]={0,0,0};


uint8_t rawADC[6];
int16_t gyroADC[3];
int16_t accADC[3];
int16_t magADC[3];




int uart_putchar(char c, FILE *stream) {
/*
    if (c == '\n') {
        uart_putchar('\r', stream);
    }
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
 */
    uart0_putc(c);
    return 0;
}
FILE uart_output = FDEV_SETUP_STREAM(uart_putchar,NULL,_FDEV_SETUP_WRITE);



void setup() {

    LED_init();
    
    cli(); //noInterrupts();           // disable all interrupts
    
    //Timer 0
    TCCR0A = 0;
    TCCR0B = 0;
    TCNT0  = 0;
    OCR0A = 159;            //10ms
    TCCR0A |= (1 << WGM01);   // CTC mode
    TCCR0B |= (1 << CS00);    // prescaler
    TIMSK0 |= (1 << OCIE0A);  // enable timer compare interrupt
    
    
    RC_init();
    
    sei(); // interrupts();
    
    PWM_init();

    uart_init(MYUBRR);

    stdout = &uart_output;
    
    TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
    Gyro_init();
    ACC_init();
    Mag_init();
    
    
}


ISR(TIMER0_COMPA_vect)
{
    tempo++;
    if(tempo>1000)
    {
        roda=1;
        tempo=0;
    }
}

void loop() {
    ocioso++;
    if(roda==1){

        roda=0;

        //printf("%d\t",ocioso);
        ocioso=0;
        
        
        Gyro_getADC();
        ACC_getADC();
        Mag_getADC();
        
        
        if(armed==0)
        {
            
            if((ref[2]<10) & (ref[3]>390))      carmed++;
            
            if(carmed==50){
                armed=1;
                
                calibra_giro_i[0]=(int)calibra_giro[0];
                calibra_giro_i[1]=(int)calibra_giro[1];
                calibra_giro_i[2]=(int)calibra_giro[2];
            }
            
            motor[0]=0;
            motor[1]=0;
            motor[2]=0;
            motor[3]=0;
            
            desliga_led();
            
            
            calibra_giro[0]=0.1*gyroADC[0]+0.9*calibra_giro[0];
            calibra_giro[1]=0.1*gyroADC[1]+0.9*calibra_giro[1];
            calibra_giro[2]=0.1*gyroADC[2]+0.9*calibra_giro[2];
            
        }
        else
        {
            
            if((ref[2]<10) & (ref[3]<-390))    carmed--;
            
            if(carmed==0)
            {
                armed=0;
                calibra_giro_i[0]=0;
                calibra_giro_i[1]=0;
                calibra_giro_i[2]=0;
            }
            
            
            liga_led();
            
            
            angulos();
            controle();
            atualiza_pid();
        }
        
        
        atualiza_motor();
        
        //printf("%d\t",ref[0]);
        //printf("%d\t",ref[1]);
        //printf("%d\t",ref[2]);
        //printf("%d\n",ref[3]);
        
        //printf("%d\t",magADC[0]);
        //printf("%d\t",magADC[1]);
        //printf("%d\n",magADC[2]);

        //printf("%d;",gyroADC[0]);
        //printf("%d;",gyroADC[1]);
        //printf("%d\n",gyroADC[2]);

        //printf("%d\t",accADC[0]);
        //printf("%d\t",accADC[1]);
        //printf("%d\n",accADC[2]);

        
        printf("%d\t",(int)(anglef[0]*180/3.1415));
        printf("%d\t",(int)(anglef[1]*180/3.1415));
        printf("%d\n",(int)(anglef[2]*180/3.1415));
        

        
        
    }
}


int main(void){
  

    setup();
    
    for(;;)
    {
        loop();
    }
    
    
    
}
