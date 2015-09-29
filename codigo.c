#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "mi2c.h"
#include "uart.h"


#define ROLL 0
#define PITCH 1
#define YAW 2
#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = Y; accADC[PITCH]  = X; accADC[YAW]  =  Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] = -Y; gyroADC[YAW] = -Z;}
#define ITG3200_ADDRESS 0X68

// #define ITG3200_SMPLRT_DIV 7  //8000Hz
//#define ITG3200_DLPF_CFG   0
#define ITG3200_SMPLRT_DIV 0  //8000Hz
#define ITG3200_DLPF_CFG   4 // Antes eu tinha colocado 4

#define BMA180_ADDRESS 0x40

#define FOSC 16000000 // Clock Speed
#define BAUD 57600

#define MYUBRR FOSC/16/BAUD-1

#define MASK_P2 (1<<2)
#define MASK_P4 (1<<4)
#define MASK_P5 (1<<5)
#define MASK_P6 (1<<6)
#define MASK_P7 (1<<7)

uint8_t P2 = 0;//Status of pin 2 H or L
uint8_t P4 = 0;//Status of pin 4 H or L
uint8_t P5 = 0;//Status of pin 5 H or L
uint8_t P6 = 0;//Status of pin 6 H or L
uint8_t P7 = 0;//Status of pin 7 H or L


float anglef[3];
float rv[2];

float erroa[2];
float errog[2];

int angle[2];


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





/*
FUNCOES GYRO
*/


void Gyro_init() {
    i2c_writeReg(ITG3200_ADDRESS, 0x3E, 0x80); //register: Power Management  --  value: reset device
    i2c_writeReg(ITG3200_ADDRESS, 0x15, ITG3200_SMPLRT_DIV); //register: Sample Rate Divider  -- default value = 0: OK
    i2c_writeReg(ITG3200_ADDRESS, 0x16, 0x18 + ITG3200_DLPF_CFG); //register: DLPF_CFG - low pass filter configuration
    i2c_writeReg(ITG3200_ADDRESS, 0x3E, 0x03); //register: Power Management  --  value: PLL with Z Gyro reference
}

void Gyro_getADC () {
    TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
    i2c_getSixRawADC(ITG3200_ADDRESS,0X1D);
    GYRO_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])>>2 , // range: +/- 8192; +/- 2000 deg/sec
                     ((rawADC[2]<<8) | rawADC[3])>>2 ,
                     ((rawADC[4]<<8) | rawADC[5])>>2 );
    
    gyroADC[0]=gyroADC[0]-calibra_giro_i[0];
    gyroADC[1]=gyroADC[1]-calibra_giro_i[1];
    gyroADC[2]=gyroADC[2]-calibra_giro_i[2];
    
    
}

/*
 FUNCOES ACC
*/


void ACC_init () {
    //default range 2G: 1G = 4096 unit.
    i2c_writeReg(BMA180_ADDRESS,0x0D,1<<4); // register: ctrl_reg0  -- value: set bit ee_w to 1 to enable writing
    
    uint8_t control = i2c_readReg(BMA180_ADDRESS, 0x20);
    control = control & 0x0F;        // save tcs register
    //control = control | (0x01 << 4); // register: bw_tcs reg: bits 4-7 to set bw -- value: set low pass filter to 20Hz
    control = control | (0x00 << 4); // set low pass filter to 10Hz (bits value = 0000xxxx)
    i2c_writeReg(BMA180_ADDRESS, 0x20, control);
    
    control = i2c_readReg(BMA180_ADDRESS, 0x30);
    control = control & 0xFC;        // save tco_z register
    control = control | 0x00;        // set mode_config to 0
    i2c_writeReg(BMA180_ADDRESS, 0x30, control);
    
    control = i2c_readReg(BMA180_ADDRESS, 0x35);
    control = control & 0xF1;        // save offset_x and smp_skip register
    control = control | (0x02 << 1); // set range to 8G
    i2c_writeReg(BMA180_ADDRESS, 0x35, control);
}

void ACC_getADC () {
    TWBR = ((F_CPU / 400000L) - 16) / 2;  // Optional line.  Sensor is good for it in the spec.
    i2c_getSixRawADC(BMA180_ADDRESS,0x02);
    //usefull info is on the 14 bits  [2-15] bits  /4 => [0-13] bits  /4 => 12 bit resolution
    ACC_ORIENTATION( ((rawADC[1]<<8) | rawADC[0])>>4 ,
                    ((rawADC[3]<<8) | rawADC[2])>>4 ,
                    ((rawADC[5]<<8) | rawADC[4])>>4 );
}






/*
 FUNCOES MATEMATICA
*/


int16_t _atan2(int32_t y, int32_t x){
    float z = (float)y / x;
    int16_t a;
    if ( abs(y) < abs(x) ){
        a = 573 * z / (1.0f + 0.28f * z * z);
        if (x<0) {
            if (y<0) a -= 1800;
            else a += 1800;
        }
    } else {
        a = 900 - 573 * z / (z * z + 0.28f);
        if (y<0) a -= 1800;
    }
    return a;
}







/*
 FUNCOES MOTOR
*/

void angulos() {
    float t[2];


    t[0]=sqrt(square((float)accADC[2])+square((float)accADC[1]));
    t[1]=sqrt(square((float)accADC[2])+square((float)accADC[0]));

    if(accADC[2]<0){
        t[1]=-t[1];
    }

    angle[0]=_atan2(accADC[0],t[0]);
    angle[1]=_atan2(accADC[1],t[1]);

    
    t[0]=(float)gyroADC[0]*cos(anglef[1]/572.9)/sqrt(  1-square( sin(anglef[0]/572.9)*sin(anglef[1]/572.9)  )   );
    t[1]=(float)gyroADC[2]*sin(anglef[1]/572.9)/sqrt(  1-square( sin(anglef[0]/572.9)*cos(anglef[1]/572.9)  )   );
    t[0]=t[0]+t[1];
    
    anglef[0]=0.01*(float)angle[0]+0.99*anglef[0]+0.02754*(t[0]);
    
    t[0]=(float)gyroADC[1]*cos(anglef[0]/572.9)/sqrt(  1+square(sin(anglef[1]/572.9)*sin(anglef[0]/572.9)   )   );
    t[1]=(float)gyroADC[2]*sin(anglef[0]/572.9)/sqrt(  1-square(sin(anglef[1]/572.9)*cos(anglef[0]/572.9)   )   );
    t[0]=t[0]-t[1];

    anglef[1]=0.01*(float)angle[1]+0.99*anglef[1]+0.02754*(t[0]);

 
    // Matlab code
    // v1(i+1)=0.01*aa(i)+0.99*v1(i)+0.99*( VarName1(i)*cos(v2(i))/sqrt(1-(sin(v1(i))*sin(v2(i)))^2)+VarName3(i)*sin(v2(i))/sqrt(1-(sin(v1(i))*cos(v2(i)))^2) )/m*pi/2 ;
    // v2(i+1)=0.01*bb(i)+0.99*v2(i)+0.99*( VarName2(i)*cos(v1(i))/sqrt(1+(sin(v2(i))*sin(v1(i)))^2)-VarName3(i)*sin(v1(i))/sqrt(1-(sin(v2(i))*cos(v1(i)))^2) )/m*pi/2 ;
 

/*
    anglef[0]=0.01*(float)angle[0]+0.02754*(float)gyroADC[0]+0.99*anglef[0];
    anglef[1]=0.01*(float)angle[1]+0.02754*(float)gyroADC[1]+0.99*anglef[1];
*/
    // 2000/8192*0.01*10 = 0.024414 //Multiwii *0.99
    // 4/14.375*0.01*10  = 0.02781  //Datasheer *0.99
}





void controle() {

    float e;
    
    pid[2]=(ref[3]-gyroADC[2]);


    pid[0]=((ref[0]-(int)anglef[0])*2-gyroADC[0])*0.25;
    pid[1]=((ref[1]-(int)anglef[1])*2-gyroADC[1])*0.25;

/*
    
    e=(ref[0]-anglef[0]);
//    rv[0]=rv[0]+4.864*e - 4.813*erroa[0];
    rv[0]=1.5*e;
    erroa[0]=e;

    if (rv[0]>1000){
        rv[0]=1000;
    }
    if (rv[0]<-1000){
        rv[0]=-1000;
    }
    
    e=rv[0]-gyroADC[0];
//    pid[0]= 2.456*e-2.361*errog[0];
    pid[0]= 0.3*e;
    errog[0]=e;

    

    
    
    e=(ref[1]-anglef[1]);
//    rv[1]=rv[1]+    4.864*e - 4.813*erroa[1];
    rv[1]=1.5*e;
    erroa[1]=e;
    
    if (rv[1]>1000){
        rv[1]=1000;
    }
    if (rv[1]<-1000){
        rv[1]=-1000;
    }

    e=rv[1]-gyroADC[1];
//    pid[1]= 2.456*e-2.361*errog[1];
    pid[1]= 0.3*e;
    errog[1]=e;

*/
           
  
}


void atualiza_pid() {
int j;

motor[0]=ref[2]+pid[0]-pid[1]-pid[2];          
motor[1]=ref[2]-pid[0]+pid[1]-pid[2];        
motor[2]=ref[2]-pid[0]-pid[1]+pid[2];          
motor[3]=ref[2]+pid[0]+pid[1]+pid[2];        

for(j=0;j<4;j++)
{
    if(motor[j]>1000) motor[j]=1000;
    if(motor[j]<90) motor[j]=90;
    if(ref[2]<10) motor[j]=0;

}

}


void atualiza_motor() {

  OCR2A = (motor[0]>>3)+125;          
  OCR2B = (motor[1]>>3)+125;        
  OCR1A = (motor[2]>>2)+250;          
  OCR1B = (motor[3]>>2)+250;        

}





void setup() {

    
    DDRB |= 1<<1; // pinMode(9,  OUTPUT);  //Saida PWM
    DDRB |= 1<<2; // pinMode(10, OUTPUT);  //Saida PWM
    DDRB |= 1<<3; // pinMode(11, OUTPUT);  //Saida PWM
    DDRD |= 1<<3; // pinMode(3, OUTPUT);  //Saida PWM
    
    DDRB |= 1<<5; // pinMode(13, OUTPUT);  //LED
    
    /* ENTRADAS NÃO PRECISA - Eh Padrao
     pinMode(2, INPUT);  //Saida PWM
     pinMode(4, INPUT);  //Saida PWM
     pinMode(5, INPUT);  //Saida PWM
     pinMode(6, INPUT);  //Saida PWM
     pinMode(7, INPUT);  //Saida PWM
     */
    
    
    //  Serial.begin(115200);
    
    
    
    cli(); //noInterrupts();           // disable all interrupts
    
    //Timer 0
    TCCR0A = 0;
    TCCR0B = 0;
    TCNT0  = 0;
    OCR0A = 159;            //10ms
    TCCR0A |= (1 << WGM01);   // CTC mode
    TCCR0B |= (1 << CS00);    // prescaler
    TIMSK0 |= (1 << OCIE0A);  // enable timer compare interrupt
    
    //Interrupcao RC
    PCICR |= 0x04;  //Interrupt2 Portas
    PCMSK2 |= 0xF0;  //Interrupt2 Pinos
    
    
    sei(); // interrupts();
    
    //Timer 1 - PWM - dividido por 64
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    OCR1A = 250;
    OCR1B = 250;
    
    TCCR1A |= (1 << COM1A1);   // Normal mode A (non-inverted)
    TCCR1A |= (1 << COM1B1);   // Normal mode B (non-inverted)
    TCCR1A |= (1 << WGM11);   // Fast PWM 9 Bits
    TCCR1B |= (1 << WGM12);   // Fast PWM 9 Bits
    TCCR1B |= (1 << CS11);    // 64 prescaler
    TCCR1B |= (1 << CS10);    // 64 prescaler
    
    //Timer 2 - PWM - dividido por 128
    TCCR2A = 0;
    TCCR2B = 0;
    TCNT2  = 0;
    OCR2A = 125;
    OCR2B = 125;
    TCCR2A |= (1 << COM2A1);   // Normal mode A (non-inverted)
    TCCR2A |= (1 << COM2B1);   // Normal mode B (non-inverted)
    TCCR2A |= (1 << WGM21);   // Fast PWM
    TCCR2A |= (1 << WGM20);   // Fast PWM
    TCCR2B |= (1 << CS22);    // 128 prescaler
    TCCR2B |= (1 << CS20);    // 128 prescaler
    
    //No timer 1 deve ser colocado o DOBRO do valor do timer 2 para mesma resposta pois PRESCALER eh diferente
    //Timer 1 entre 250 e 500
    //Timer 2 entre 125 e 250
    
    
    
    //USART_Init(MYUBRR);
    

    
    uart_init(MYUBRR);

    stdout = &uart_output;
    
    
    TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
    Gyro_init();
    ACC_init();
    
    
}


ISR(TIMER0_COMPA_vect)          // timer compare interrupt service routine
{
    
    //calibra_motor();
    tempo++;
    
    
    if(tempo>1000)
    {
        roda=1;
        tempo=0;
    }
    
    
}



ISR(PCINT2_vect)
{
    //Interrupcao Pinos para PC
    
    if ((PIND & MASK_P5) != P5)//if digital PIN5 has changed
    {
        if(P5)
        {
            pino[0]=tempo-pino[0];
            if (pino[0]<0)
            {
                pino[0]=pino[0]+1000;
            }
            ref[0]=(pino[0]-150)*10;
            
        }else{
            pino[0]=tempo;
            
        }
        P5 = PIND & MASK_P5;//Set new status
    }
    
    if ((PIND & MASK_P4) != P4)//if digital PIN4 has changed
    {
        if(P4)
        {
            pino[1]=tempo-pino[1];
            if (pino[1]<0)
            {
                pino[1]=pino[1]+1000;
            }
            ref[1]=(pino[1]-150)*10;
            
        }else{
            pino[1]=tempo;
            
        }
        P4 = PIND & MASK_P4;//Set new status
    }
    
    if ((PIND & MASK_P6) != P6)//if digital PIN6 has changed
    {
        if(P6)
        {
            pino[2]=tempo-pino[2];
            if (pino[2]<0)
            {
                pino[2]=pino[2]+1000;
            }
            ref[2]=(pino[2]-110)*10;
            
        }else{
            pino[2]=tempo;
            
        }
        P6 = PIND & MASK_P6;//Set new status
    }
    
    if ((PIND & MASK_P7) != P7)//if digital PIN7 has changed
    {
        if(P7)
        {
            pino[3]=tempo-pino[3];
            if (pino[3]<0)
            {
                pino[3]=pino[3]+1000;
            }
            //ref[3]=(pino[3]-150)*10;
            ref[3]=-(pino[3]-150)*10;
            
        }else{
            pino[3]=tempo;
            
        }
        P7 = PIND & MASK_P7;//Set new status
    }
    
    
}



void loop() {
    ocioso++;
    if(roda==1){

        roda=0;

        printf("%d\t",ocioso);
        ocioso=0;
        
        
        Gyro_getADC();
        ACC_getADC();
        
        
        if(armed==0)
        {
            
            if((ref[2]<10) & (ref[3]>390))      carmed++;
            
            if(carmed==50){
                armed=1;
                calibra_giro_i[0]=(int)calibra_giro[0];
                calibra_giro_i[1]=(int)calibra_giro[1];
                calibra_giro_i[2]=(int)calibra_giro[2];
                       // printf("AT+BAUD7");
            }
            
            motor[0]=0;
            motor[1]=0;
            motor[2]=0;
            motor[3]=0;
            
            
            PORTB &= ~(1<<5); //Desliga LED
            
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
                                    //    printf("AT");
            }
            
            
            PORTB |= 1<<5; //Liga LED
            
            
            angulos();
            controle();
            atualiza_pid();
        }
        
        
        atualiza_motor();
        
/*
        uart_int(ref[0]);
        uart_int(ref[1]);

        uart_int((int)anglef[0]);
        uart_int((int)anglef[1]);

        uart_int(gyroADC[0]);
        uart_int(gyroADC[1]);

        
        
        uart_int(pid[0]);
        uart_int(pid[1]);

 
*/
        
        
        printf("%d\t",ref[0]);
        printf("%d\t",ref[1]);
        printf("%d\t",ref[2]);
        printf("%d\t",ref[3]);

        printf("%d\t",gyroADC[0]);
        printf("%d\t",gyroADC[1]);
        printf("%d\t",gyroADC[2]);
        
        printf("%d\t",(int)anglef[0]);
        printf("%d\n",(int)anglef[1]);

        
    }
}


int main(void){
  

    setup();
    
    for(;;)
    {
        loop();
    }
    
    
    
}
