#include <avr/io.h>
#include <avr/interrupt.h>
#include "rc.h"

#define MASK_P2 (1<<2)
#define MASK_P4 (1<<4)
#define MASK_P5 (1<<5)
#define MASK_P6 (1<<6)
#define MASK_P7 (1<<7)

uint8_t P2;//Status of pin 2 H or L
uint8_t P4;//Status of pin 4 H or L
uint8_t P5;//Status of pin 5 H or L
uint8_t P6;//Status of pin 6 H or L
uint8_t P7;//Status of pin 7 H or L

void RC_init()
{
//Interrupcao RC
PCICR |= 0x04;  //Interrupt2 Portas
PCMSK2 |= 0xF0;  //Interrupt2 Pinos
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
