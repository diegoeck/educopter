#include <avr/io.h>
#include <avr/interrupt.h>
#include "pwm.h"

void PWM_init()
{
    //Habilita saida
    DDRB |= 1<<1; // pinMode(9,  OUTPUT);  //Saida PWM
    DDRB |= 1<<2; // pinMode(10, OUTPUT);  //Saida PWM
    DDRB |= 1<<3; // pinMode(11, OUTPUT);  //Saida PWM
    DDRD |= 1<<3; // pinMode(3, OUTPUT);  //Saida PWM
    
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
}

void atualiza_motor() {
    
    OCR2A = (motor[0]>>3)+125;
    OCR2B = (motor[1]>>3)+125;
    OCR1A = (motor[2]>>2)+250;
    OCR1B = (motor[3]>>2)+250;
    
}
