#include <avr/io.h>
#include "led.h"

void LED_init()
{
    DDRB |= 1<<5; // pinMode(13, OUTPUT);  //LED
}


void desliga_led()
{
    PORTB &= ~(1<<5); //Desliga LED
}

void liga_led()
{
    PORTB |= 1<<5; //Liga LED
}


