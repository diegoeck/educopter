#ifndef ESTIMATION_H
#define ESTIMATION_H

extern uint8_t rawADC[6];
extern int16_t gyroADC[3];
extern int16_t accADC[3];

extern float anglef[3];
extern float angle[2];


void angulos();

#endif
