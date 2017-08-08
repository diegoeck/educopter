#ifndef CONTROL_H
#define CONTROL_H

extern float anglef[3];
extern int ref[4];
extern int pid[3];
extern int erro_ga[3];
extern int16_t gyroADC[3];
extern int motor[4];

void controle();
void atualiza_pid();

#endif
