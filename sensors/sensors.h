#ifndef SENSORS_H
#define SENSORS_H

extern float calibra_giro_i[3];
extern int16_t gyroADC[3];
extern int16_t accADC[3];

void Gyro_init();
void Gyro_getADC();
void ACC_init ();
void ACC_getADC();

#endif
