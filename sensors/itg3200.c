#include <avr/io.h>
#include "../lib/mi2c.h"
#include "sensors.h"

#define ITG3200_ADDRESS 0X68

#define ITG3200_DLPF_CFG   1
// DLPF_CFG FILTER  Sampling Rate
// 0        256Hz   8kHz   -> Tem grafico de bode estranho, nao recomendo.
// 1        188z   1kHz
// 2        98Hz   1kHz
// 3        42Hz   1kHz
// 4        20Hz   1kHz
// 5        10Hz   1kHz
// 6        5Hz   1kHz

#define ITG3200_SMPLRT_DIV 0
//Fsample=Finternal/(divider+1)
//se 0 então Fsample = 1kHz/(9+1)=100Hz -> 10ms 

void Gyro_init() {
    i2c_writeReg(ITG3200_ADDRESS, 0x3E, 0x80); //register: Power Management  --  value: reset device
    i2c_writeReg(ITG3200_ADDRESS, 0x15, ITG3200_SMPLRT_DIV); //register: Sample Rate Divider  -- default value = 0: OK
    i2c_writeReg(ITG3200_ADDRESS, 0x16, 0x18 + ITG3200_DLPF_CFG); //register: DLPF_CFG - low pass filter configuration
    i2c_writeReg(ITG3200_ADDRESS, 0x3E, 0x03); //register: Power Management  --  value: PLL with Z Gyro reference
}

void Gyro_getADC () {
    TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
    i2c_getSixRawADC(ITG3200_ADDRESS,0X1D);
    gyroADC[0] =  ((rawADC[0]<<8) | rawADC[1]);
    gyroADC[1] = -((rawADC[2]<<8) | rawADC[3]); // na placa está invertido
    gyroADC[2] = -((rawADC[4]<<8) | rawADC[5]); // na placa está invertido
    
    //Sensor tem medida de 14.375 bits para cada grau/segundo.
    
    gyroADC[0]=gyroADC[0]-calibra_giro_i[0];
    gyroADC[1]=gyroADC[1]-calibra_giro_i[1];
    gyroADC[2]=gyroADC[2]-calibra_giro_i[2];
    
}
