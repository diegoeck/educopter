#include <avr/io.h>
#include "../lib/mi2c.h"
#include "sensors.h"

#define ITG3200_ADDRESS 0X68
#define ITG3200_SMPLRT_DIV 0  //8000Hz
#define ITG3200_DLPF_CFG   4 // Antes eu tinha colocado 4

#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[0] =  X; gyroADC[1] = -Y; gyroADC[2] = -Z;}


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
