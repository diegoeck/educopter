#include <avr/io.h>
#include <math.h>
#include "../lib/mi2c.h"
#include "sensors.h"

#define HMC5883L_ADDRESS 0x1E

void Mag_init () {

    
    i2c_writeReg(HMC5883L_ADDRESS,0x00,0x70);
    i2c_writeReg(HMC5883L_ADDRESS,0x01,0x00);
    i2c_writeReg(HMC5883L_ADDRESS,0x02,0x00);

    
}

void Mag_getADC () {
    TWBR = ((F_CPU / 400000L) - 16) / 2;  // Optional line.  Sensor is good for it in the spec.
    i2c_getSixRawADC(HMC5883L_ADDRESS,0x03);
    
    magADC[0]=((rawADC[0]<<8) | rawADC[1])-36-85;
    magADC[1]=((rawADC[2]<<8) | rawADC[3])+7;
    magADC[2]=((rawADC[4]<<8) | rawADC[5])+167+345;

}
