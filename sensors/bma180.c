#include <avr/io.h>
#include "../lib/mi2c.h"
#include "sensors.h"

#define ACC_ORIENTATION(X, Y, Z)  {accADC[0]  = Y; accADC[1]  = X; accADC[2]  =  Z;}

#define BMA180_ADDRESS 0x40


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
