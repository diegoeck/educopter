#include <avr/io.h>
#include "../lib/mi2c.h"
#include "sensors.h"

#define BMA180_ADDRESS 0x40

void ACC_init () {
    //default range 2G: 1G = 4096 unit.
    i2c_writeReg(BMA180_ADDRESS,0x0D,1<<4); // register: ctrl_reg0  -- value: set bit ee_w to 1 to enable writing
    
    uint8_t control = i2c_readReg(BMA180_ADDRESS, 0x20);
    control = control & 0x0F;     // save tcs register
    //control = control | (0x00); // set low pass filter to 10Hz  (bits value = 0000xxxx)
    //control = control | (0x10); // set low pass filter to 20Hz  (bits value = 0001xxxx)
    //control = control | (0x20); // set low pass filter to 40Hz  (bits value = 0010xxxx)
    //control = control | (0x30); // set low pass filter to 75Hz  (bits value = 0011xxxx)
    //control = control | (0x40); // set low pass filter to 150Hz (bits value = 0100xxxx)
    //control = control | (0x50); // set low pass filter to 300Hz (bits value = 0101xxxx)
    //control = control | (0x60); // set low pass filter to 600Hz (bits value = 0110xxxx)
    control = control | (0x70); // set low pass filter to 1200Hz  (bits value = 0111xxxx)echo "<h2>Orientações em andamento</h2><h3>Doutorado</h3><ul>";
    i2c_writeReg(BMA180_ADDRESS, 0x20, control);
    
    control = i2c_readReg(BMA180_ADDRESS, 0x30);
    control = control & 0xFC;        // save tco_z register
    control = control | 0x00;        // set mode_config to 0 - LOW NOISE WITH HIGH CURRENT
    i2c_writeReg(BMA180_ADDRESS, 0x30, control);
    
    control = i2c_readReg(BMA180_ADDRESS, 0x35);
    //control = control & 0xF1;        // save offset_x and smp_skip register
    //control = control | (0x00 << 1); // set range to 1G   (bits value = xxxx000x) 8192 LSB/g
    //control = control | (0x01 << 1); // set range to 1.5G (bits value = xxxx001x) 5460 LSB/g
    control = control | (0x02 << 1);   // set range to 2G   (bits value = xxxx010x) 4096 LSB/g
    //control = control | (0x03 << 1); // set range to 3G   (bits value = xxxx011x) 2730 LSB/g
    //control = control | (0x04 << 1); // set range to 4G   (bits value = xxxx100x) 2048 LSB/g
    //control = control | (0x05 << 1); // set range to 8G   (bits value = xxxx101x) 1024 LSB/g
    //control = control | (0x06 << 1); // set range to 16G  (bits value = xxxx110x)  512 LSB/g
    i2c_writeReg(BMA180_ADDRESS, 0x35, control);
}

void ACC_getADC () {
    TWBR = ((F_CPU / 400000L) - 16) / 2;  // Optional line.  Sensor is good for it in the spec.
    i2c_getSixRawADC(BMA180_ADDRESS,0x02);
    
    //usefull info is on the 14 bits  [2-15] bits  /4 => [0-13] bits  /4 => 12 bit resolution
    accADC[0]=((rawADC[3]<<8) | rawADC[2])>>2; //Y
    accADC[1]=((rawADC[1]<<8) | rawADC[0])>>2; //X
    accADC[2]=((rawADC[5]<<8) | rawADC[4])>>2; //Z
    
    //PORQUE DIVIDIR POR 4???? ISTO ESTA ERRADO!
}
