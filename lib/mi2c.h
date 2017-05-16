#ifndef MI2C_H
#define MI2C_H

extern uint8_t rawADC[6];

void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val);
uint8_t i2c_readReg(uint8_t add, uint8_t reg);
void i2c_getSixRawADC(uint8_t add, uint8_t reg);

#endif