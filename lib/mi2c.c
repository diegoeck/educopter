#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include "mi2c.h"


void waitTransmissionI2C() {
    uint16_t count = 255;
    while (!(TWCR & (1<<TWINT))) {
        count--;
        if (count==0) {              //we are in a blocking state => we don't insist
            TWCR = 0;                  //and we force a reset on TWINT register

            //neutralizeTime = micros(); //we take a timestamp here to neutralize the value during a short delay // Eu Desliguei ISTO
            //i2c_errors_count++;
            
            
            break;
        }
    }
}


void i2c_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    //  while(TWCR & (1<<TWSTO));                // <- can produce a blocking state with some WMP clones
}




uint8_t i2c_read(uint8_t ack) {
    TWCR = (1<<TWINT) | (1<<TWEN) | (ack? (1<<TWEA) : 0);
    waitTransmissionI2C();
    uint8_t r = TWDR;
    if (!ack) i2c_stop();
    return r;
}



void i2c_rep_start(uint8_t address) {
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) ; // send REPEAT START condition
    waitTransmissionI2C();                       // wait until transmission completed
    TWDR = address;                              // send device address
    TWCR = (1<<TWINT) | (1<<TWEN);
    waitTransmissionI2C();                       // wail until transmission completed
}

void i2c_write(uint8_t data ) {
    TWDR = data;                                 // send data to the previously addressed device
    TWCR = (1<<TWINT) | (1<<TWEN);
    waitTransmissionI2C();
}



void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val) {
    i2c_rep_start(add<<1); // I2C write direction
    i2c_write(reg);        // register selection
    i2c_write(val);        // value to write in register
    i2c_stop();
}



size_t i2c_read_to_buf(uint8_t add, void *buf, size_t size) {
    i2c_rep_start((add<<1) | 1);  // I2C read direction
    size_t bytes_read = 0;
    uint8_t *b = (uint8_t*)buf;
    while (size--) {
        /* acknowledge all but the final byte */
        *b++ = i2c_read(size > 0);
        /* TODO catch I2C errors here and abort */
        bytes_read++;
    }
    return bytes_read;
}

size_t i2c_read_reg_to_buf(uint8_t add, uint8_t reg, void *buf, size_t size) {
    i2c_rep_start(add<<1); // I2C write direction
    i2c_write(reg);        // register selection
    return i2c_read_to_buf(add, buf, size);
}

uint8_t i2c_readReg(uint8_t add, uint8_t reg) {
    uint8_t val;
    i2c_read_reg_to_buf(add, reg, &val, 1);
    return val;
}


void i2c_getSixRawADC(uint8_t add, uint8_t reg) {
    i2c_read_reg_to_buf(add, reg, &rawADC, 6);
}

