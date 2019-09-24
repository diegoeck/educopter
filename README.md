# educopter
Educational Quadricopter

## Description

This project aims to design a quadcopter to educational purposes.
It is focused in simplicity and well written code.

It is based on MultiWii board with following specification:
* AtMega328p Processor
* 16 MHz
* Serial 57.6 kbps
* BMA180 triaxial acceleration sensors
* ITG3200 triaxial gyroscope
* HMC5883L triaxial compass
* 4 Channel PPM Remote Control

## Install Firmware

Set communication parameters on Makefile and run make:

```
make flash
```


## Use

This Firmware implements one mode:
* Altitute - control thrust in open-loop
* Roll - control roll angle in closed-loop
* Pitch - control pitch angle in closed-loop
* Yaw - control yaw angle rate in closed-loop


## Contributors

Diego Eckhard - diegoeck@ufrgs.br - @diegoeck
