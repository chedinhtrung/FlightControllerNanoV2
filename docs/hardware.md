Pin configurations please refer to the respective header files in `include/drivers`

Drivers need to be rewritten if hardware changes.

## MCU 
This version has been built for the Teensy 4.0 MCU.

## IMU 
There are two IMUs supported: 
- ICM42688P via SPI interface 
- MPU9250 via I2C and SPI interface 

## Barometer
- MS5611 via I2C and SPI interface

## Optical Flow 
- MTF02-P via UART interface, Micolink protocol

## Receiver 
- FlySky's AFHDS 2A receiver, uses PPM protocol

