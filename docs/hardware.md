Pin configurations please refer to the respective header files in `include/drivers`

Drivers need to be rewritten if hardware changes.

Refer to [Ardupilot's hwdef](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/DAKEFPVH743Pro/hwdef.dat) for this board for further hardware information.

## MCU 
This version has been built for the DakeFPV H743 Pro flight controller board.

## IMU 
- ICM42688P via SPI interface

## Barometer
SPL06 barometer via I2C interface 

## Optical Flow 
- MTF02-P via UART interface, Micolink protocol

## Receiver 
- FlySky's AFHDS 2A receiver, i-Bus protocol.


