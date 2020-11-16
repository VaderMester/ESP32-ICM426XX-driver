# CubeOS V0.1

## Current Platform
Currently CubeOS is running on an ESP32, Dual Core version.
For the RAM requirements it's running on a WROVER module. Production variant will use a version with 16MB of SPI-FLASH

## WROVER IO USAGE
ESP32 has a limited set of GPIO pins, but current stage pins are enough.
LED Matric Power Enable function on Pin32 does not work yet, as well as none of inputs are currently configured. Probably in V0.2 :)

I want to add a vibration motor output later. Since we are out of GPIOs, I consider the UART RX (GPIO3) as the output for the motor. For this pin a pull-down resistor will be attached.
To allow UART data on the RX pin during development, an AND gate will be used to AND LED Matrix PWR EN and Motor signals to drive the motor.

[cubeOS IO](http://www.kepfeltoltes.eu/view.php?filename=776v0.1_IO.jpg)