
Presentation and code corresponding to 
**Go and Raspberries**
*GDG Barcelona Event - 2014/11/12*

## Objective
The objective of the presentation is describe how is possible take the control of the hardware peripherals of the broadcom BCM2835 SoC.

BCM2835 is a binding between Go and the bcm2835 C Library, a great C library, very well documented. 

Library source is available at http://www.airspayce.com/mikem/bcm2835

C and C++ support for Broadcom BCM 2835 as used in Raspberry Pi projects.

Author: *Mike McCauley*

## bcm2835 library
It provides access to GPIO and other IO functions on the Broadcom BCM2835 chip.

It provides functions for reading digital inputs and setting digital outputs, using SPI and I2C, and for accessing the system timers.

Pin event detection is supported by polling (interrupts are not supported).

