-- --------------------------------------------------------------------
-- Presentation and code corresponding to 
-- Go Raspberries
-- GDG Barcelona - 2014/11/12
-- --------------------------------------------------------------------
-- Taking hardware control of the BCM2835
-- --------------------------------------------------------------------
--
-- PhD. Esteve Farres Berenguer
-- C / Can Planes, 6
-- ES17160 - Anglès - Girona
-- Calalonia - Spain
--
-- TEL: +34 659 17 59 69 
-- web: https://plus.google.com/+EsteveFarresBerenguer/posts
-- email: esteve.farres@gmail.com
--
-- --------------------------------------------------------------------
-- Code Revision History :
-- --------------------------------------------------------------------
-- Ver: | Author |Mod. Date   |Changes Made:
-- V1.0 | E.F.B. | 2014/10/05 | Initial ver
-- --------------------------------------------------------------------

BCM2835 bridge between Go and bcm2835 C Library

Great C library, very well documented. 
http://www.airspayce.com/mikem/bcm2835

// bcm2835.h
//
// C and C++ support for Broadcom BCM 2835 as used in Raspberry Pi
//
// Author: Mike McCauley

It provides access to GPIO and other IO functions on the Broadcom BCM 2835 chip.
It provides functions for reading digital inputs and setting digital outputs, using SPI and I2C, and for accessing the system timers.
Pin event detection is supported by polling (interrupts are not supported).

