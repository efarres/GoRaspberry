// BCM2835Bridge

// #cgo LDFLAGS: -l bcm2835 -static

package bcm2835

/*
#include <bcm2835.h>
#include <stdint.h>
#include <stdlib.h>
*/
import "C"

import (
	"errors"
	"unsafe"
)

const (
	HIGH = uint8(0x1) /// This means pin HIGH, true, 3.3volts on a pin.
	LOW  = uint8(0x0) /// This means pin LOW, false, 0volts on a pin.
)

/// Speed of the core clock core_clk
const BCM2835_CORE_CLK_HZ = 250000000 ///< 250 MHz

// Physical addresses for various peripheral register sets
/// Base Physical Address of the BCM 2835 peripheral registers
const BCM2835_PERI_BASE = 0x20000000

/// Base Physical Address of the System Timer registers
const BCM2835_ST_BASE = (BCM2835_PERI_BASE + 0x3000)

/// Base Physical Address of the Pads registers
const BCM2835_GPIO_PADS = (BCM2835_PERI_BASE + 0x100000)

/// Base Physical Address of the Clock/timer registers
const BCM2835_CLOCK_BASE = (BCM2835_PERI_BASE + 0x101000)

/// Base Physical Address of the GPIO registers
const BCM2835_GPIO_BASE = (BCM2835_PERI_BASE + 0x200000)

/// Base Physical Address of the SPI0 registers
const BCM2835_SPI0_BASE = (BCM2835_PERI_BASE + 0x204000)

/// Base Physical Address of the BSC0 registers
const BCM2835_BSC0_BASE = (BCM2835_PERI_BASE + 0x205000)

/// Base Physical Address of the PWM registers
const BCM2835_GPIO_PWM = (BCM2835_PERI_BASE + 0x20C000)

/// Base Physical Address of the BSC1 registers
const BCM2835_BSC1_BASE = (BCM2835_PERI_BASE + 0x804000)

/// Size of memory page on RPi
const BCM2835_PAGE_SIZE = (4 * 1024)

/// Size of memory block on RPi
const BCM2835_BLOCK_SIZE = (4 * 1024)

// Defines for GPIO
// The BCM2835 has 54 GPIO pins.
//      BCM2835 data sheet, Page 90 onwards.
/// GPIO register offsets from BCM2835_GPIO_BASE. Offsets into the GPIO Peripheral block in bytes per 6.1 Register View
const BCM2835_GPFSEL0 = 0x0000   ///< GPIO Function Select 0
const BCM2835_GPFSEL1 = 0x0004   ///< GPIO Function Select 1
const BCM2835_GPFSEL2 = 0x0008   ///< GPIO Function Select 2
const BCM2835_GPFSEL3 = 0x000c   ///< GPIO Function Select 3
const BCM2835_GPFSEL4 = 0x0010   ///< GPIO Function Select 4
const BCM2835_GPFSEL5 = 0x0014   ///< GPIO Function Select 5
const BCM2835_GPSET0 = 0x001c    ///< GPIO Pin Output Set 0
const BCM2835_GPSET1 = 0x0020    ///< GPIO Pin Output Set 1
const BCM2835_GPCLR0 = 0x0028    ///< GPIO Pin Output Clear 0
const BCM2835_GPCLR1 = 0x002c    ///< GPIO Pin Output Clear 1
const BCM2835_GPLEV0 = 0x0034    ///< GPIO Pin Level 0
const BCM2835_GPLEV1 = 0x0038    ///< GPIO Pin Level 1
const BCM2835_GPEDS0 = 0x0040    ///< GPIO Pin Event Detect Status 0
const BCM2835_GPEDS1 = 0x0044    ///< GPIO Pin Event Detect Status 1
const BCM2835_GPREN0 = 0x004c    ///< GPIO Pin Rising Edge Detect Enable 0
const BCM2835_GPREN1 = 0x0050    ///< GPIO Pin Rising Edge Detect Enable 1
const BCM2835_GPFEN0 = 0x0058    ///< GPIO Pin Falling Edge Detect Enable 0
const BCM2835_GPFEN1 = 0x005c    ///< GPIO Pin Falling Edge Detect Enable 1
const BCM2835_GPHEN0 = 0x0064    ///< GPIO Pin High Detect Enable 0
const BCM2835_GPHEN1 = 0x0068    ///< GPIO Pin High Detect Enable 1
const BCM2835_GPLEN0 = 0x0070    ///< GPIO Pin Low Detect Enable 0
const BCM2835_GPLEN1 = 0x0074    ///< GPIO Pin Low Detect Enable 1
const BCM2835_GPAREN0 = 0x007c   ///< GPIO Pin Async. Rising Edge Detect 0
const BCM2835_GPAREN1 = 0x0080   ///< GPIO Pin Async. Rising Edge Detect 1
const BCM2835_GPAFEN0 = 0x0088   ///< GPIO Pin Async. Falling Edge Detect 0
const BCM2835_GPAFEN1 = 0x008c   ///< GPIO Pin Async. Falling Edge Detect 1
const BCM2835_GPPUD = 0x0094     ///< GPIO Pin Pull-up/down Enable
const BCM2835_GPPUDCLK0 = 0x0098 ///< GPIO Pin Pull-up/down Enable Clock 0
const BCM2835_GPPUDCLK1 = 0x009c ///< GPIO Pin Pull-up/down Enable Clock 1

/// \brief bcm2835PortFunction
/// Port function select modes for bcm2835_gpio_fsel()
const (
	BCM2835_GPIO_FSEL_INPT = "0b000" ///< Input
	BCM2835_GPIO_FSEL_OUTP = "0b001" ///< Output
	BCM2835_GPIO_FSEL_ALT0 = "0b100" ///< Alternate function 0
	BCM2835_GPIO_FSEL_ALT1 = "0b101" ///< Alternate function 1
	BCM2835_GPIO_FSEL_ALT2 = "0b110" ///< Alternate function 2
	BCM2835_GPIO_FSEL_ALT3 = "0b111" ///< Alternate function 3
	BCM2835_GPIO_FSEL_ALT4 = "0b011" ///< Alternate function 4
	BCM2835_GPIO_FSEL_ALT5 = "0b010" ///< Alternate function 5
	BCM2835_GPIO_FSEL_MASK = "0b111" ///< Function select bits mask
)

/// \brief bcm2835PUDControl
/// Pullup/Pulldown defines for bcm2835_gpio_pud()
const (
	BCM2835_GPIO_PUD_OFF  = "0b00" ///< Off ? disable pull-up/down
	BCM2835_GPIO_PUD_DOWN = "0b01" ///< Enable Pull Down control
	BCM2835_GPIO_PUD_UP   = "0b10" ///< Enable Pull Up control
)

/// Pad control register offsets from BCM2835_GPIO_PADS
const BCM2835_PADS_GPIO_0_27 = 0x002c  ///< Pad control register for pads 0 to 27
const BCM2835_PADS_GPIO_28_45 = 0x0030 ///< Pad control register for pads 28 to 45
const BCM2835_PADS_GPIO_46_53 = 0x0034 ///< Pad control register for pads 46 to 53

/// Pad Control masks
const BCM2835_PAD_PASSWRD = (0x5A << 24)     ///< Password to enable setting pad mask
const BCM2835_PAD_SLEW_RATE_UNLIMITED = 0x10 ///< Slew rate unlimited
const BCM2835_PAD_HYSTERESIS_ENABLED = 0x08  ///< Hysteresis enabled
const BCM2835_PAD_DRIVE_2mA = 0x00           ///< 2mA drive current
const BCM2835_PAD_DRIVE_4mA = 0x01           ///< 4mA drive current
const BCM2835_PAD_DRIVE_6mA = 0x02           ///< 6mA drive current
const BCM2835_PAD_DRIVE_8mA = 0x03           ///< 8mA drive current
const BCM2835_PAD_DRIVE_10mA = 0x04          ///< 10mA drive current
const BCM2835_PAD_DRIVE_12mA = 0x05          ///< 12mA drive current
const BCM2835_PAD_DRIVE_14mA = 0x06          ///< 14mA drive current
const BCM2835_PAD_DRIVE_16mA = 0x07          ///< 16mA drive current

/// \brief bcm2835PadGroup
/// Pad group specification for bcm2835_gpio_pad()
const (
	BCM2835_PAD_GROUP_GPIO_0_27  = 0 ///< Pad group for GPIO pads 0 to 27
	BCM2835_PAD_GROUP_GPIO_28_45 = 1 ///< Pad group for GPIO pads 28 to 45
	BCM2835_PAD_GROUP_GPIO_46_53 = 2 ///< Pad group for GPIO pads 46 to 53
)

const (
	Input  = 0
	Output = 1
)

/// \brief GPIO Pin Numbers
///
/// Here we define Raspberry Pin GPIO pins on P1 in terms of the underlying BCM GPIO pin numbers.
/// These can be passed as a pin number to any function requiring a pin.
/// Not all pins on the RPi 26 bin IDE plug are connected to GPIO pins
/// and some can adopt an alternate function.
/// RPi version 2 has some slightly different pinouts, and these are values RPI_V2_*.
/// At bootup, pins 8 and 10 are set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively
/// When SPI0 is in use (ie after bcm2835_spi_begin()), pins 19, 21, 23, 24, 26 are dedicated to SPI
/// and cant be controlled independently
const (
	RPI_GPIO_P1_03 = uint8(0)  ///< Version 1, Pin P1-03
	RPI_GPIO_P1_05 = uint8(1)  ///< Version 1, Pin P1-05
	RPI_GPIO_P1_07 = uint8(4)  ///< Version 1, Pin P1-07
	RPI_GPIO_P1_08 = uint8(14) ///< Version 1, Pin P1-08, defaults to alt function 0 UART0_TXD
	RPI_GPIO_P1_10 = uint8(15) ///< Version 1, Pin P1-10, defaults to alt function 0 UART0_RXD
	RPI_GPIO_P1_11 = uint8(17) ///< Version 1, Pin P1-11
	RPI_GPIO_P1_12 = uint8(18) ///< Version 1, Pin P1-12, can be PWM channel 0 in ALT FUN 5
	RPI_GPIO_P1_13 = uint8(21) ///< Version 1, Pin P1-13
	RPI_GPIO_P1_15 = uint8(22) ///< Version 1, Pin P1-15
	RPI_GPIO_P1_16 = uint8(23) ///< Version 1, Pin P1-16
	RPI_GPIO_P1_18 = uint8(24) ///< Version 1, Pin P1-18
	RPI_GPIO_P1_19 = uint8(10) ///< Version 1, Pin P1-19, MOSI when SPI0 in use
	RPI_GPIO_P1_21 = uint8(9)  ///< Version 1, Pin P1-21, MISO when SPI0 in use
	RPI_GPIO_P1_22 = uint8(25) ///< Version 1, Pin P1-22
	RPI_GPIO_P1_23 = uint8(11) ///< Version 1, Pin P1-23, CLK when SPI0 in use
	RPI_GPIO_P1_24 = uint8(8)  ///< Version 1, Pin P1-24, CE0 when SPI0 in use
	RPI_GPIO_P1_26 = uint8(7)  ///< Version 1, Pin P1-26, CE1 when SPI0 in use

	// RPi Version 2
	RPI_V2_GPIO_P1_03 = uint8(2)  ///< Version 2, Pin P1-03
	RPI_V2_GPIO_P1_05 = uint8(3)  ///< Version 2, Pin P1-05
	RPI_V2_GPIO_P1_07 = uint8(4)  ///< Version 2, Pin P1-07
	RPI_V2_GPIO_P1_08 = uint8(14) ///< Version 2, Pin P1-08, defaults to alt function 0 UART0_TXD
	RPI_V2_GPIO_P1_10 = uint8(15) ///< Version 2, Pin P1-10, defaults to alt function 0 UART0_RXD
	RPI_V2_GPIO_P1_11 = uint8(17) ///< Version 2, Pin P1-11
	RPI_V2_GPIO_P1_12 = uint8(18) ///< Version 2, Pin P1-12, can be PWM channel 0 in ALT FUN 5
	RPI_V2_GPIO_P1_13 = uint8(27) ///< Version 2, Pin P1-13
	RPI_V2_GPIO_P1_15 = uint8(22) ///< Version 2, Pin P1-15
	RPI_V2_GPIO_P1_16 = uint8(23) ///< Version 2, Pin P1-16
	RPI_V2_GPIO_P1_18 = uint8(24) ///< Version 2, Pin P1-18
	RPI_V2_GPIO_P1_19 = uint8(10) ///< Version 2, Pin P1-19, MOSI when SPI0 in use
	RPI_V2_GPIO_P1_21 = uint8(9)  ///< Version 2, Pin P1-21, MISO when SPI0 in use
	RPI_V2_GPIO_P1_22 = uint8(25) ///< Version 2, Pin P1-22
	RPI_V2_GPIO_P1_23 = uint8(11) ///< Version 2, Pin P1-23, CLK when SPI0 in use
	RPI_V2_GPIO_P1_24 = uint8(8)  ///< Version 2, Pin P1-24, CE0 when SPI0 in use
	RPI_V2_GPIO_P1_26 = uint8(7)  ///< Version 2, Pin P1-26, CE1 when SPI0 in use

	// RPi Version 2, new plug P5
	RPI_V2_GPIO_P5_03 = uint8(28) ///< Version 2, Pin P5-03
	RPI_V2_GPIO_P5_04 = uint8(29) ///< Version 2, Pin P5-04
	RPI_V2_GPIO_P5_05 = uint8(30) ///< Version 2, Pin P5-05
	RPI_V2_GPIO_P5_06 = uint8(31) ///< Version 2, Pin P5-06
)

/// \brief bcm2835SPIBitOrder SPI Bit order
/// Specifies the SPI data bit ordering for bcm2835_spi_setBitOrder()
const BCM2835_SPI_BIT_ORDER_LSBFIRST = uint8(0) ///< LSB First
const BCM2835_SPI_BIT_ORDER_MSBFIRST = uint8(1) ///< MSB First

/// \brief SPI Data mode
/// Specify the SPI data mode to be passed to bcm2835_spi_setDataMode()
const BCM2835_SPI_MODE0 = uint8(0) ///< CPOL = 0, CPHA = 0
const BCM2835_SPI_MODE1 = uint8(1) ///< CPOL = 0, CPHA = 1
const BCM2835_SPI_MODE2 = uint8(2) ///< CPOL = 1, CPHA = 0
const BCM2835_SPI_MODE3 = uint8(3) ///< CPOL = 1, CPHA = 1

/// \brief bcm2835SPIChipSelect
/// Specify the SPI chip select pin(s)
const BCM2835_SPI_CS0 = uint8(0)     ///< Chip Select 0
const BCM2835_SPI_CS1 = uint8(1)     ///< Chip Select 1
const BCM2835_SPI_CS2 = uint8(2)     ///< Chip Select 2 (ie pins CS1 and CS2 are asserted)
const BCM2835_SPI_CS_NONE = uint8(3) ///< No CS, control it yourself

/// \brief bcm2835SPIClockDivider
/// Specifies the divider used to generate the SPI clock from the system clock.
/// Figures below give the divider, clock period and clock frequency.
/// Clock divided is based on nominal base clock rate of 250MHz
/// It is reported that (contrary to the documentation) any even divider may used.
/// The frequencies shown for each divider have been confirmed by measurement
const BCM2835_SPI_CLOCK_DIVIDER_65536 = uint16(0)     ///< 65536 = 262.144us = 3.814697260kHz
const BCM2835_SPI_CLOCK_DIVIDER_32768 = uint16(32768) ///< 32768 = 131.072us = 7.629394531kHz
const BCM2835_SPI_CLOCK_DIVIDER_16384 = uint16(16384) ///< 16384 = 65.536us = 15.25878906kHz
const BCM2835_SPI_CLOCK_DIVIDER_8192 = uint16(8192)   ///< 8192 = 32.768us = 30/51757813kHz
const BCM2835_SPI_CLOCK_DIVIDER_4096 = uint16(4096)   ///< 4096 = 16.384us = 61.03515625kHz
const BCM2835_SPI_CLOCK_DIVIDER_2048 = uint16(2048)   ///< 2048 = 8.192us = 122.0703125kHz
const BCM2835_SPI_CLOCK_DIVIDER_1024 = uint16(1024)   ///< 1024 = 4.096us = 244.140625kHz
const BCM2835_SPI_CLOCK_DIVIDER_512 = uint16(512)     ///< 512 = 2.048us = 488.28125kHz
const BCM2835_SPI_CLOCK_DIVIDER_256 = uint16(256)     ///< 256 = 1.024us = 976.5625MHz
const BCM2835_SPI_CLOCK_DIVIDER_128 = uint16(128)     ///< 128 = 512ns = = 1.953125MHz
const BCM2835_SPI_CLOCK_DIVIDER_64 = uint16(64)       ///< 64 = 256ns = 3.90625MHz
const BCM2835_SPI_CLOCK_DIVIDER_32 = uint16(32)       ///< 32 = 128ns = 7.8125MHz
const BCM2835_SPI_CLOCK_DIVIDER_16 = uint16(16)       ///< 16 = 64ns = 15.625MHz
const BCM2835_SPI_CLOCK_DIVIDER_8 = uint16(8)         ///< 8 = 32ns = 31.25MHz
const BCM2835_SPI_CLOCK_DIVIDER_4 = uint16(4)         ///< 4 = 16ns = 62.5MHz
const BCM2835_SPI_CLOCK_DIVIDER_2 = uint16(2)         ///< 2 = 8ns = 125MHz, fastest you can get
const BCM2835_SPI_CLOCK_DIVIDER_1 = uint16(1)         ///< 1 = 262.144us = 3.814697260kHz, same as 0/65536

// Defines for I2C
// GPIO register offsets from BCM2835_BSC*_BASE.
// Offsets into the BSC Peripheral block in bytes per 3.1 BSC Register Map
const BCM2835_BSC_C = 0x0000    ///< BSC Master Control
const BCM2835_BSC_S = 0x0004    ///< BSC Master Status
const BCM2835_BSC_DLEN = 0x0008 ///< BSC Master Data Length
const BCM2835_BSC_A = 0x000c    ///< BSC Master Slave Address
const BCM2835_BSC_FIFO = 0x0010 ///< BSC Master Data FIFO
const BCM2835_BSC_DIV = 0x0014  ///< BSC Master Clock Divider
const BCM2835_BSC_DEL = 0x0018  ///< BSC Master Data Delay
const BCM2835_BSC_CLKT = 0x001c ///< BSC Master Clock Stretch Timeout

// Register masks for BSC_C
const BCM2835_BSC_C_I2CEN = 0x00008000   ///< I2C Enable, 0 = disabled, 1 = enabled
const BCM2835_BSC_C_INTR = 0x00000400    ///< Interrupt on RX
const BCM2835_BSC_C_INTT = 0x00000200    ///< Interrupt on TX
const BCM2835_BSC_C_INTD = 0x00000100    ///< Interrupt on DONE
const BCM2835_BSC_C_ST = 0x00000080      ///< Start transfer, 1 = Start a new transfer
const BCM2835_BSC_C_CLEAR_1 = 0x00000020 ///< Clear FIFO Clear
const BCM2835_BSC_C_CLEAR_2 = 0x00000010 ///< Clear FIFO Clear
const BCM2835_BSC_C_READ = 0x00000001    ///< Read transfer

// Register masks for BSC_S
const BCM2835_BSC_S_CLKT = 0x00000200 ///< Clock stretch timeout
const BCM2835_BSC_S_ERR = 0x00000100  ///< ACK error
const BCM2835_BSC_S_RXF = 0x00000080  ///< RXF FIFO full, 0 = FIFO is not full, 1 = FIFO is full
const BCM2835_BSC_S_TXE = 0x00000040  ///< TXE FIFO full, 0 = FIFO is not full, 1 = FIFO is full
const BCM2835_BSC_S_RXD = 0x00000020  ///< RXD FIFO contains data
const BCM2835_BSC_S_TXD = 0x00000010  ///< TXD FIFO can accept data
const BCM2835_BSC_S_RXR = 0x00000008  ///< RXR FIFO needs reading (full)
const BCM2835_BSC_S_TXW = 0x00000004  ///< TXW FIFO needs writing (full)
const BCM2835_BSC_S_DONE = 0x00000002 ///< Transfer DONE
const BCM2835_BSC_S_TA = 0x00000001   ///< Transfer Active

const BCM2835_BSC_FIFO_SIZE = 16 ///< BSC FIFO size

/// \brief bcm2835I2CClockDivider
/// Specifies the divider used to generate the I2C clock from the system clock.
/// Clock divided is based on nominal base clock rate of 250MHz
const (
	BCM2835_I2C_CLOCK_DIVIDER_2500 = 2500 ///< 2500 = 10us = 100 kHz
	BCM2835_I2C_CLOCK_DIVIDER_626  = 626  ///< 622 = 2.504us = 399.3610 kHz
	BCM2835_I2C_CLOCK_DIVIDER_150  = 150  ///< 150 = 60ns = 1.666 MHz (default at reset)
	BCM2835_I2C_CLOCK_DIVIDER_148  = 148  ///< 148 = 59ns = 1.689 MHz
)

/// \brief bcm2835I2CReasonCodes
/// Specifies the reason codes for the bcm2835_i2c_write and bcm2835_i2c_read functions.
const (
	BCM2835_I2C_REASON_OK         = 0x00 ///< Success
	BCM2835_I2C_REASON_ERROR_NACK = 0x01 ///< Received a NACK
	BCM2835_I2C_REASON_ERROR_CLKT = 0x02 ///< Received Clock Stretch Timeout
	BCM2835_I2C_REASON_ERROR_DATA = 0x04 ///< Not all data is sent / received
)

// Defines for ST
// GPIO register offsets from BCM2835_ST_BASE.
// Offsets into the ST Peripheral block in bytes per 12.1 System Timer Registers
// The System Timer peripheral provides four 32-bit timer channels and a single 64-bit free running counter.
// BCM2835_ST_CLO is the System Timer Counter Lower bits register.
// The system timer free-running counter lower register is a read-only register that returns the current value
// of the lower 32-bits of the free running counter.
// BCM2835_ST_CHI is the System Timer Counter Upper bits register.
// The system timer free-running counter upper register is a read-only register that returns the current value
// of the upper 32-bits of the free running counter.
const BCM2835_ST_CS = 0x0000  ///< System Timer Control/Status
const BCM2835_ST_CLO = 0x0004 ///< System Timer Counter Lower 32 bits
const BCM2835_ST_CHI = 0x0008 ///< System Timer Counter Upper 32 bits

/// @}

// Defines for PWM, word offsets (ie 4 byte multiples)
const BCM2835_PWM_CONTROL = 0
const BCM2835_PWM_STATUS = 1
const BCM2835_PWM_DMAC = 2
const BCM2835_PWM0_RANGE = 4
const BCM2835_PWM0_DATA = 5
const BCM2835_PWM_FIF1 = 6
const BCM2835_PWM1_RANGE = 8
const BCM2835_PWM1_DATA = 9

// Defines for PWM Clock, word offsets (ie 4 byte multiples)
const BCM2835_PWMCLK_CNTL = 40
const BCM2835_PWMCLK_DIV = 41
const BCM2835_PWM_PASSWRD = (0x5A << 24) ///< Password to enable setting PWM clock

const BCM2835_PWM1_MS_MODE = 0x8000  ///< Run in Mark/Space mode
const BCM2835_PWM1_USEFIFO = 0x2000  ///< Data from FIFO
const BCM2835_PWM1_REVPOLAR = 0x1000 ///< Reverse polarity
const BCM2835_PWM1_OFFSTATE = 0x0800 ///< Ouput Off state
const BCM2835_PWM1_REPEATFF = 0x0400 ///< Repeat last value if FIFO empty
const BCM2835_PWM1_SERIAL = 0x0200   ///< Run in serial mode
const BCM2835_PWM1_ENABLE = 0x0100   ///< Channel Enable

const BCM2835_PWM0_MS_MODE = 0x0080   ///< Run in Mark/Space mode
const BCM2835_PWM_CLEAR_FIFO = 0x0040 ///< Clear FIFO
const BCM2835_PWM0_USEFIFO = 0x0020   ///< Data from FIFO
const BCM2835_PWM0_REVPOLAR = 0x0010  ///< Reverse polarity
const BCM2835_PWM0_OFFSTATE = 0x0008  ///< Ouput Off state
const BCM2835_PWM0_REPEATFF = 0x0004  ///< Repeat last value if FIFO empty
const BCM2835_PWM0_SERIAL = 0x0002    ///< Run in serial mode
const BCM2835_PWM0_ENABLE = 0x0001    ///< Channel Enable

/// \brief bcm2835PWMClockDivider
/// Specifies the divider used to generate the PWM clock from the system clock.
/// Figures below give the divider, clock period and clock frequency.
/// Clock divided is based on nominal PWM base clock rate of 19.2MHz
/// The frequencies shown for each divider have been confirmed by measurement
const (
	BCM2835_PWM_CLOCK_DIVIDER_32768 = 32768 ///< 32768 = 585Hz
	BCM2835_PWM_CLOCK_DIVIDER_16384 = 16384 ///< 16384 = 1171.8Hz
	BCM2835_PWM_CLOCK_DIVIDER_8192  = 8192  ///< 8192 = 2.34375kHz
	BCM2835_PWM_CLOCK_DIVIDER_4096  = 4096  ///< 4096 = 4.6875kHz
	BCM2835_PWM_CLOCK_DIVIDER_2048  = 2048  ///< 2048 = 9.375kHz
	BCM2835_PWM_CLOCK_DIVIDER_1024  = 1024  ///< 1024 = 18.75kHz
	BCM2835_PWM_CLOCK_DIVIDER_512   = 512   ///< 512 = 37.5kHz
	BCM2835_PWM_CLOCK_DIVIDER_256   = 256   ///< 256 = 75kHz
	BCM2835_PWM_CLOCK_DIVIDER_128   = 128   ///< 128 = 150kHz
	BCM2835_PWM_CLOCK_DIVIDER_64    = 64    ///< 64 = 300kHz
	BCM2835_PWM_CLOCK_DIVIDER_32    = 32    ///< 32 = 600.0kHz
	BCM2835_PWM_CLOCK_DIVIDER_16    = 16    ///< 16 = 1.2MHz
	BCM2835_PWM_CLOCK_DIVIDER_8     = 8     ///< 8 = 2.4MHz
	BCM2835_PWM_CLOCK_DIVIDER_4     = 4     ///< 4 = 4.8MHz
	BCM2835_PWM_CLOCK_DIVIDER_2     = 2     ///< 2 = 9.6MHz, fastest you can get
	BCM2835_PWM_CLOCK_DIVIDER_1     = 1     ///< 1 = 4.6875kHz, same as divider 4096
)

func Random() int {
	return int(C.random())
}

func Seed(i int) {
	C.srandom(C.uint(i))
}

/// \defgroup init Library initialisation and management
/// These functions allow you to intialise and control the bcm2835 library
/// @{

func Init() (err error) {
	/// Initialise the library by opening /dev/mem and getting pointers to the
	/// internal memory for BCM 2835 device registers. You must call this (successfully)
	/// before calling any other
	/// functions in this library (except bcm2835_set_debug).
	/// If bcm2835_init() fails by returning 0,
	/// calling any other function may result in crashes or other failures.
	/// Prints messages to stderr in case of errors.
	/// \return 1 if successful else 0
	/// extern int bcm2835_init(void);
	if C.bcm2835_init() == 0 {
		return errors.New("Init: failed")
	}
	return
}

func Close() (err error) {
	/// Close the library, deallocating any allocated memory and closing /dev/mem
	/// \return 1 if successful else 0
	/// extern int bcm2835_close(void);
	if C.bcm2835_close() == 0 {
		return errors.New("Close: failed")
	}
	return
}

func SetDebug(debug uint8) {
	/// Sets the debug level of the library.
	/// A value of 1 prevents mapping to /dev/mem, and makes the library print out
	/// what it would do, rather than accessing the GPIO registers.
	/// A value of 0, the default, causes normal operation.
	/// Call this before calling bcm2835_init();
	/// \param[in] debug The new debug level. 1 means debug
	/// extern void  bcm2835_set_debug(uint8_t debug);
	C.bcm2835_set_debug(C.uint8_t(debug))
}

/// \defgroup lowlevel Low level register access
/// These functions provide low level register access, and should not generally
/// need to be used
///
/// @{
/*
func PeriRead(paddr uint32) uint32 {
	/// Reads 32 bit value from a peripheral address
	/// The read is done twice, and is therefore always safe in terms of
	/// manual section 1.3 Peripheral access precautions for correct memory ordering
	/// \param[in] paddr Physical address to read from. See BCM2835_GPIO_BASE etc.
	/// \return the value read from the 32 bit register
	/// \sa Physical Addresses
	/// extern uint32_t bcm2835_peri_read(volatile uint32_t* paddr);
    paddrC := (C.uint32_t)(paddr)
	return uint32(C.bcm2835_peri_read(paddrC))
}

func PeriReadNB(paddr uint32) uint32 {
	/// Reads 32 bit value from a peripheral address without the read barrier
	/// You should only use this when your code has previously called bcm2835_peri_read()
	/// within the same peripheral, and no other peripheral access has occurred since.
	/// \param[in] paddr Physical address to read from. See BCM2835_GPIO_BASE etc.
	/// \return the value read from the 32 bit register
	/// \sa Physical Addresses
	/// extern uint32_t bcm2835_peri_read_nb(volatile uint32_t* paddr);
    paddrC := (C.uint32_t)(paddr)
	return uint32(C.bcm2835_peri_read_nb(paddrC))
}

func PeriWrite(paddr, value uint32) {
	/// Writes 32 bit value from a peripheral address
	/// The write is done twice, and is therefore always safe in terms of
	/// manual section 1.3 Peripheral access precautions for correct memory ordering
	/// \param[in] paddr Physical address to read from. See BCM2835_GPIO_BASE etc.
	/// \param[in] value The 32 bit value to write
	/// \sa Physical Addresses
	/// extern void bcm2835_peri_write(volatile uint32_t* paddr, uint32_t value);
    paddrC := (C.uint32_t)(paddr)
	C.bcm2835_peri_write(paddrC, C.uint32_t(value))
}

func PeriWriteNB(paddr, value uint32) {
	/// Writes 32 bit value from a peripheral address without the write barrier
	/// You should only use this when your code has previously called bcm2835_peri_write()
	/// within the same peripheral, and no other peripheral access has occurred since.
	/// \param[in] paddr Physical address to read from. See BCM2835_GPIO_BASE etc.
	/// \param[in] value The 32 bit value to write
	/// \sa Physical Addresses
	/// extern void bcm2835_peri_write_nb(volatile uint32_t* paddr, uint32_t value);
    paddrC := (C.uint32_t)(paddr)
	C.bcm2835_peri_write_nb(paddrC, C.uint32_t(value))
}

func PeriSetBits(paddr, value, mask uint32) {
	/// Alters a number of bits in a 32 peripheral regsiter.
	/// It reads the current value and then alters the bits defines as 1 in mask,
	/// according to the bit value in value.
	/// All other bits that are 0 in the mask are unaffected.
	/// Use this to alter a subset of the bits in a register.
	/// The write is done twice, and is therefore always safe in terms of
	/// manual section 1.3 Peripheral access precautions for correct memory ordering
	/// \param[in] paddr Physical address to read from. See BCM2835_GPIO_BASE etc.
	/// \param[in] value The 32 bit value to write, masked in by mask.
	/// \param[in] mask Bitmask that defines the bits that will be altered in the register.
	/// \sa Physical Addresses
	/// extern void bcm2835_peri_set_bits(volatile uint32_t* paddr, uint32_t value, uint32_t mask);
	paddrC := C.uint32_t(paddr)
	C.bcm2835_peri_set_bits(paddrC, C.uint32_t(value), C.uint32_t(mask))
}
*/
/// @} // end of lowlevel

/// \defgroup gpio GPIO register access
/// These functions allow you to control the GPIO interface. You can set the
/// function of each GPIO pin, read the input state and set the output state.
/// @{

func GpioFSel(pin, mode uint8) {
	/// Sets the Function Select register for the given pin, which configures
	/// the pin as Input, Output or one of the 6 alternate functions.
	/// \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
	/// \param[in] mode Mode to set the pin to, one of BCM2835_GPIO_FSEL_* from \ref bcm2835FunctionSelect
	/// extern void bcm2835_gpio_fsel(uint8_t pin, uint8_t mode);
	C.bcm2835_gpio_fsel(C.uint8_t(pin), C.uint8_t(mode))
}

func GpioSet(pin uint8) {
	/// Sets the specified pin output to
	/// HIGH.
	/// \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
	/// \sa bcm2835_gpio_write()
	/// extern void bcm2835_gpio_set(uint8_t pin);
	C.bcm2835_gpio_set(C.uint8_t(pin))
}

func GpioClr(pin uint8) {
	/// Sets the specified pin output to
	/// LOW.
	/// \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
	/// \sa bcm2835_gpio_write()
	/// extern void bcm2835_gpio_clr(uint8_t pin);
	C.bcm2835_gpio_clr(C.uint8_t(pin))

}

func GpioSetMulti(mask uint32) {
	/// Sets any of the first 32 GPIO output pins specified in the mask to
	/// HIGH.
	/// \param[in] mask Mask of pins to affect. Use eg: (1 << RPI_GPIO_P1_03) | (1 << RPI_GPIO_P1_05)
	/// \sa bcm2835_gpio_write_multi()
	/// extern void bcm2835_gpio_set_multi(uint32_t mask);
	C.bcm2835_gpio_set_multi(C.uint32_t(mask))
}

func GpioClrMulti(mask uint32) {
	/// Sets any of the first 32 GPIO output pins specified in the mask to
	/// LOW.
	/// \param[in] mask Mask of pins to affect. Use eg: (1 << RPI_GPIO_P1_03) | (1 << RPI_GPIO_P1_05)
	/// \sa bcm2835_gpio_write_multi()
	/// extern void bcm2835_gpio_clr_multi(uint32_t mask);
	C.bcm2835_gpio_clr_multi(C.uint32_t(mask))
}

func GpioLev(pin uint8) uint8 {
	/// Reads the current level on the specified
	/// pin and returns either HIGH or LOW. Works whether or not the pin
	/// is an input or an output.
	/// \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
	/// \return the current level  either HIGH or LOW
	/// extern uint8_t bcm2835_gpio_lev(uint8_t pin);
	return uint8(C.bcm2835_gpio_lev(C.uint8_t(pin)))
}

func GpioEDS(pin uint8) uint8 {
	/// Event Detect Status.
	/// Tests whether the specified pin has detected a level or edge
	/// as requested by bcm2835_gpio_ren(), bcm2835_gpio_fen(), bcm2835_gpio_hen(),
	/// bcm2835_gpio_len(), bcm2835_gpio_aren(), bcm2835_gpio_afen().
	/// Clear the flag for a given pin by calling bcm2835_gpio_set_eds(pin);
	/// \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
	/// \return HIGH if the event detect status for the given pin is true.
	/// extern uint8_t bcm2835_gpio_eds(uint8_t pin);
	return uint8(C.bcm2835_gpio_eds(C.uint8_t(pin)))
}

func GpioSetEDS(pin uint8) {
	/// Sets the Event Detect Status register for a given pin to 1,
	/// which has the effect of clearing the flag. Use this afer seeing
	/// an Event Detect Status on the pin.
	/// \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
	/// extern void bcm2835_gpio_set_eds(uint8_t pin);
	C.bcm2835_gpio_set_eds(C.uint8_t(pin))
}

func GpioREN(pin uint8) {
	/// Enable Rising Edge Detect Enable for the specified pin.
	/// When a rising edge is detected, sets the appropriate pin in Event Detect Status.
	/// The GPRENn registers use
	/// synchronous edge detection. This means the input signal is sampled using the
	/// system clock and then it is looking for a ?011? pattern on the sampled signal. This
	/// has the effect of suppressing glitches.
	/// \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
	/// extern void bcm2835_gpio_ren(uint8_t pin);
	C.bcm2835_gpio_ren(C.uint8_t(pin))
}

func GpioClrREN(pin uint8) {
	/// Disable Rising Edge Detect Enable for the specified pin.
	/// \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
	/// extern void bcm2835_gpio_clr_ren(uint8_t pin);
	C.bcm2835_gpio_clr_ren(C.uint8_t(pin))
}

func GpioFEN(pin uint8) {
	/// Enable Falling Edge Detect Enable for the specified pin.
	/// When a falling edge is detected, sets the appropriate pin in Event Detect Status.
	/// The GPRENn registers use
	/// synchronous edge detection. This means the input signal is sampled using the
	/// system clock and then it is looking for a ?100? pattern on the sampled signal. This
	/// has the effect of suppressing glitches.
	/// \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
	/// extern void bcm2835_gpio_fen(uint8_t pin);
	C.bcm2835_gpio_fen(C.uint8_t(pin))
}

func GpioClrFEN(pin uint8) {
	/// Disable Falling Edge Detect Enable for the specified pin.
	/// \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
	/// extern void bcm2835_gpio_clr_fen(uint8_t pin);
	C.bcm2835_gpio_clr_fen(C.uint8_t(pin))
}

func GpioHEN(pin uint8) {
	/// Enable High Detect Enable for the specified pin.
	/// When a HIGH level is detected on the pin, sets the appropriate pin in Event Detect Status.
	/// \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
	/// extern void bcm2835_gpio_hen(uint8_t pin);
	C.bcm2835_gpio_hen(C.uint8_t(pin))
}

func GpioClrHEN(pin uint8) {
	/// Disable High Detect Enable for the specified pin.
	/// \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
	/// extern void bcm2835_gpio_clr_hen(uint8_t pin);
	C.bcm2835_gpio_clr_hen(C.uint8_t(pin))
}

func GpioLEN(pin uint8) {
	/// Enable Low Detect Enable for the specified pin.
	/// When a LOW level is detected on the pin, sets the appropriate pin in Event Detect Status.
	/// \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
	/// extern void bcm2835_gpio_len(uint8_t pin);
	C.bcm2835_gpio_len(C.uint8_t(pin))
}

func GpioClrLEN(pin uint8) {
	/// Disable Low Detect Enable for the specified pin.
	/// \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
	/// extern void bcm2835_gpio_clr_len(uint8_t pin);
	C.bcm2835_gpio_clr_len(C.uint8_t(pin))
}

func GpioAREN(pin uint8) {
	/// Enable Asynchronous Rising Edge Detect Enable for the specified pin.
	/// When a rising edge is detected, sets the appropriate pin in Event Detect Status.
	/// Asynchronous means the incoming signal is not sampled by the system clock. As such
	/// rising edges of very short duration can be detected.
	/// \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
	/// extern void bcm2835_gpio_aren(uint8_t pin);
	C.bcm2835_gpio_aren(C.uint8_t(pin))
}

func GpioClrAREN(pin uint8) {
	/// Disable Asynchronous Rising Edge Detect Enable for the specified pin.
	/// \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
	/// extern void bcm2835_gpio_clr_aren(uint8_t pin);
	C.bcm2835_gpio_clr_aren(C.uint8_t(pin))
}

func GpioAFEN(pin uint8) {
	/// Enable Asynchronous Falling Edge Detect Enable for the specified pin.
	/// When a falling edge is detected, sets the appropriate pin in Event Detect Status.
	/// Asynchronous means the incoming signal is not sampled by the system clock. As such
	/// falling edges of very short duration can be detected.
	/// \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
	/// extern void bcm2835_gpio_afen(uint8_t pin);
	C.bcm2835_gpio_afen(C.uint8_t(pin))
}

func GpioClrAFEN(pin uint8) {
	/// Disable Asynchronous Falling Edge Detect Enable for the specified pin.
	/// \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
	/// extern void bcm2835_gpio_clr_afen(uint8_t pin);
	C.bcm2835_gpio_clr_afen(C.uint8_t(pin))
}

func GpioPUD(pud uint8) {
	/// Sets the Pull-up/down register for the given pin. This is
	/// used with bcm2835_gpio_pudclk() to set the  Pull-up/down resistor for the given pin.
	/// However, it is usually more convenient to use bcm2835_gpio_set_pud().
	/// \param[in] pud The desired Pull-up/down mode. One of BCM2835_GPIO_PUD_* from bcm2835PUDControl
	/// \sa bcm2835_gpio_set_pud()
	/// extern void bcm2835_gpio_pud(uint8_t pud);
	C.bcm2835_gpio_pud(C.uint8_t(pud))
}

func GpioPUDClock(pin, on uint8) {
	/// Clocks the Pull-up/down value set earlier by bcm2835_gpio_pud() into the pin.
	/// \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
	/// \param[in] on HIGH to clock the value from bcm2835_gpio_pud() into the pin.
	/// LOW to remove the clock.
	/// \sa bcm2835_gpio_set_pud()
	/// extern void bcm2835_gpio_pudclk(uint8_t pin, uint8_t on);
	C.bcm2835_gpio_pudclk(C.uint8_t(pin), C.uint8_t(on))
}

func GpioPad(group uint8) uint32 {
	/// Reads and returns the Pad Control for the given GPIO group.
	/// \param[in] group The GPIO pad group number, one of BCM2835_PAD_GROUP_GPIO_*
	/// \return Mask of bits from BCM2835_PAD_* from \ref bcm2835PadGroup
	/// extern uint32_t bcm2835_gpio_pad(uint8_t group);
	return uint32(C.bcm2835_gpio_pad(C.uint8_t(group)))
}

func GpioSetPad(group uint8, control uint32) {
	/// Sets the Pad Control for the given GPIO group.
	/// \param[in] group The GPIO pad group number, one of BCM2835_PAD_GROUP_GPIO_*
	/// \param[in] control Mask of bits from BCM2835_PAD_* from \ref bcm2835PadGroup. Note
	/// that it is not necessary to include BCM2835_PAD_PASSWRD in the mask as this
	/// is automatically included.
	/// extern void bcm2835_gpio_set_pad(uint8_t group, uint32_t control);
	C.bcm2835_gpio_set_pad(C.uint8_t(group), C.uint32_t(control))
}

func Delay(millis uint) {
	/// Delays for the specified number of milliseconds.
	/// Uses nanosleep(), and therefore does not use CPU until the time is up.
	/// However, you are at the mercy of nanosleep(). From the manual for nanosleep():
	/// If the interval specified in req is not an exact multiple of the granularity
	/// underlying  clock  (see  time(7)),  then the interval will be
	/// rounded up to the next multiple. Furthermore, after the sleep completes,
	/// there may still be a delay before the CPU becomes free to once
	/// again execute the calling thread.
	/// \param[in] millis Delay in milliseconds
	/// extern void bcm2835_delay (unsigned int millis);
	C.bcm2835_delay(C.uint(millis))
}

func DelayMicroseconds(micros uint64) {
	/// Delays for the specified number of microseconds.
	/// Uses a combination of nanosleep() and a busy wait loop on the BCM2835 system timers,
	/// However, you are at the mercy of nanosleep(). From the manual for nanosleep():
	/// If the interval specified in req is not an exact multiple of the granularity
	/// underlying  clock  (see  time(7)),  then the interval will be
	/// rounded up to the next multiple. Furthermore, after the sleep completes,
	/// there may still be a delay before the CPU becomes free to once
	/// again execute the calling thread.
	/// For times less than about 450 microseconds, uses a busy wait on the System Timer.
	/// It is reported that a delay of 0 microseconds on RaspberryPi will in fact
	/// result in a delay of about 80 microseconds. Your mileage may vary.
	/// \param[in] micros Delay in microseconds
	/// extern void bcm2835_delayMicroseconds (uint64_t micros);
	C.bcm2835_delayMicroseconds(C.uint64_t(micros))
}

func GpioWrite(pin, on uint8) {
	/// Sets the output state of the specified pin
	/// \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
	/// \param[in] on HIGH sets the output to HIGH and LOW to LOW.
	/// extern void bcm2835_gpio_write(uint8_t pin, uint8_t on);
	C.bcm2835_gpio_write(C.uint8_t(pin), C.uint8_t(on))
}

func GpioWriteMulti(mask uint32, on uint8) {
	/// Sets any of the first 32 GPIO output pins specified in the mask to the state given by on
	/// \param[in] mask Mask of pins to affect. Use eg: (1 << RPI_GPIO_P1_03) | (1 << RPI_GPIO_P1_05)
	/// \param[in] on HIGH sets the output to HIGH and LOW to LOW.
	/// extern void bcm2835_gpio_write_multi(uint32_t mask, uint8_t on);
	C.bcm2835_gpio_write_multi(C.uint32_t(mask), C.uint8_t(on))
}

func GpioWriteMask(value, mask uint32) {
	/// Sets the first 32 GPIO output pins specified in the mask to the value given by value
	/// \param[in] value values required for each bit masked in by mask, eg: (1 << RPI_GPIO_P1_03) | (1 << RPI_GPIO_P1_05)
	/// \param[in] mask Mask of pins to affect. Use eg: (1 << RPI_GPIO_P1_03) | (1 << RPI_GPIO_P1_05)
	/// extern void bcm2835_gpio_write_mask(uint32_t value, uint32_t mask);
	C.bcm2835_gpio_write_mask(C.uint32_t(value), C.uint32_t(mask))
}

func GpioSetPUD(pin, pud uint8) {
	/// Sets the Pull-up/down mode for the specified pin. This is more convenient than
	/// clocking the mode in with bcm2835_gpio_pud() and bcm2835_gpio_pudclk().
	/// \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
	/// \param[in] pud The desired Pull-up/down mode. One of BCM2835_GPIO_PUD_* from bcm2835PUDControl
	/// extern void bcm2835_gpio_set_pud(uint8_t pin, uint8_t pud);
	C.bcm2835_gpio_set_pud(C.uint8_t(pin), C.uint8_t(pud))
}

/// \defgroup spi SPI access
/// These functions let you use SPI0 (Serial Peripheral Interface) to
/// interface with an external SPI device.
func SpiBegin() {
	/// Start SPI operations.
	/// Forces RPi SPI0 pins P1-19 (MOSI), P1-21 (MISO), P1-23 (CLK), P1-24 (CE0) and P1-26 (CE1)
	/// to alternate function ALT0, which enables those pins for SPI interface.
	/// You should call bcm2835_spi_end() when all SPI funcitons are complete to return the pins to
	/// their default functions
	/// \sa  bcm2835_spi_end()
	/// extern void bcm2835_spi_begin(void);
	C.bcm2835_spi_begin()
}

func SpiEnd() {
	/// End SPI operations.
	/// SPI0 pins P1-19 (MOSI), P1-21 (MISO), P1-23 (CLK), P1-24 (CE0) and P1-26 (CE1)
	/// are returned to their default INPUT behaviour.
	/// extern void bcm2835_spi_end(void);
	C.bcm2835_spi_end()
}

func SpiSetBitOrder(order uint8) {
	/// Sets the SPI bit order
	/// NOTE: has no effect. Not supported by SPI0.
	/// Defaults to
	/// \param[in] order The desired bit order, one of BCM2835_SPI_BIT_ORDER_*,
	/// see \ref bcm2835SPIBitOrder
	/// extern void bcm2835_spi_setBitOrder(uint8_t order);
	C.bcm2835_spi_setBitOrder(C.uint8_t(order))
}

func SpiSetClockDivider(divider uint16) {
	/// Sets the SPI clock divider and therefore the
	/// SPI clock speed.
	/// \param[in] divider The desired SPI clock divider, one of BCM2835_SPI_CLOCK_DIVIDER_*,
	/// see \ref bcm2835SPIClockDivider
	/// extern void bcm2835_spi_setClockDivider(uint16_t divider);
	C.bcm2835_spi_setClockDivider(C.uint16_t(divider))
}

func SpiSetDataMode(mode uint8) {
	/// Sets the SPI data mode
	/// Sets the clock polariy and phase
	/// \param[in] mode The desired data mode, one of BCM2835_SPI_MODE*,
	/// see \ref bcm2835SPIMode
	/// extern void bcm2835_spi_setDataMode(uint8_t mode);
	C.bcm2835_spi_setDataMode(C.uint8_t(mode))
}

func SpiChipSelect(cs uint8) {
	/// Sets the chip select pin(s)
	/// When an bcm2835_spi_transfer() is made, the selected pin(s) will be asserted during the
	/// transfer.
	/// \param[in] cs Specifies the CS pins(s) that are used to activate the desired slave.
	///   One of BCM2835_SPI_CS*, see \ref bcm2835SPIChipSelect
	/// extern void bcm2835_spi_chipSelect(uint8_t cs);
	C.bcm2835_spi_chipSelect(C.uint8_t(cs))
}

func SpiSetChipSelectPolarity(cs, active uint8) {
	/// Sets the chip select pin polarity for a given pin
	/// When an bcm2835_spi_transfer() occurs, the currently selected chip select pin(s)
	/// will be asserted to the
	/// value given by active. When transfers are not happening, the chip select pin(s)
	/// return to the complement (inactive) value.
	/// \param[in] cs The chip select pin to affect
	/// \param[in] active Whether the chip select pin is to be active HIGH
	/// extern void bcm2835_spi_setChipSelectPolarity(uint8_t cs, uint8_t active);
	C.bcm2835_spi_setChipSelectPolarity(C.uint8_t(cs), C.uint8_t(active))
}

func SpiTransfer(value uint8) uint8 {
	/// Transfers one byte to and from the currently selected SPI slave.
	/// Asserts the currently selected CS pins (as previously set by bcm2835_spi_chipSelect)
	/// during the transfer.
	/// Clocks the 8 bit value out on MOSI, and simultaneously clocks in data from MISO.
	/// Returns the read data byte from the slave.
	/// Uses polled transfer as per section 10.6.1 of the BCM 2835 ARM Peripherls manual
	/// \param[in] value The 8 bit data byte to write to MOSI
	/// \return The 8 bit byte simultaneously read from  MISO
	/// \sa bcm2835_spi_transfern()
	/// extern uint8_t bcm2835_spi_transfer(uint8_t value);
	return uint8(C.bcm2835_spi_transfer(C.uint8_t(value)))
}

func SpiTransfernb(tbuf []byte, rbuf []byte, len uint32) {
	/// Transfers any number of bytes to and from the currently selected SPI slave.
	/// Asserts the currently selected CS pins (as previously set by bcm2835_spi_chipSelect)
	/// during the transfer.
	/// Clocks the len 8 bit bytes out on MOSI, and simultaneously clocks in data from MISO.
	/// The data read read from the slave is placed into rbuf. rbuf must be at least len bytes long
	/// Uses polled transfer as per section 10.6.1 of the BCM 2835 ARM Peripherls manual
	/// \param[in] tbuf Buffer of bytes to send.
	/// \param[out] rbuf Received bytes will by put in this buffer
	/// \param[in] len Number of bytes in the tbuf buffer, and the number of bytes to send/received
	/// \sa bcm2835_spi_transfer()
	/// extern void bcm2835_spi_transfernb(char* tbuf, char* rbuf, uint32_t len);
	rbufC := (*C.char)(unsafe.Pointer(&rbuf[0]))
	tbufC := (*C.char)(unsafe.Pointer(&tbuf[0]))
	C.bcm2835_spi_transfernb(tbufC, rbufC, C.uint32_t(len))
}

func SpiTransfern(buf []byte, len uint32) {
	/// Transfers any number of bytes to and from the currently selected SPI slave
	/// using bcm2835_spi_transfernb.
	/// The returned data from the slave replaces the transmitted data in the buffer.
	/// \param[in,out] buf Buffer of bytes to send. Received bytes will replace the contents
	/// \param[in] len Number of bytes int eh buffer, and the number of bytes to send/received
	/// \sa bcm2835_spi_transfer()
	/// extern void bcm2835_spi_transfern(char* buf, uint32_t len);
	bufC := (*C.char)(unsafe.Pointer(&buf[0]))
	C.bcm2835_spi_transfern(bufC, C.uint32_t(len))
}

func SpiWritenb(buf []byte, len uint32) {
	/// Transfers any number of bytes to the currently selected SPI slave.
	/// Asserts the currently selected CS pins (as previously set by bcm2835_spi_chipSelect)
	/// during the transfer.
	/// \param[in] buf Buffer of bytes to send.
	/// \param[in] len Number of bytes in the tbuf buffer, and the number of bytes to send
	/// extern void bcm2835_spi_writenb(char* buf, uint32_t len);
	bufC := (*C.char)(unsafe.Pointer(&buf[0]))
	C.bcm2835_spi_writenb(bufC, C.uint32_t(len))
}

/// These functions let you use I2C (The Broadcom Serial Control bus with the Philips
/// I2C bus/interface version 2.1 January 2000.) to interface with an external I2C device.

func I2cBegin() {
	/// Start I2C operations.
	/// Forces RPi I2C pins P1-03 (SDA) and P1-05 (SCL)
	/// to alternate function ALT0, which enables those pins for I2C interface.
	/// You should call bcm2835_i2c_end() when all I2C functions are complete to return the pins to
	/// their default functions
	/// \sa  bcm2835_i2c_end()
	/// extern void bcm2835_i2c_begin(void);
	C.bcm2835_i2c_begin()
}

func I2cEnd() {
	/// End I2C operations.
	/// I2C pins P1-03 (SDA) and P1-05 (SCL)
	/// are returned to their default INPUT behaviour.
	/// extern void bcm2835_i2c_end(void);
	C.bcm2835_i2c_end()
}

func I2cSetSlaveAddress(addr uint8) {
	/// Sets the I2C slave address.
	/// \param[in] addr The I2C slave address.
	/// extern void bcm2835_i2c_setSlaveAddress(uint8_t addr);
	C.bcm2835_i2c_setSlaveAddress(C.uint8_t(addr))
}

func I2cSetClockDivider(divider uint16) {
	/// Sets the I2C clock divider and therefore the I2C clock speed.
	/// \param[in] divider The desired I2C clock divider, one of BCM2835_I2C_CLOCK_DIVIDER_*,
	/// see \ref bcm2835I2CClockDivider
	/// extern void bcm2835_i2c_setClockDivider(uint16_t divider);
	C.bcm2835_i2c_setClockDivider(C.uint16_t(divider))
}

func I2cSetBaudrate(baudrate uint32) {
	/// Sets the I2C clock divider by converting the baudrate parameter to
	/// the equivalent I2C clock divider. ( see \sa bcm2835_i2c_setClockDivider)
	/// For the I2C standard 100khz you would set baudrate to 100000
	/// The use of baudrate corresponds to its use in the I2C kernel device
	/// driver. (Of course, bcm2835 has nothing to do with the kernel driver)
	/// extern void bcm2835_i2c_set_baudrate(uint32_t baudrate);
	C.bcm2835_i2c_set_baudrate(C.uint32_t(baudrate))
}

func I2cWrite(buf []byte, len uint32) uint8 {
	/// Transfers any number of bytes to the currently selected I2C slave.
	/// (as previously set by \sa bcm2835_i2c_setSlaveAddress)
	/// \param[in] buf Buffer of bytes to send.
	/// \param[in] len Number of bytes in the buf buffer, and the number of bytes to send.
	/// \return reason see \ref bcm2835I2CReasonCodes
	/// extern uint8_t bcm2835_i2c_write(const char * buf, uint32_t len);
	bufC := (*C.char)(unsafe.Pointer(&buf[0]))
	return uint8(C.bcm2835_i2c_write(bufC, C.uint32_t(len)))
}

func I2cRead(buf []byte, len uint32) uint8 {

	/// Transfers any number of bytes from the currently selected I2C slave.
	/// (as previously set by \sa bcm2835_i2c_setSlaveAddress)
	/// \param[in] buf Buffer of bytes to receive.
	/// \param[in] len Number of bytes in the buf buffer, and the number of bytes to received.
	/// \return reason see \ref bcm2835I2CReasonCodes
	/// extern uint8_t bcm2835_i2c_read(char* buf, uint32_t len);
	bufC := (*C.char)(unsafe.Pointer(&buf[0]))
	return uint8(C.bcm2835_i2c_read(bufC, C.uint32_t(len)))
}

func I2cReadRegisterRS(regaddr, buf []byte, len uint32) uint8 {
	/// Allows reading from I2C slaves that require a repeated start (without any prior stop)
	/// to read after the required slave register has been set. For example, the popular
	/// MPL3115A2 pressure and temperature sensor. Note that your device must support or
	/// require this mode. If your device does not require this mode then the standard
	/// combined:
	///   \sa bcm2835_i2c_write
	///   \sa bcm2835_i2c_read
	/// are a better choice.
	/// Will read from the slave previously set by \sa bcm2835_i2c_setSlaveAddress
	/// \param[in] regaddr Buffer containing the slave register you wish to read from.
	/// \param[in] buf Buffer of bytes to receive.
	/// \param[in] len Number of bytes in the buf buffer, and the number of bytes to received.
	/// \return reason see \ref bcm2835I2CReasonCodes
	/// extern uint8_t bcm2835_i2c_read_register_rs(char* regaddr, char* buf, uint32_t len);
	regaddrC := (*C.char)(unsafe.Pointer(&regaddr[0]))
	bufC := (*C.char)(unsafe.Pointer(&buf[0]))
	return uint8(C.bcm2835_i2c_read_register_rs(regaddrC, bufC, C.uint32_t(len)))
}

func I2cWriteReadRS(cmds []byte, cmds_len uint32, buf []byte, buf_len uint32) uint8 {
	/// Allows sending an arbitrary number of bytes to I2C slaves before issuing a repeated
	/// start (with no prior stop) and reading a response.
	/// Necessary for devices that require such behavior, such as the MLX90620.
	/// Will write to and read from the slave previously set by \sa bcm2835_i2c_setSlaveAddress
	/// \param[in] cmds Buffer containing the bytes to send before the repeated start condition.
	/// \param[in] cmds_len Number of bytes to send from cmds buffer
	/// \param[in] buf Buffer of bytes to receive.
	/// \param[in] buf_len Number of bytes to receive in the buf buffer.
	/// \return reason see \ref bcm2835I2CReasonCodes
	/// extern uint8_t bcm2835_i2c_write_read_rs(char* cmds, uint32_t cmds_len, char* buf, uint32_t buf_len);
	cmdsC := (*C.char)(unsafe.Pointer(&cmds[0]))
	bufC := (*C.char)(unsafe.Pointer(&buf[0]))
	return uint8(C.bcm2835_i2c_write_read_rs(cmdsC, C.uint32_t(cmds_len), bufC, C.uint32_t(buf_len)))
}

/// @}

/// \defgroup st System Timer access
/// Allows access to and delays using the System Timer Counter.
/// @{

func StRead() uint64 {
	/// Read the System Timer Counter register.
	/// \return the value read from the System Timer Counter Lower 32 bits register
	/// extern uint64_t bcm2835_st_read(void);
	return uint64(C.bcm2835_st_read())
}

func StDelay(offset_micros, micros uint64) {
	/// Delays for the specified number of microseconds with offset.
	/// \param[in] offset_micros Offset in microseconds
	/// \param[in] micros Delay in microseconds
	/// extern void bcm2835_st_delay(uint64_t offset_micros, uint64_t micros);
	C.bcm2835_st_delay(C.uint64_t(offset_micros), C.uint64_t(micros))
}

/// @}

/// \defgroup pwm Pulse Width Modulation
/// Allows control of 2 independent PWM channels. A limited subset of GPIO pins
/// can be connected to one of these 2 channels, allowing PWM control of GPIO pins.
/// You have to set the desired pin into a particular Alt Fun to PWM output. See the PWM
/// documentation on the Main Page.
/// @{

func PwmSetClock(divisor uint32) {
	/// Sets the PWM clock divisor,
	/// to control the basic PWM pulse widths.
	/// \param[in] divisor Divides the basic 19.2MHz PWM clock. You can use one of the common
	/// values BCM2835_PWM_CLOCK_DIVIDER_* in \ref bcm2835PWMClockDivider
	/// extern void bcm2835_pwm_set_clock(uint32_t divisor);
	C.bcm2835_pwm_set_clock(C.uint32_t(divisor))
}

func PwmSetMode(channel, markspace, enabled uint8) {
	/// Sets the mode of the given PWM channel,
	/// allowing you to control the PWM mode and enable/disable that channel
	/// \param[in] channel The PWM channel. 0 or 1.
	/// \param[in] markspace Set true if you want Mark-Space mode. 0 for Balanced mode.
	/// \param[in] enabled Set true to enable this channel and produce PWM pulses.
	/// extern void bcm2835_pwm_set_mode(uint8_t channel, uint8_t markspace, uint8_t enabled);
	C.bcm2835_pwm_set_mode(C.uint8_t(channel), C.uint8_t(markspace), C.uint8_t(enabled))
}

func PwmSetRange(channel uint8, pwmRange uint32) {
	/// Sets the maximum range of the PWM output.
	/// The data value can vary between 0 and this range to control PWM output
	/// \param[in] channel The PWM channel. 0 or 1.
	/// \param[in] range The maximum value permitted for DATA.
	/// extern void bcm2835_pwm_set_range(uint8_t channel, uint32_t range);
	C.bcm2835_pwm_set_range(C.uint8_t(channel), C.uint32_t(pwmRange))
}

func PwmSetData(channel uint8, data uint32) {
	/// Sets the PWM pulse ratio to emit to DATA/RANGE,
	/// where RANGE is set by bcm2835_pwm_set_range().
	/// \param[in] channel The PWM channel. 0 or 1.
	/// \param[in] data Controls the PWM output ratio as a fraction of the range.
	///  Can vary from 0 to RANGE.
	/// extern void bcm2835_pwm_set_data(uint8_t channel, uint32_t data);
	C.bcm2835_pwm_set_data(C.uint8_t(channel), C.uint32_t(data))
}
