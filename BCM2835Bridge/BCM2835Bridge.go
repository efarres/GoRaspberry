// BCM2835Bridge
// package BCM2835Bridge
// #cgo LDFLAGS: -l bcm2835 -static
package main

import (
	"C"
	"bcm2835"
	"fmt"
	"log"
)

func main() {
	fmt.Println("SPI loop back!")
	// Initialize the library
	if err := bcm2835.Init(); err != nil {
		log.Fatal(err)
	}
	// continues on the next slide

	// SPLIT OMIT
	bcm2835.SpiBegin()
	defer bcm2835.Close()
	defer bcm2835.SpiEnd()

	// All defaults
	bcm2835.SpiSetBitOrder(bcm2835.BCM2835_SPI_BIT_ORDER_MSBFIRST)
	bcm2835.SpiSetDataMode(bcm2835.BCM2835_SPI_MODE0)
	bcm2835.SpiSetClockDivider(bcm2835.BCM2835_SPI_CLOCK_DIVIDER_65536)
	bcm2835.SpiChipSelect(bcm2835.BCM2835_SPI_CS0)
	bcm2835.SpiSetChipSelectPolarity(bcm2835.BCM2835_SPI_CS0, bcm2835.LOW)

	// Send a byte to the slave and simultaneously read a byte back from the slave
	// If you tie MISO to MOSI, you should read back what was sent
	fmt.Printf("Read from SPI: %#X\n", bcm2835.SpiTransfer(0x23))
}
