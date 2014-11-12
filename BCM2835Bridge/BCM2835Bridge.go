// BCM2835Bridge
// package BCM2835Bridge
// #cgo LDFLAGS: -l bcm2835 -static
package main
import "C"
import (
	"bcm2835"
    "fmt"
)
func main() {
	fmt.Println("SPI loop back!")
	err := bcm2835.Init() // Initialize the library
	if err != nil {
		fmt.Println(err)
		return
	}
	bcm2835.SpiBegin()
	bcm2835.SpiSetBitOrder(bcm2835.BCM2835_SPI_BIT_ORDER_MSBFIRST)       // The default
	bcm2835.SpiSetDataMode(bcm2835.BCM2835_SPI_MODE0 )                   // The default
	bcm2835.SpiSetClockDivider(bcm2835.BCM2835_SPI_CLOCK_DIVIDER_65536 ) // The default
	bcm2835.SpiChipSelect(bcm2835.BCM2835_SPI_CS0 )                      // The default
	bcm2835.SpiSetChipSelectPolarity(bcm2835.BCM2835_SPI_CS0, bcm2835.LOW)    // the default
	//~ // Send a byte to the slave and simultaneously read a byte back from the slave
	//~ // If you tie MISO to MOSI, you should read back what was sent
	data := bcm2835.SpiTransfer(0x23)
	fmt.Printf("Read from SPI: %#X\n", data)
	bcm2835.SpiEnd()
	bcm2835.Close()
}
