// BCM2835Bridge
// package BCM2835Bridge
// #cgo LDFLAGS: -l bcm2835 -static

package bcm2835

import (
    "fmt"
    "testing"
    "time"
)


func TestSPI(t *testing.T) {
    // Test requires root permission
    // To run the test 
    // sudo /usr/local/go/bin/go test

	err := Init() // Initialize the library
	if err != nil {
		fmt.Println(err)
		t.Log(err)
	}
	SpiBegin()
	SpiSetBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST)      // The default
	SpiSetDataMode(BCM2835_SPI_MODE0 )                   // The default
	SpiSetClockDivider(BCM2835_SPI_CLOCK_DIVIDER_65536 ) // The default
	SpiChipSelect(BCM2835_SPI_CS0 )                      // The default
	SpiSetChipSelectPolarity(BCM2835_SPI_CS0, LOW)    // the default
	//~ // Send a byte to the slave and simultaneously read a byte back from the slave
	//~ // If you tie MISO to MOSI, you should read back what was sent
	data := SpiTransfer(0x23)
	fmt.Printf("Read from SPI: %#X\n", data)
	SpiEnd()
	Close()

}


func TestI2C(t *testing.T) {
    // Test requires root permission
    // To run the test 
    // sudo /usr/local/go/bin/go test
    data := []byte{'g','o','l','a','n','g'}
	fmt.Printf("Buffer:%v \n", data)
        
	err := Init() // Initialize the library
	if err != nil {
		fmt.Println(err)
		t.Log(err)
	}
	I2cBegin()
    I2cSetSlaveAddress(0x48)
    I2cSetClockDivider(BCM2835_I2C_CLOCK_DIVIDER_2500)   
    
    err1 := I2cRead(data, 6) 
    if err1 == BCM2835_I2C_REASON_ERROR_NACK {
        fmt.Println("NACK")
        t.Log(err1)
    }

	fmt.Printf("Read from I2C:%v \n", data)
	I2cEnd()
    
	Close()

}
func TestGPIO(t *testing.T) {
    // Test requires root permission
    // To run the test 
    // sudo /usr/local/go/bin/go test

	err := Init() // Initialize the library
	if err != nil {
		fmt.Println(err)
		t.Log(err)
	}
    // Blinks on RPi Plug P1 pin 11 (which is GPIO pin 17)
    // #define PIN RPI_GPIO_P1_11
    // Set the pin to be an output
    GpioFSel(RPI_GPIO_P1_11, BCM2835_GPIO_FSEL_OUTP) 

    // Blink
    for i:=0; i < 10; i++ {
        // Turn it off STATUS_LED_N
        GpioWrite(RPI_GPIO_P1_11, HIGH);
        
        // wait a bit, millis
        time.Sleep(500 * time.Millisecond)
        // Delay(500)
        
        // turn it on STATUS_LED_N
        GpioWrite(RPI_GPIO_P1_11, LOW);
        
        // wait a bit, millis
        //Delay(1500)
        time.Sleep(1500 * time.Millisecond)

    }  

    
	Close()

}


// PWM output on RPi Plug P1 pin 12 (which is GPIO pin 18)
// in alt fun 5.
// Note that this is the _only_ PWM pin available on the RPi IO headers
// #define PIN RPI_GPIO_P1_12
// and it is controlled by PWM channel 0
// #define PWM_CHANNEL 0
// This controls the max range of the PWM signal
// #define RANGE 1024
func TestPWM(t *testing.T) {
    // Test requires root permission
    // To run the test 
    // sudo /usr/local/go/bin/go test

	err := Init() // Initialize the library
	if err != nil {
		fmt.Println(err)
		t.Log(err)
	}

    // Set the output pin to Alt Fun 5, to allow PWM channel 0 to be output there
    GpioFSel(RPI_GPIO_P1_11, BCM2835_GPIO_FSEL_ALT5)

    // Clock divider is set to 16.
    // With a divider of 16 and a RANGE of 1024, in MARKSPACE mode,
    // the pulse repetition frequency will be
    // 1.2MHz/1024 = 1171.875Hz, suitable for driving a DC motor with PWM
    PwmSetClock(BCM2835_PWM_CLOCK_DIVIDER_16)
    PwmSetMode(0, 1, 1)
    PwmSetRange(0, 1024)

    // Vary the PWM m/s ratio between 1/RANGE and (RANGE-1)/RANGE
    // direction := 1
    data := uint32(1)

    PwmSetData(0, data)
    // wait a bit, millis
    //Delay(1500)
    time.Sleep(15000 * time.Millisecond)

    Close()
}


