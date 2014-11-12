package main
/*
#include <stdint.h>
#include <stdlib.h>
*/
import "C"
import (
	"fmt"
)
func Random() int {
	return int(C.random())
}
func Seed(i int) {
	C.srandom(C.uint(i))
}
func main() {
	Seed(42)
	fmt.Println(Random())
	fmt.Println("Well done!")
}
