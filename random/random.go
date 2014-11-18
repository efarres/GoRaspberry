package main

/*
#include <stdint.h>
#include <stdlib.h>
*/
import (
	"C"
	"fmt"
)

func random() int { return int(C.random()) }
func seed(i int)  { C.srandom(C.uint(i)) }

func main() {
	seed(42)
	fmt.Println(random())
}
