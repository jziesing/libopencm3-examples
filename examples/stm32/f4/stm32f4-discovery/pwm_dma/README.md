# README

This example demonstrates the use of timers and pwm on on LED on the eval board.

It's intended for the ST STM32F4DISCOVERY eval board. 
The program will pulse one led on the eval board. It uses dma to transfer a block of data to the the timer oc  register.  The timer oc register is in pwm mode so it sends pwm to the af of the gpio.

## Board connections

*none required*
