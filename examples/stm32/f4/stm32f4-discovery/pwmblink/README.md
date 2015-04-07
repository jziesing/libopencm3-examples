# README

This example demonstrates the use of timers and pwm on on LED on the eval board.

It's intended for the ST STM32F4DISCOVERY eval board. 

This program will light one LED up on the eval board.  It trigger pwm by the initial value set up in the timers
output compare register.  For this example we use a large value to make the led bright.  The duty cycle is defined by the oc value / timer period.  

## Board connections

*none required*
