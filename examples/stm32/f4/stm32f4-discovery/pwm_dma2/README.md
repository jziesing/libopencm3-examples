# README

This example demonstrates the use of timers and pwm on on LED on the eval board.

It's intended for the ST STM32F4DISCOVERY eval board. 
The program will pulse one led on the eval board. It uses dma to transfer a block of data to the the timer oc  register.  The timer oc register is in pwm mode so it sends pwm to the af of the gpio.  This differs from the first pwm_led example becuase it uses a dma interrupt on both half transfer and transfer complete to update values in the block of data transfered.  It updates the values fairley screwey, but it makes the led pulse.  It updates two values from 0-200 and then fills the data block with the value of the two values multiplied.  This makes values between 0 and 40000 roughly, and those values are in the block of data that the dma controller transfers to the peripheral.  So each interrupt after the first half is transferred we update a new multiplier, usually getting larger from the previous or smaller, then compute and update new values for the first half of the buffer in memory that the dma controller transfers.  Then the transfer complete interrupt is triggered and we compute and update the second half of the memory buffer but we don't need to increment or deincrement the multiplies since we did so in the first half of data, and the data is similar.

## Board connections

*none required*
