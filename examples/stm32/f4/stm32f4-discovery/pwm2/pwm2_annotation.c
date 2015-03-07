
/*
* This file is part of the libopencm3 project.
*
* Copyright (C) 2015 Piotr Esden-Tempski <piotr@esden.net>
* Copyright (C) 2015 Jack Ziesing <jziesing@gmail.com>
*
* This library is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License for more details.
*/


/*
* NOTES:
* ----------------------------------------------------------------
* ----------------------------------------------------------------
* Note:  this version is the same as pwm2.c but with many more comments for explanation, mainly for me, 
* but also to check my understanding with those experts on libopencm3 and the stm32 (Piotr).
* 
* Overview: This program pulses leds on stm32 f4 discovery board with pwm, a timer, and an inturrup.
*
* Steps:
*   We start by setting up out clock to 3.3 volts at 168mhz.  This is the main rcc clock that drives the chip.
* 
*   Next we set up some gpio so our cpu has some input.  In any case of I/O with a microcontrollers core
*   we must enable gpio on the chip to either a peripheral, or some other source.  So we do so in gpio_setup.
*   In this method we first choose gpio d, and enable it by calling rcc_periph_clock_enable, I think this enables
*   the clock on all pins of gpio d, not positive, still understanding gpio details.  Next we set the mode of the gpio.
*   this is a huge step, as for we define how we will use gpio.  So first specify port d.  This is good becuase this has a clock enabled.
*   Then we choose the mode,  There are several modes of gpio, don't really understand all of them deeply, but I think i get a general idea.
*   For this we need the alternate function mode enabled because this allows a peripheral like pwm, to use it.  Then we se the pull up pull down 
*   to none.  This is just a way to set the pin high or low on the start.. I believe.  I feel like there is good uses for the PUPD but I am not sure what specific uses would be ideal.
*   Then we tell it what pins to hook up to gpio d.  For this example we use gpios 12-15 which are hooked up to led on the discovery board.  
*   Lastly we call a function to set the alternate function, because there are a bunch to deal with.  We set all gpio pins to the same alternate function, af2.
*   
*   Now that the clock and gpio modes are set, we ca setup our timer to inturrupt and automattically update the register which the dutycycle is defined.  
*   We use timer 4 because the discovery boards leds are only attached to timer 4, see stm32 f4 discovery board data sheet, page 30.
*   The other setting in the timer mode are more complicated.  The TIM_CR1_CKD_CK_INT is a way to set the timer divisor, but I believe the CK_INT means the value is none, so no divsion of clock.
*   The TIM_CR1_CMS_EDGE sets the center alignment mode to an edge, or the start or end of the timer.
*   Finally we set the mode to count up, rather than down.  
*   Question: With all the params or the mode setup for the timer is the CR1 as in TIM_CR1_CKD_CK_INT or TIM_CR1_CMS_EDGE.  I see more CR's or compare registers, i think? but why would one use cr1 over cr2.
*   Need to read more up on this, but hard to find explanation on the what different CR's do.  Maybe it just allows more functions, you just need to be specific in that you use and consistant. 
*   
*   After the mode setup, we need to set the timer up a bit more.
*
*   We set a prescalar of 0, because we want to use the normal speed of out rcc clock, 168mhz
*   Then we set it to continuous mode, so the timer continues to inturrupt, as long as the program is running.
*   Then we set the period to 65535, which is the larges 16 bit value.  Since timer4 is a 16 bit timer. 
*   This is the largest period we are possibly allowed to have, so we take it all.  But I believe you could change this to something else, but it will affect the duty cycle, making the led turn on and off in faster intervals.
*   
*   Now we want to setup the output compare registers for each channel on timer4.  We first disable and clear some things to properly set them up.
*   Next we need to enable the preload so the output compare of each channel of the timer with get automatically updated.
*   We also set the speed to slow mode, just a slower timer I belive.
*   Next we set the mode of the OC's to ocm_pwm1, a mode for pwm peripheral.  
*   Next we set the OC polarity high for all channels.  I guess this makes it update faster, not too sure.
*   The final thing we do before re-enabling OC, is set the OC value.  This is the value that sets the duty cycle for each gpio.
*   We set it to dim or short duty cycle of 500.  This out of 65535, so the led is on for a short time making it dim.
*   Then we re-enable output,preload, and counter and 
*
*   Enable the interrupt.
*   We set up the inturrupt to act on the ARR, or auto relod register.  This means that whenever the 65535 period ends, our interrup is called, or it might be when it starts, either way, not in the middle, and not dependent on a compare register.
*   So the inturrupt occurs at the same point each time.
*
*   In the inturrupt, we look for the update flag getting set, when it is set we know to do something.
*   First we just clear the flag
*   Then we decide what our new duty cycle value will be, by seeing if it is in the periods range? and if its getting brighter or not?
*   Then we set the output compare register of the timer to that value so the led gets brighter or dimmer incrementally.   Lastly we reset the update flag so the inturrupt with occur again once the pperiod ends again.  
*   Using the update we only get inturrupts every 65535, but then we set the oc value.  Our hightest value is 65000, so if the OC is set at the highest or brightest value,
*   then the pwm will pulse only once for a long time before the interrupt interrupts.  In dimmer cases the led will have a small OC and then it will turn on and off several times within the period of the timer.
*   Once the new OC is set, the led with pwm with this new duty cycle blink at that rate until the interrupt interrupts again and a new duty cycle is chosen and set.
*
*   Question: I am curious why we don't need to reset the update interrupt flag after clearing it?  Is it that once we enable_irq() the interrupt will always occur and set the flag if it is not set when the interrupt interrupts?
*
*
*
*/
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencmsis/core_cm3.h>
/*
 * global variables:
 *   -increment
 *   -descending 
 *
 *  These variable are used to pulse the led's bright and dim. 
 *  Descending is a simple flag to determine if the increment should increase or decrease.
 *  Increment is the 16 bit holder of the value of our duty cycle, which we change at earch inturrupt.
 */
uint16_t increment;
uint16_t descending;

static void clock_setup(void)
{
    /*
     * setup main clock at 3.3 volts and 168 mhz
     */
    rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);
}

static void gpio_setup(void)
{
    /* Enable clock on GPIOD pin. */
    rcc_periph_clock_enable(RCC_GPIOD);
    /*
     * Setup gpio mode:
     *   -Set port D which has clock
     *   -Set the mode to the alternate function enabled for pwm
     *   -Disable or set the pull up pull down register to none
     *   -Enable it on gpios 12, 13, 14, and 15.
     */
    gpio_mode_setup(GPIOD, GPIO_MODE_AF,
                    GPIO_PUPD_NONE, GPIO12 | GPIO13 | GPIO14 | GPIO15);
    /* Set GPIO12, GPIO13, GPIO14, and GPIO15 (in GPIO port D) alternate function
     * to allow peripheral pwm to have use of gpio pins
     */
    gpio_set_af(GPIOD, GPIO_AF2, GPIO12 | GPIO13 | GPIO14 | GPIO15);
}

static void tim_setup(void)
{
    /*
     * Enable rcc clock on TIM4
     * enable nested vector inturrupt
     * reset timer from previous value in registers
     */
    rcc_periph_clock_enable(RCC_TIM4);
    nvic_enable_irq(NVIC_TIM4_IRQ);
    timer_reset(TIM4);
    /* Timer global mode:
    * - No divider
    * - Alignment edge
    * - Direction up
    */
    timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT,
                   TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    /*
     * No prescaler, normal clock will suffice.
     * enable continuous clock
     * set period to max 16 bit value or 65535
     */
    timer_set_prescaler(TIM4, 0);
    timer_continuous_mode(TIM4);
    timer_set_period(TIM4, 65535);
    /*
     * disable output compare output on all 4 channels
     */
    timer_disable_oc_output(TIM4, TIM_OC1);
    timer_disable_oc_output(TIM4, TIM_OC2);
    timer_disable_oc_output(TIM4, TIM_OC3);
    timer_disable_oc_output(TIM4, TIM_OC4);
    /*
     * clear output compare registers on all 4 channels
     */
    timer_disable_oc_clear(TIM4, TIM_OC1);
    timer_disable_oc_clear(TIM4, TIM_OC2);
    timer_disable_oc_clear(TIM4, TIM_OC3);
    timer_disable_oc_clear(TIM4, TIM_OC4);
    /*
     * enable output compare preloading or auto updating
     */
    timer_enable_oc_preload(TIM4, TIM_OC1);
    timer_enable_oc_preload(TIM4, TIM_OC2);
    timer_enable_oc_preload(TIM4, TIM_OC3);
    timer_enable_oc_preload(TIM4, TIM_OC4);
    /*
     * set the output compare to slow mode in all 4 channels
     */
    timer_set_oc_slow_mode(TIM4, TIM_OC1);
    timer_set_oc_slow_mode(TIM4, TIM_OC2);
    timer_set_oc_slow_mode(TIM4, TIM_OC3);
    timer_set_oc_slow_mode(TIM4, TIM_OC4);
    /*
     * set the output compare to pwm1 for all for channels
     */
    timer_set_oc_mode(TIM4, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_mode(TIM4, TIM_OC2, TIM_OCM_PWM1);
    timer_set_oc_mode(TIM4, TIM_OC3, TIM_OCM_PWM1);
    timer_set_oc_mode(TIM4, TIM_OC4, TIM_OCM_PWM1);
    /*
     * disable output compare output on all 4 channels
     */
    timer_set_oc_polarity_high(TIM4, TIM_OC1);
    timer_set_oc_polarity_high(TIM4, TIM_OC2);
    timer_set_oc_polarity_high(TIM4, TIM_OC3);
    timer_set_oc_polarity_high(TIM4, TIM_OC4);
    /*
     * set output compare value to 500, dim or short duty cycle
     */
    timer_set_oc_value(TIM4, TIM_OC1, 500);
    timer_set_oc_value(TIM4, TIM_OC2, 500);
    timer_set_oc_value(TIM4, TIM_OC3, 500);
    timer_set_oc_value(TIM4, TIM_OC4, 500);
    /*
     * enable output compare output on all 4 channels
     */
    timer_enable_oc_output(TIM4, TIM_OC1);
    timer_enable_oc_output(TIM4, TIM_OC2);
    timer_enable_oc_output(TIM4, TIM_OC3);
    timer_enable_oc_output(TIM4, TIM_OC4);
    /*
     * enable the preload
     * enable the counter
     * enable the inturrupt on the auto update register
     */
    timer_enable_preload(TIM4);
    timer_enable_counter(TIM4);
    timer_enable_irq(TIM4, TIM_DIER_UIE);
}

void tim4_isr(void)
{
    if (timer_get_flag(TIM4, TIM_SR_UIF)) {
        /* clear update flag */
        timer_clear_flag(TIM4, TIM_SR_UIF);
        /*
         * simple logic to keep oc value increasing and decreasing
         * changing value increment get updated by increases or
         * decreases the speed of the how fast the leds get bright and dim
         */
        if (descending == 0)
        {
            if (increment >= 65000)
            {
                descending = 1;
                increment -= 50;
            } else {
                increment += 50;
            }
        } else if (descending == 1)
        {
            if (increment <= 500)
            {
                descending = 0;
                increment += 50;
            } else {
                increment -= 50;
            }

        }
        /*
         * set new incremented value for oc register
         * which is setup for pwm by the alternate function
         */
        timer_set_oc_value(TIM4, TIM_OC1, increment);
        timer_set_oc_value(TIM4, TIM_OC2, increment);
        timer_set_oc_value(TIM4, TIM_OC3, increment);
        timer_set_oc_value(TIM4, TIM_OC4, increment);
    }
}

int main(void)
{
    /*
     * initialize to increasing duty cycle
     * setup the clock, gpio, and timer.. in that order
     */
    descending = 0;
    clock_setup();
    gpio_setup();
    tim_setup();
    /*
     * just wait for the inturrupt..
     */
    while (1) {
        __WFI();
    }
    return 0;
}
