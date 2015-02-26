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
*
* You should have received a copy of the GNU Lesser General Public License
* along with this library. If not, see <http://www.gnu.org/licenses/>.
*/
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencmsis/core_cm3.h>

static void clock_setup(void)
{
    rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);
}

static void gpio_setup(void)
{
    /* Enable GPIOD clock. */
    rcc_periph_clock_enable(RCC_GPIOD);
    /* Set GPIO12 (in GPIO port C) to 'output push-pull'. */
    gpio_mode_setup(GPIOD, GPIO_MODE_AF,
                    GPIO_PUPD_NONE, GPIO12);
    /* Set GPIO12 (in GPIO port C) alternate function */
    gpio_set_af(GPIOD, GPIO_AF2, GPIO12);
}

static void tim_setup(void)
{
    /* Enable TIM4 clock. */
    rcc_periph_clock_enable(RCC_TIM4);
    /* Reset TIM4 peripheral. */
    timer_reset(TIM4);
    /* Timer global mode:
    * - No divider
    * - Alignment edge
    * - Direction up
    */
    timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT,
                   TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM4, 0);
    /* Enable preload. */
    timer_disable_preload(TIM4);
    /* Continous mode. */
    timer_continuous_mode(TIM4);
    /* Period (36kHz). */
    timer_set_period(TIM4, 65535);
    /* Disable outputs. */
    timer_disable_oc_output(TIM4, TIM_OC1);
    timer_disable_oc_output(TIM4, TIM_OC2);
    timer_disable_oc_output(TIM4, TIM_OC3);
    timer_disable_oc_output(TIM4, TIM_OC4);
    
    /* -- OC1 configuration -- */
    /* Configure global mode of line 1. */
    timer_disable_oc_clear(TIM4, TIM_OC1);
    timer_enable_oc_preload(TIM4, TIM_OC1);
    timer_set_oc_slow_mode(TIM4, TIM_OC1);
    timer_set_oc_mode(TIM4, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_polarity_high(TIM4, TIM_OC1);

    /* Set the capture compare value for OC1. */
    timer_set_oc_value(TIM4, TIM_OC1, 65534);    
    timer_enable_oc_output(TIM4, TIM_OC1);
    timer_enable_preload(TIM4);
    /* Counter enable. */
    timer_enable_counter(TIM4);
}

int main(void)
{
    clock_setup();
    gpio_setup();
    tim_setup();

    while (1) { 
        __asm__("nop");
    }
    return 0;
}
