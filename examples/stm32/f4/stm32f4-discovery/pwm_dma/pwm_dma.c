
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
/*
 * global variables:
 *   -increment = pwm duty cycle value
 *   -descending = flag for increasing or decreasing increment
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
    /* Set GPIO12, GPIO13, GPIO14, and GPIO15 (in GPIO port D) alternate function */
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
    if (timer_get_flag(TIM4, TIM_SR_CC1IF)) {
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

void tim4_isr(void)
{
    TIM_CR1(TIM4) = 0;
    TIM_DIER(TIM4) = 0;
    TIM_SR(TIM4) = 0;
    ws2812_dma_disable();
    ws2812_state = ws2812_state_idle;
}

void ws2812_dma_init(void)
{
    rcc_periph_clock_enable(RCC_DMA1);

    TIM_CCER(TIM4) = (TIM_CCER_CC1E);
    TIM_CCMR1(TIM4) = (TIM_CCMR1_OC1M_PWM1 |
                       TIM_CCMR1_OC1PE);
    //potential jack fixes8
    DMA_SCR(DMA1, DMA_CHANNEL7) = 0;
    DMA_SNDTR(DMA1, DMA_CHANNEL7) = WS2812_BUFFER_SIZE;
    DMA_SPAR(DMA1, DMA_CHANNEL7) = (uint32_t)(&TIM_CCR1(TIM4));
    DMA_SM0AR(DMA1, DMA_CHANNEL7) = (uint32)(&ws2812_dma_buffer[0]);
    DMA_LIFCR(DMA1) = 0;
    DMA_HIFCR(DMA1) = 0;



    

    /* This is setting up the DMA and has to be ported to F4... */
    //DMA_CCR(DMA1, DMA_CHANNEL7) = 0;
    //DMA_CNDTR(DMA1, DMA_CHANNEL7) = WS2812_BUFFER_SIZE;
    //DMA_CPAR(DMA1, DMA_CHANNEL7) = (uint32_t)(&TIM_CCR1(TIM4));
    //DMA_CMAR(DMA1, DMA_CHANNEL7) = (uint32_t)(&ws2812_dma_buffer[0]);
    //DMA_IFCR(DMA1) = DMA_IFCR_CGIF(DMA_CHANNEL7);

    /* This is setting up the interrupt controller to call the DMA interrupts. */
    //nvic_clear_pending_irq(NVIC_DMA1_CHANNEL7_IRQ);
    //nvic_enable_irq(NVIC_DMA1_CHANNEL7_IRQ);
    //nvic_set_priority(NVIC_DMA1_CHANNEL7_IRQ, 0);
}

/*--------------------------------------------------------------------*/
static void dma_setup(void)
{
    /* DAC channel 1 uses DMA controller 1 Stream 5 Channel 7. */
    /* Enable DMA1 clock and IRQ */
    rcc_periph_clock_enable(RCC_DMA1);
    nvic_enable_irq(NVIC_DMA1_STREAM5_IRQ);
    dma_stream_reset(DMA1, DMA_STREAM5);
    dma_set_priority(DMA1, DMA_STREAM5, DMA_SxCR_PL_LOW);
    dma_set_memory_size(DMA1, DMA_STREAM5, DMA_SxCR_MSIZE_8BIT);
    dma_set_peripheral_size(DMA1, DMA_STREAM5, DMA_SxCR_PSIZE_8BIT);
    dma_enable_memory_increment_mode(DMA1, DMA_STREAM5);
    dma_enable_circular_mode(DMA1, DMA_STREAM5);
    dma_set_transfer_mode(DMA1, DMA_STREAM5,
                DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
    /* The register to target is the DAC1 8-bit right justified data
       register */
    dma_set_peripheral_address(DMA1, DMA_STREAM5, (uint32_t) &DAC_DHR8R1);
    /* The array v[] is filled with the waveform data to be output */
    dma_set_memory_address(DMA1, DMA_STREAM5, (uint32_t) waveform);
    dma_set_number_of_data(DMA1, DMA_STREAM5, 256);
    dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM5);
    dma_channel_select(DMA1, DMA_STREAM5, DMA_SxCR_CHSEL_7);
    dma_enable_stream(DMA1, DMA_STREAM5);
}

void dma1_stream6_isr(void)
{
    /* TODO */
    //DMA_IFCR(DMA1) = DMA_IFCR_CGIF(DMA_CHANNEL7);
        
    /* Not done yet, so fill the next buffer and wait for the send to
     * complete. */
    if (ws2812_send_begin != ws2812_send_end) {
            ws2812_fill_next_buffer(ws2812_timer_one_high(), ws2812_timer_zero_high());
        return;
    }

    /* Just clock out one extra buffer to make sure everything is
     * completely out before we start the reset sequence. */
    if (ws2812_send_begin != 0) {
        ws2812_send_begin = 0;
        ws2812_send_end = 0;
        return;
    }
    
    /* May already have clocked something out, but we don't care at this
     * point since it's past the end of the chain. */
    ws2812_dma_disable();
    ws2812_timer_start_reset();
}



void ws2812_fill_next_buffer(uint8_t high, uint8_t low)
{
    uint8_t *target;
    if (ws2812_buffer_half) {
        target = &ws2812_dma_buffer[(WS2812_BUFFER_SIZE)/2];
    } else {
        target = &ws2812_dma_buffer[0];
    }
    ws2812_buffer_half = !ws2812_buffer_half;
    for (int total = 0; total < (WS2812_BUFFER_SIZE)/(8*2) &&
            ws2812_send_begin != ws2812_send_end; ++ws2812_send_begin, ++total) {
        uint8_t source = *ws2812_send_begin;
        for (int bit=0; bit<8; bit++, source <<= 1, ++target) {
            if (source & 0x80) {
                *target = high;
            } else {
                *target = low;
            }
        }
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
