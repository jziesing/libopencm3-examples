
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
#include <libopencm3/stm32/dma.h>
#include <libopencmsis/core_cm3.h>

//void dma_start(void);
/*
 * global variables:
 *   -increment = pwm duty cycle value
 *   -descending = flag for increasing or decreasing increment
 */
 uint16_t data_block[256];
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
    gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO12);
    /* Set GPIO12, GPIO13, GPIO14, and GPIO15 (in GPIO port D) alternate function */
    gpio_set_af(GPIOD, GPIO_AF2, GPIO12);
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
    // timer_disable_oc_output(TIM4, TIM_OC2);
    // timer_disable_oc_output(TIM4, TIM_OC3);
    // timer_disable_oc_output(TIM4, TIM_OC4);
    /*
     * clear output compare registers on all 4 channels
     */
    timer_disable_oc_clear(TIM4, TIM_OC1);
    // timer_disable_oc_clear(TIM4, TIM_OC2);
    // timer_disable_oc_clear(TIM4, TIM_OC3);
    // timer_disable_oc_clear(TIM4, TIM_OC4);
    /*
     * enable output compare preloading or auto updating
     */
    timer_enable_oc_preload(TIM4, TIM_OC1);
    // timer_enable_oc_preload(TIM4, TIM_OC2);
    // timer_enable_oc_preload(TIM4, TIM_OC3);
    // timer_enable_oc_preload(TIM4, TIM_OC4);
    /*
     * set the output compare to slow mode in all 4 channels
     */
    timer_set_oc_slow_mode(TIM4, TIM_OC1);
    // timer_set_oc_slow_mode(TIM4, TIM_OC2);
    // timer_set_oc_slow_mode(TIM4, TIM_OC3);
    // timer_set_oc_slow_mode(TIM4, TIM_OC4);
    /*
     * set the output compare to pwm1 for all for channels
     */
    timer_set_oc_mode(TIM4, TIM_OC1, TIM_OCM_PWM1);
    // timer_set_oc_mode(TIM4, TIM_OC2, TIM_OCM_PWM1);
    // timer_set_oc_mode(TIM4, TIM_OC3, TIM_OCM_PWM1);
    // timer_set_oc_mode(TIM4, TIM_OC4, TIM_OCM_PWM1);
    /*
     * disable output compare output on all 4 channels
     */
    timer_set_oc_polarity_high(TIM4, TIM_OC1);
    // timer_set_oc_polarity_high(TIM4, TIM_OC2);
    // timer_set_oc_polarity_high(TIM4, TIM_OC3);
    // timer_set_oc_polarity_high(TIM4, TIM_OC4);
    /*
     * set output compare value to 500, dim or short duty cycle
     */
    timer_set_oc_value(TIM4, TIM_OC1, 500);
    // timer_set_oc_value(TIM4, TIM_OC2, 500);
    // timer_set_oc_value(TIM4, TIM_OC3, 500);
    // timer_set_oc_value(TIM4, TIM_OC4, 500);
    /*
     * enable output compare output on all 4 channels
     */
    timer_enable_oc_output(TIM4, TIM_OC1);
    // timer_enable_oc_output(TIM4, TIM_OC2);
    // timer_enable_oc_output(TIM4, TIM_OC3);
    // timer_enable_oc_output(TIM4, TIM_OC4);
    /*
     * enable the preload
     * enable the counter
     * enable the inturrupt on the auto update register

     */
    //timer_set_dma_on_update_event(TIM4);
    
    timer_enable_preload(TIM4);
    timer_enable_counter(TIM4);
    timer_enable_irq(TIM4, TIM_DIER_UDE);
}
/*--------------------------------------------------------------------*/
static void dma_setup(void)
{
    // good
    rcc_periph_clock_enable(RCC_DMA1);
    nvic_enable_irq(NVIC_DMA1_STREAM6_IRQ);
    dma_stream_reset(DMA1, DMA_STREAM6);
    dma_set_priority(DMA1, DMA_STREAM6, DMA_SxCR_PL_MEDIUM);
    // 16 bit seems good size
    dma_set_memory_size(DMA1, DMA_STREAM6, DMA_SxCR_MSIZE_16BIT);
    dma_set_peripheral_size(DMA1, DMA_STREAM6, DMA_SxCR_PSIZE_16BIT);
    // not too sure about these settings
    dma_enable_memory_increment_mode(DMA1, DMA_STREAM6);
    dma_enable_circular_mode(DMA1, DMA_STREAM6);
    dma_set_transfer_mode(DMA1, DMA_STREAM6, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
    // not convinced
    dma_set_peripheral_address(DMA1, DMA_STREAM6, (uint32_t)&TIM4_CCR1);
    // I think this should be a local variable need to make it
    dma_set_memory_address(DMA1, DMA_STREAM6,(uint32_t)data_block);
    // number of datablocks to transfer from mem to peripheral
    dma_set_number_of_data(DMA1, DMA_STREAM6, 256);
    // inturrupt when halfway and when complete.
    //dma_enable_half_transfer_interrupt(DMA1, DMA_STREAM6);
    dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM6);
    // use channel 2 b/c its mapped to TIM4_UP on stream 6
    dma_channel_select(DMA1, DMA_STREAM6, DMA_SxCR_CHSEL_2);
    


    // needed to catch dma compete flag
    nvic_clear_pending_irq(NVIC_DMA1_STREAM6_IRQ);
    nvic_enable_irq(NVIC_DMA1_STREAM6_IRQ);
    nvic_set_priority(NVIC_DMA1_STREAM6_IRQ, 0);
}

static void dma_start(void)
{
     // good to get dma running
    dma_enable_stream(DMA1, DMA_STREAM6);
}

void dma1_stream6_isr(void)
{
    
    if (dma_get_interrupt_flag(DMA1, DMA_STREAM6, DMA_TCIF)) {
#if 0
        dma_clear_interrupt_flags(DMA1, DMA_STREAM6, DMA_TCIF);
     
        dma_stream_reset(DMA1, DMA_STREAM6);
        dma_set_priority(DMA1, DMA_STREAM6, DMA_SxCR_PL_MEDIUM);
        // 16 bit seems good size
        dma_set_memory_size(DMA1, DMA_STREAM6, DMA_SxCR_MSIZE_16BIT);
        dma_set_peripheral_size(DMA1, DMA_STREAM6, DMA_SxCR_PSIZE_16BIT);
        // not too sure about these settings
        dma_enable_memory_increment_mode(DMA1, DMA_STREAM6);
        //dma_enable_circular_mode(DMA1, DMA_STREAM6);
        dma_set_transfer_mode(DMA1, DMA_STREAM6, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
        // not convinced
        dma_set_peripheral_address(DMA1, DMA_STREAM6, (uint32_t)&TIM4_CCR1);
        // I think this should be a local variable need to make it
        dma_set_memory_address(DMA1, DMA_STREAM6,(uint32_t)data_block);
        // number of datablocks to transfer from mem to peripheral
        dma_set_number_of_data(DMA1, DMA_STREAM6, 256);
        // inturrupt when complete, send data again but this time less in inturrupt
        dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM6);
        // use channel 2 b/c its mapped to TIM4_UP on stream 6
        dma_channel_select(DMA1, DMA_STREAM6, DMA_SxCR_CHSEL_2);
        // good to get dma running
        dma_enable_stream(DMA1, DMA_STREAM6);
#endif
    }
}

int main(void)
{
    /*
     * initialize to increasing duty cycle
     * setup the clock, gpio, and timer.. in that order
     */
    clock_setup();
    gpio_setup();
    tim_setup();
    dma_setup();
    int i;
    for(i=0; i<256; i++) {
        data_block[i] = i*100;
    }
    dma_start();
    /*
     * just wait for the inturrupt..
     */

    while (1) {
        __WFI();
    }
    return 0;
}


// void tim4_isr(void)
// {
//     if (timer_get_flag(TIM4, TIM_SR_CC1IF)) {
//         timer_clear_flag(TIM4, TIM_SR_UIF);
//         if (descending == 0)
//         {
//             if (increment >= 65000)
//             {
//                 descending = 1;
//                 increment -= 50;
//             } else {
//                 increment += 50;
//             }
//         } else if (descending == 1)
//         {
//             if (increment <= 500)
//             {
//                 descending = 0;
//                 increment += 50;
//             } else {
//                 increment -= 50;
//             }

//         }
//         timer_set_oc_value(TIM4, TIM_OC1, increment);
//         timer_set_oc_value(TIM4, TIM_OC2, increment);
//         timer_set_oc_value(TIM4, TIM_OC3, increment);
//         timer_set_oc_value(TIM4, TIM_OC4, increment);
//     }
// }