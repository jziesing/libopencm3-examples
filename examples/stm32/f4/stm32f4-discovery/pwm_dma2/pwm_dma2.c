
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
/*
 * global variables:
 *   -increment = pwm duty cycle value
 *   -descending = flag for increasing or decreasing increment
 */ 
uint16_t data_block[256];
uint16_t multiplier;
uint16_t index;
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
    /* Set GPIO12 (in GPIO port D) alternate function */
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
    timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
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
    /*
     * clear output compare registers on all 4 channels
     */
    timer_disable_oc_clear(TIM4, TIM_OC1);
    /*
     * enable output compare preloading or auto updating
     */
    timer_enable_oc_preload(TIM4, TIM_OC1);
    /*
     * set the output compare to slow mode in all 4 channels
     */
    timer_set_oc_slow_mode(TIM4, TIM_OC1);
    /*
     * set the output compare to pwm1 for all for channels
     */
    timer_set_oc_mode(TIM4, TIM_OC1, TIM_OCM_PWM1);
    /*
     * disable output compare output on all 4 channels
     */
    timer_set_oc_polarity_high(TIM4, TIM_OC1);
    /*
     * set output compare value to 0, dim or short duty cycle
     */
    timer_set_oc_value(TIM4, TIM_OC1, 0);
    /*
     * enable output compare output on all 4 channels
     */
    timer_enable_oc_output(TIM4, TIM_OC1);
    /*
     * enable the preload
     * enable the counter
     * enable the inturrupt on the auto update register

     */    
    timer_enable_preload(TIM4);
    timer_enable_counter(TIM4);
    timer_enable_irq(TIM4, TIM_DIER_UDE);
}
static void dma_init(void)
{     
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
    // inturrupt when complete, send data again but this time less in inturrupt
    dma_enable_half_transfer_interrupt(DMA1, DMA_STREAM6);
    dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM6);
    // use channel 2 b/c its mapped to TIM4_UP on stream 6
    dma_channel_select(DMA1, DMA_STREAM6, DMA_SxCR_CHSEL_2);
}
/*--------------------------------------------------------------------*/
static void dma_setup(void)
{
    // setup clock and global interrupt on dma controller
    rcc_periph_clock_enable(RCC_DMA1);
    nvic_enable_irq(NVIC_DMA1_STREAM6_IRQ);
    // call dma init to handle most dma setup calls
    dma_init();
    // enable global interrupts to catch dma transfer complete
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
    // Catch half transfer flag
    if (dma_get_interrupt_flag(DMA1, DMA_STREAM6, DMA_HTIF)) {
        // Clear half transfer flag
        dma_clear_interrupt_flags(DMA1, DMA_STREAM6, DMA_HTIF);
        // Logic to update multipliers to compute values in the data to transfer
        // Very screwey, but does make values generally pulse large and small making led's pulse
        if (descending == 0)
        {
            if (multiplier >= 200)
            {
                descending = 1;
                multiplier -= 10;
                index-= 14;
            } else {
                multiplier += 10;
                index+=14;
            }
        } else if (descending == 1)
        {
            if (multiplier <= 0)
            {
                descending = 0;
                multiplier += 10;
                index += 14;
            } else {
                multiplier -= 10;
                index -= 14;
            }
        }
        // Update to fill first half of data block
        int j;
        for(j=0; j<128; j++) {
            data_block[j] = index*multiplier;
        }
    }
    // Complete transfer flag
    if (dma_get_interrupt_flag(DMA1, DMA_STREAM6, DMA_TCIF)) {
        // Clear complete transfer flag
        dma_clear_interrupt_flags(DMA1, DMA_STREAM6, DMA_TCIF);
        // Update remaining spots in data block
        int j;
        for(j=128; j<256; j++) {
            data_block[j] = index*multiplier;
        }
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
    descending = 0;
    multiplier = 10;
    index = 1;
    int i;
    dma_start();
    /*
     * just wait for the inturrupt..
     */
    while (1) {
        __WFI();
    }
    return 0;
}