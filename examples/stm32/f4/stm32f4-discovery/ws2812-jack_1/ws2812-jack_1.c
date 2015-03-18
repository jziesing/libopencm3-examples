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

uint16_t data_block[136];

static void clock_setup(void)
{
    rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);
}
static void gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOD);
    gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO12);
    gpio_set_af(GPIOD, GPIO_AF2, GPIO12);
}
static void tim_setup(void)
{
    rcc_periph_clock_enable(RCC_TIM4);
    nvic_enable_irq(NVIC_TIM4_IRQ);
    timer_reset(TIM4);
    timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM4, 0);
    timer_continuous_mode(TIM4);
    timer_set_period(TIM4, 104);
    timer_disable_oc_output(TIM4, TIM_OC1);
    timer_disable_oc_clear(TIM4, TIM_OC1);
    timer_enable_oc_preload(TIM4, TIM_OC1);
    timer_set_oc_slow_mode(TIM4, TIM_OC1);
    timer_set_oc_mode(TIM4, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_polarity_high(TIM4, TIM_OC1);
    timer_set_oc_value(TIM4, TIM_OC1, 0);
    timer_enable_oc_output(TIM4, TIM_OC1);
    timer_enable_preload(TIM4);
    timer_enable_counter(TIM4);
    timer_enable_irq(TIM4, TIM_DIER_UDE);
}
static void dma_init(void)
{
    dma_stream_reset(DMA1, DMA_STREAM6);
    dma_set_priority(DMA1, DMA_STREAM6, DMA_SxCR_PL_VERY_HIGH);
    dma_set_memory_size(DMA1, DMA_STREAM6, DMA_SxCR_MSIZE_8BIT);
    dma_set_peripheral_size(DMA1, DMA_STREAM6, DMA_SxCR_PSIZE_16BIT);
    dma_enable_circular_mode(DMA1, DMA_STREAM6);
    dma_enable_memory_increment_mode(DMA1, DMA_STREAM6);
    dma_set_transfer_mode(DMA1, DMA_STREAM6, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
    dma_set_peripheral_address(DMA1, DMA_STREAM6, (uint32_t)&TIM4_CCR1);
    dma_set_memory_address(DMA1, DMA_STREAM6, (uint32_t)&data_block);
    // 2 LED's = (24 * 2) + 40 
    // 40 = 50us/1.25us
    dma_set_number_of_data(DMA1, DMA_STREAM6, 136);
    dma_enable_half_transfer_interrupt(DMA1, DMA_STREAM6);
    dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM6);
    dma_channel_select(DMA1, DMA_STREAM6, DMA_SxCR_CHSEL_2);
}
/*--------------------------------------------------------------------*/
static void dma_setup(void)
{
    rcc_periph_clock_enable(RCC_DMA1);
    nvic_enable_irq(NVIC_DMA1_STREAM6_IRQ);
    dma_init();
    nvic_clear_pending_irq(NVIC_DMA1_STREAM6_IRQ);
    nvic_enable_irq(NVIC_DMA1_STREAM6_IRQ);
    nvic_set_priority(NVIC_DMA1_STREAM6_IRQ, 0);
}
static void dma_start(void)
{
    dma_enable_stream(DMA1, DMA_STREAM6);
}
void dma1_stream6_isr(void)
{
    if (dma_get_interrupt_flag(DMA1, DMA_STREAM6, DMA_HTIF)) {
        dma_clear_interrupt_flags(DMA1, DMA_STREAM6, DMA_HTIF);
    }
    if (dma_get_interrupt_flag(DMA1, DMA_STREAM6, DMA_TCIF)) {
        dma_clear_interrupt_flags(DMA1, DMA_STREAM6, DMA_TCIF);
    }
}
int main(void)
{
    clock_setup();
    gpio_setup();
    tim_setup();
    dma_setup();
    int i;
    // 60 = 1 
    // 29 = 0
    // test make first two led's green
    for (i = 0; i < 136; i++) {
        // green
        if (i < 8) {
            data_block[i] = 60;
        } 
        // red
        else if (i > 31 && i < 40) {
            data_block[i] = 60;
        }
        // green 
        else if (i > 47 && i < 56) {
            data_block[i] = 60;
        }
        // red 
        else if (i > 79 && i < 88) {
            data_block[i] = 60;
        }
        // break
        else if (i > 95) {
            data_block[i] = 0;
        }
        else {
            data_block[i] = 29;
        }

    }
    dma_start();
    while (1) {
        __WFI();
    }
    return 0;
}
