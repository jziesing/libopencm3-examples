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
 * Note: this program is an attempt to port github user tiltit's ws2812 driver code to stm32 f4.
 * Seems ported, but not working, need to DEBUG.
 */
/* Global vars. */
uint16_t data_block[256];
uint16_t test_block[90];
#define NUMBER_LEDS 8
#define WS2811_HIGH 20
#define WS2811_LOW  9
typedef struct Rgb {
    uint8_t g;
    uint8_t r;
    uint8_t b;
};
struct Rgb leds[NUMBER_LEDS];
struct Rgb color_ring[NUMBER_LEDS];
uint8_t bufout[48]; // 2 Leds
volatile bool transfered = true;
volatile uint32_t count_leds;
struct Rgb hsv_to_rgb(double h, double s, double v)
{
    double dr, dg, db;
    int16_t i;
    struct Rgb ret;
    i = (int16_t)(h * 6);
    double f = h * 6 - i;
    double p = v * (1 - s);
    double q = v * (1 - f * s);
    double t = v * (1 - (1 - f) * s);
    switch (i % 6) {
    case 0: dr = v, dg = t, db = p; break;
    case 1: dr = q, dg = v, db = p; break;
    case 2: dr = p, dg = v, db = t; break;
    case 3: dr = p, dg = q, db = v; break;
    case 4: dr = t, dg = p, db = v; break;
    case 5: dr = v, dg = p, db = q; break;
    }
    ret.r = dr * 255.0;
    ret.g = dg * 255.0;
    ret.b = db * 255.0;
    return ret;
}

/* Set STM32 to 168 MHz. */
static void clock_setup(void)
{
    rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[((CLOCK_3V3_48MHZ/2)*3)]);
}
/* Enable clock, set alternate function mode, use GPIOD pin 12. */
static void gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOD);
    gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO12);
    gpio_set_output_options(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO12);
    gpio_set_af(GPIOD, GPIO_AF2, GPIO12);
}
/* Set timer to period 104 for 800kHz. Enable continuous, preload, counter, output compare, and interrupt. */
/* Set OC to pwm1 mode. Set interrupt on update dma even. */
static void tim_setup(void)
{
    rcc_periph_clock_enable(RCC_TIM4);
    nvic_enable_irq(NVIC_TIM4_IRQ);
    timer_reset(TIM4);
    timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    //timer_set_prescaler(TIM4, (uint16_t)(rcc_apb1_frequency/24000000) - 1 );
    timer_set_prescaler(TIM4, 2 );
    timer_continuous_mode(TIM4);
    timer_set_period(TIM4, 29);
    timer_disable_preload(TIM4);

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

static void dma_send()
{
    rcc_periph_clock_enable(RCC_DMA1);
    nvic_enable_irq(NVIC_DMA1_STREAM6_IRQ);
    int i, j;
    uint8_t *led8;
    led8 = (uint8_t*)&leds[0];
    for (i = 0; i != 6; ++i) {
        for (j = 0; j != 8; ++j) {
            if ( ( *(led8 + i) & (1 << j)) > 0) {
                bufout[ (i * 8) + (7 - j)] = WS2811_HIGH;
            } else {
                bufout[ (i * 8) + (7 - j)] = WS2811_LOW;
            }
        }
    }
    count_leds = 2;
    transfered = false;
    dma_stream_reset(DMA1, DMA_STREAM6);
    dma_set_peripheral_address(DMA1, DMA_STREAM6, (uint32_t)&TIM4_CCR1);
    dma_set_memory_address(DMA1, DMA_STREAM6,(uint32_t)test_block);
    //dma_set_memory_address(DMA1, DMA_CHANNEL6, (uint32_t)&bufout);
    dma_set_number_of_data(DMA1, DMA_STREAM6, 48);
    //dma_set_number_of_data(DMA1, DMA_CHANNEL6, 48);
    dma_set_transfer_mode(DMA1, DMA_STREAM6, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
    //dma_set_read_from_memory(DMA1, DMA_CHANNEL6);
    dma_enable_memory_increment_mode(DMA1, DMA_STREAM6);
    dma_set_memory_size(DMA1, DMA_STREAM6, DMA_SxCR_MSIZE_8BIT);
    dma_set_peripheral_size(DMA1, DMA_STREAM6, DMA_SxCR_PSIZE_16BIT);
    //dma_set_peripheral_size(DMA1, DMA_CHANNEL6, DMA_CCR_PSIZE_16BIT);
    //dma_set_memory_size(DMA1, DMA_CHANNEL6, DMA_CCR_MSIZE_8BIT);
    dma_set_priority(DMA1, DMA_STREAM6, DMA_SxCR_PL_HIGH);
    dma_enable_circular_mode(DMA1, DMA_STREAM6);
    timer_set_dma_on_compare_event(TIM4);
    dma_enable_half_transfer_interrupt(DMA1, DMA_STREAM6);
    dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM6);
    dma_channel_select(DMA1, DMA_STREAM6, DMA_SxCR_CHSEL_2);
    //dma_enable_channel(DMA1, DMA_CHANNEL6);
    dma_enable_stream(DMA1, DMA_STREAM6);
    timer_enable_counter(TIM4);

   // nvic_clear_pending_irq(NVIC_DMA1_STREAM6_IRQ);
    //nvic_enable_irq(NVIC_DMA1_STREAM6_IRQ);
    //nvic_set_priority(NVIC_DMA1_STREAM6_IRQ, 0);

}
/* Convience method to start the dma after all is setup. */
static void dma_start(void)
{
    dma_enable_stream(DMA1, DMA_STREAM6);
}
/* Stop dma convience. */
static void dma_stop(void) 
{
    dma_disable_stream(DMA1, DMA_STREAM6);
}

void dma1_stream6_isr(void)
{
    static int i;
    uint8_t *led8;
    /* Half Transfer interupt*/
    if (dma_get_interrupt_flag(DMA1, DMA_STREAM6, DMA_HTIF)) {
        gpio_set(GPIOD, GPIO12);
        dma_clear_interrupt_flags(DMA1, DMA_STREAM6, DMA_HTIF);
        if (count_leds >= NUMBER_LEDS) {
            for (i = 0; i != 24; ++i) {
                bufout[i] = 0;
            }
        } else {
            led8 = (uint8_t*)&leds[count_leds];
            bufout[0] = (*led8 & 0x80) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[1] = (*led8 & 0x40) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[2] = (*led8 & 0x20) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[3] = (*led8 & 0x10) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[4] = (*led8 & 0x08) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[5] = (*led8 & 0x04) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[6] = (*led8 & 0x02) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[7] = (*led8 & 0x01) > 0 ? WS2811_HIGH : WS2811_LOW;
            led8++;
            bufout[8] = (*led8 & 0x80) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[9] = (*led8 & 0x40) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[10] = (*led8 & 0x20) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[11] = (*led8 & 0x10) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[12] = (*led8 & 0x08) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[13] = (*led8 & 0x04) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[14] = (*led8 & 0x02) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[15] = (*led8 & 0x01) > 0 ? WS2811_HIGH : WS2811_LOW;
            led8++;
            bufout[16] = (*led8 & 0x80) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[17] = (*led8 & 0x40) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[18] = (*led8 & 0x20) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[19] = (*led8 & 0x10) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[20] = (*led8 & 0x08) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[21] = (*led8 & 0x04) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[22] = (*led8 & 0x02) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[23] = (*led8 & 0x01) > 0 ? WS2811_HIGH : WS2811_LOW;
        }
        count_leds++;
    }
    /* Transfer conplete interupt*/
    if (dma_get_interrupt_flag(DMA1, DMA_STREAM6, DMA_TCIF)) {
        gpio_clear(GPIOD, GPIO12);
        dma_clear_interrupt_flags(DMA1, DMA_STREAM6, DMA_TCIF);
        /* If the data for all leds has been sent, set the pwm pin low */
        if (count_leds >= NUMBER_LEDS) {
            for (i = 24; i != 48; ++i) {
                bufout[i] = 0;
            }
        } else {
            led8 = (uint8_t*)&leds[count_leds];
            bufout[24] = (*led8 & 0x80) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[25] = (*led8 & 0x40) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[26] = (*led8 & 0x20) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[27] = (*led8 & 0x10) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[28] = (*led8 & 0x08) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[29] = (*led8 & 0x04) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[30] = (*led8 & 0x02) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[31] = (*led8 & 0x01) > 0 ? WS2811_HIGH : WS2811_LOW;
            led8++;
            bufout[32] = (*led8 & 0x80) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[33] = (*led8 & 0x40) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[34] = (*led8 & 0x20) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[35] = (*led8 & 0x10) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[36] = (*led8 & 0x08) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[37] = (*led8 & 0x04) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[38] = (*led8 & 0x02) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[39] = (*led8 & 0x01) > 0 ? WS2811_HIGH : WS2811_LOW;
            led8++;
            bufout[40] = (*led8 & 0x80) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[41] = (*led8 & 0x40) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[42] = (*led8 & 0x20) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[43] = (*led8 & 0x10) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[44] = (*led8 & 0x08) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[45] = (*led8 & 0x04) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[46] = (*led8 & 0x02) > 0 ? WS2811_HIGH : WS2811_LOW;
            bufout[47] = (*led8 & 0x01) > 0 ? WS2811_HIGH : WS2811_LOW;
        }
        count_leds++;
        if (count_leds >= NUMBER_LEDS + 6) {
            timer_disable_counter(TIM4);
            //dma_disable_channel(DMA1, DMA_STREAM6);
            dma_disable_stream(DMA1, DMA_STREAM6);
            TIM4_CCR1 = 0;
            transfered = true;
        }
    }
}

int main(void)
{
    // clock_setup();
    // gpio_setup();
    // tim_setup();
    // dma_setup();
    // dma_start();
    // dma_stop();
    // while (1) {
    //     __WFI();
    // }
    //return 0;
    int i;
    uint32_t cnt = 0;
    for (i = 0; i < (1 << 16); i++) /* Wait a bit. */
        __asm__("nop");
    clock_setup();
    gpio_setup();
    tim_setup();
    nvic_set_priority(NVIC_DMA1_STREAM6_IRQ, 0);
    nvic_enable_irq(NVIC_DMA1_STREAM6_IRQ);
    for (i = 0; i != NUMBER_LEDS; ++i) {
        color_ring[i] = hsv_to_rgb(1.0 / NUMBER_LEDS * i, 1.0, 1.0);
    }
    while (1) {
        for (i = 0; i != NUMBER_LEDS ; ++i) {
            leds[i] = color_ring[ ( cnt + i ) % NUMBER_LEDS ];
        }
        cnt++;
        cnt %= NUMBER_LEDS;
        while (!transfered);
        dma_send();
        for (i = 0; i < (1 << 20); i++) // Wait a bit.
            __asm__("nop");
    }
    return 0;
}




/* Method to break up dma setup.  Set's up dma specific configurations for driver. */
/* Using 16 bit stream size. Transfers mem (var data_block) to periph (TIM4_CCR1). */
/* Sets the block size to 256 (65535 max possible). Enable half and full transfer interrupts. Uses channel 1. */
/*static void dma_init(void)
{     
    int i;
    int num_leds = 2;
    uint16_t addr_place = 0;
    uint16_t buff_len = (num_leds*24)+42; 
    while (num_leds) {
        for(i=0; i<8; i++) {
            test_block[addr_place] = 9;
            addr_place++;
        }
        for(i=0; i<8; i++) {
            test_block[addr_place] = 9;
            addr_place++;
        }
        for(i=0; i<8; i++) {
            test_block[addr_place] = 17;
            addr_place++;
        }
        num_leds--;
    }
    while (addr_place < buff_len) {
        test_block[addr_place] = 0;
        addr_place++;
    }

    dma_stream_reset(DMA1, DMA_STREAM6);
    dma_set_priority(DMA1, DMA_STREAM6, DMA_SxCR_PL_HIGH);
    dma_set_memory_size(DMA1, DMA_STREAM6, DMA_SxCR_MSIZE_16BIT);
    dma_set_peripheral_size(DMA1, DMA_STREAM6, DMA_SxCR_PSIZE_16BIT);
    dma_enable_memory_increment_mode(DMA1, DMA_STREAM6);
    //dma_enable_circular_mode(DMA1, DMA_STREAM6);
    dma_set_transfer_mode(DMA1, DMA_STREAM6, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
    dma_set_number_of_data(DMA1, DMA_STREAM6, buff_len);
    dma_set_peripheral_address(DMA1, DMA_STREAM6, (uint32_t)&TIM4_CCR1);
    dma_set_memory_address(DMA1, DMA_STREAM6,(uint32_t)test_block);
    dma_enable_half_transfer_interrupt(DMA1, DMA_STREAM6);
    dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM6);
    dma_channel_select(DMA1, DMA_STREAM6, DMA_SxCR_CHSEL_2);
}*/
/* Method called to setup dma for driver.*/
/* Simplified with init method.  Method handles interactions with clock and global interrupt. */
/*static void dma_setup(void)
{
    rcc_periph_clock_enable(RCC_DMA1);
    nvic_enable_irq(NVIC_DMA1_STREAM6_IRQ);
    dma_init();
    nvic_clear_pending_irq(NVIC_DMA1_STREAM6_IRQ);
    nvic_enable_irq(NVIC_DMA1_STREAM6_IRQ);
    nvic_set_priority(NVIC_DMA1_STREAM6_IRQ, 0);
}*/


/* Interrupt handler.  Handle both half and full transfer flags. */ 
/*void dma1_stream6_isr(void)
{   
    if (dma_get_interrupt_flag(DMA1, DMA_STREAM6, DMA_HTIF)) {
        dma_clear_interrupt_flags(DMA1, DMA_STREAM6, DMA_HTIF);
    }
    if (dma_get_interrupt_flag(DMA1, DMA_STREAM6, DMA_TCIF)) {
        dma_clear_interrupt_flags(DMA1, DMA_STREAM6, DMA_TCIF);
    }
}*/

/*int i;
    int num_leds = 2;
    uint16_t addr_place = 0;
    uint16_t buff_len = (num_leds*24)+42; 
    while (num_leds) {
        for(i=0; i<8; i++) {
            test_block[addr_place] = 9;
            addr_place++;
        }
        for(i=0; i<8; i++) {
            test_block[addr_place] = 9;
            addr_place++;
        }
        for(i=0; i<8; i++) {
            test_block[addr_place] = 17;
            addr_place++;
        }
        num_leds--;
    }
    while (addr_place < buff_len) {
        test_block[addr_place] = 0;
        addr_place++;
    }*/