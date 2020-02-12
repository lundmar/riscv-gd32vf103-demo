/*
 * Copyright (c) 2020, Martin Lund
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdint.h>
#include "gd32vf103.h"
#include "systick.h"
#include "riscv_encoding.h"

extern uint32_t  _data_start;
extern uint32_t  _data_end;
extern uint32_t  _data_load;
extern uint32_t  _bss_start;
extern uint32_t  _bss_end;
extern uint32_t  _heap_start;
extern uint32_t  _stack_end;
extern uint32_t  _global_pointer;
extern uint32_t  _start_vectors;

extern void main(void);

__attribute__((section(".init"), naked))
void reset(void)
{
    register uint32_t *src, *dst;

    /* Setup global pointer and stack */
    asm volatile("la gp, _global_pointer");
    asm volatile("la sp, _stack_end");

    /* Set up Enhanced CLIC for vectored interrupt table */
    asm volatile("csrw mtvec, %0":: "r"((uint8_t *)(&_start_vectors) + 0x1));

    /* Copy the .data section (initialized data) from flash to RAM */
    src = (uint32_t *) &_data_load;
    dst = (uint32_t *) &_data_start;
    while (dst < (uint32_t *)&_data_end)
    {
        *dst++ = *src++;
    }

    /* Zero initialize the BSS section (zero initialized data) */
    dst = &_bss_start;
    while (dst < (uint32_t *)&_bss_end)
    {
        *dst++ = 0U;
    }

    /* Call main() */
    main();
}

__attribute__((interrupt))
void isr_sync_irq(void)
{
    uint32_t mcause_val = read_csr(mcause);
    uint32_t mstatus_val = read_csr(mstatus);
    uint32_t mtval_val = read_csr(mtval);

    /* Print exception details */
    printf("Exception! (mcause=%ld, mstatus=0x%lx, mtval=0x%lx)\n",
                                  mcause_val, mstatus_val, mtval_val);

    /* Indicate exception by blinking faster */
    while (1)
    {
        /* Turn on LED D3 */
        gpio_bit_set(GPIOC, GPIO_PIN_13);
        delay_1ms(200);

        /* Turn off LED D3 */
        gpio_bit_reset(GPIOC, GPIO_PIN_13);
        delay_1ms(200);
    }
}

__attribute__((interrupt))
void isr_default(void)
{
    printf("Unhandled interrupt - please install missing ISR handler!\n");

    /* Panic */
    while (1)
    {
        /* Turn on LED D3 */
        gpio_bit_set(GPIOC, GPIO_PIN_13);
        delay_1ms(200);

        /* Turn off LED D3 */
        gpio_bit_reset(GPIOC, GPIO_PIN_13);
        delay_1ms(200);
    }
}

/*
 * Interrupt vector table based on addresses of ISR pointer functions (only
 * supported by some RISC-V implementations, mtvec.mode = 0x3) which makes for
 * simpler and more compact code. Not supported by GD32VF103 :(
 */

/*
__attribute__ ((section(".isr_vectors"),aligned(4)))
void (* const isr_vector[])(void) =
{
    isr_default,                    // Synchronous exception
    0,                              // Reserved
    0,                              // Reserved
    isr_default,                    // Software interrupt - eclic_msip_handler
    0,                              // Reserved
    0,                              // Reserved
    0,                              // Reserved
    isr_default,                    // Timer interrupt - eclic_mtip_handler
    0,                              // Reserved
    0,                              // Reserved
    0,                              // Reserved
    isr_default,                    // External interrupt
    isr_default,                    // CLIC S/W interrupt
    0,                              // Reserved
    0,                              // Reserved
    0,                              // Reserved
    0,                              // Reserved
    isr_default,                    // eclic_bwei_handler
    isr_default,                    // eclic_pmovi_handler
    isr_default,                    // WWDGT interrupt
    isr_default,                    // LVD from EXTI interrupt
    isr_default,                    // Tamper interrupt
    isr_default,                    // RTC global interrupt
    isr_default,                    // FMC global interrupt
    isr_default,                    // RCU global interrupt
    isr_default,                    // EXTI Line0 interrupt
    isr_default,                    // EXTI Line1 interrupt
    isr_default,                    // EXTI Line2 interrupt
    isr_default,                    // EXTI Line3 interrupt
    isr_default,                    // EXTI Line4 interrupt
    isr_default,                    // DMA0 channel0 global interrupt
    isr_default,                    // DMA0 channel1 global interrupt
    isr_default,                    // DMA0 channel2 global interrupt
    isr_default,                    // DMA0 channel3 global interrupt
    isr_default,                    // DMA0 channel4 global interrupt
    isr_default,                    // DMA0 channel5 global interrupt
    isr_default,                    // DMA0 channel6 global interrupt
    isr_default,                    // ADC0 and ADC1 global interrupt
    isr_default,                    // CAN0 TX interrupt
    isr_default,                    // CAN0 RX0 interrupt
    isr_default,                    // CAN0 RX1 interrupt
    isr_default,                    // CAN0 EWMC interrupt
    isr_default,                    // EXTI line[9:5] interrupts
    isr_default,                    // TIMER0 break interrupt
    isr_default,                    // TIMER0 update interrupt
    isr_default,                    // TIMER0 trigger and channel commutation interrupt
    isr_default,                    // TIMER0 channel capture compare interrupt
    isr_default,                    // TIMER1 global interrupt
    isr_default,                    // TIMER2 global interrupt
    isr_default,                    // TIMER3 global interrupt
    isr_default,                    // I2C0 event interrupt
    isr_default,                    // I2C0 error interrupt
    isr_default,                    // I2C1 event interrupt
    isr_default,                    // I2C1 error interrupt
    isr_default,                    // SPI0 global interrupt
    isr_default,                    // SPI1 global interrupt
    isr_default,                    // USART0 global interrupt
    isr_default,                    // USART1 global interrupt
    isr_default,                    // USART2 global interrupt
    isr_default,                    // EXTI line[15:10] interrupts
    isr_default,                    // RTC alarm from EXTI interrupt
    isr_default,                    // USBFS wakeup from EXTI interrupt
    0,                              // Reserved
    0,                              // Reserved
    0,                              // Reserved
    0,                              // Reserved
    0,                              // Reserved
    0,                              // Reserved
    0,                              // Reserved
    isr_default,                    // TIMER4 global interrupt
    isr_default,                    // SPI2 global interrupt
    isr_default,                    // USART3 global interrupt
    isr_default,                    // USART4 global interrupt
    isr_default,                    // TIMER5 global interrupt
    isr_default,                    // TIMER6 global interrupt
    isr_default,                    // DMA1 channel0 global interrupt
    isr_default,                    // DMA1 channel1 global interrupt
    isr_default,                    // DMA1 channel2 global interrupt
    isr_default,                    // DMA1 channel3 global interrupt
    isr_default,                    // DMA1 channel4 global interrupt
    0,                              // Reserved
    0,                              // Reserved
    isr_default,                    // CAN1 TX interrupt
    isr_default,                    // CAN1 RX0 interrupt
    isr_default,                    // CAN1 RX1 interrupt
    isr_default,                    // CAN1 EWMC interrupt
    isr_default,                    // USBFS global interrupt
};
*/
