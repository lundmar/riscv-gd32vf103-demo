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

/*
 * Simple example on how to boot and use the GD32VF103 RISC-V chip
 *
 * Blinks LED D3 and prints to USART0
 *
 * Uses the GD32VF103 firmware library
 *
 * Authored by martin.lund@keep-it-simple.com
 *
 */

#include <stdio.h>
#include "gd32vf103.h"
#include "systick.h"

void setup_blinking_led(void)
{
    /* Enable the LED clock */
    rcu_periph_clock_enable(RCU_GPIOC);

    /* Configure LED GPIO port */
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);

    /* Turn off LED D3 */
    gpio_bit_reset(GPIOC, GPIO_PIN_13);
}

void setup_usart0(void)
{
    /* Enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);

    /* Enable USART clock */
    rcu_periph_clock_enable(RCU_USART0);

    /* Connect port to USARTx_Tx */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

    /* Connect port to USARTx_Rx */
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

    /* USART configure (115200,8n1) */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);

    /* Enable USART0 */
    usart_enable(USART0);
}

void main(void)
{
    int count = 10;

    /* Initialize system (clocks, power, etc.)*/
    SystemInit();

    /* Setup LED D3 */
    setup_blinking_led();

    /* Setup USART0 (115200,8n1) */
    setup_usart0();

    while(count--)
    {
        /* Turn on LED D3 */
        gpio_bit_set(GPIOC, GPIO_PIN_13);
        delay_1ms(500);

        /* Turn off LED D3 */
        gpio_bit_reset(GPIOC, GPIO_PIN_13);
        delay_1ms(500);

        /* Lets say hello */
        printf("Hello world!\n");
    }

    /* Throw environment call exception (system call) */
    asm volatile("ecall");
}

