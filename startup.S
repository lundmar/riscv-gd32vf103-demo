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

.macro isr_entry
    addi sp, sp, -64
    sw x1,  0(sp)
    sw x5,  4(sp)
    sw x6,  8(sp)
    sw x7,  12(sp)
    sw x10, 16(sp)
    sw x11, 20(sp)
    sw x12, 24(sp)
    sw x13, 28(sp)
    sw x14, 32(sp)
    sw x15, 36(sp)
    sw x16, 40(sp)
    sw x17, 44(sp)
    sw x28, 48(sp)
    sw x29, 52(sp)
    sw x30, 56(sp)
    sw x31, 60(sp)
.endm

.macro isr_exit
    lw x1,  0(sp)
    lw x5,  4(sp)
    lw x6,  8(sp)
    lw x7,  12(sp)
    lw x10, 16(sp)
    lw x11, 20(sp)
    lw x12, 24(sp)
    lw x13, 28(sp)
    lw x14, 32(sp)
    lw x15, 36(sp)
    lw x16, 40(sp)
    lw x17, 44(sp)
    lw x28, 48(sp)
    lw x29, 52(sp)
    lw x30, 56(sp)
    lw x31, 60(sp)
    addi sp, sp, 64
    mret
.endm

.section .isr_vectors
.align 4
    j _isr_sync_irq             // Synchronous exception
    j _isr_reserved             // Reserved
    j _isr_reserved             // Reserved
    j _isr_default              // Software interrupt
    j _isr_reserved             // Reserved
    j _isr_reserved             // Reserved
    j _isr_reserved             // Reserved
    j _isr_default              // Timer interrupt
    j _isr_reserved             // Reserved
    j _isr_reserved             // Reserved
    j _isr_reserved             // Reserved
    j _isr_default              // External interrupt
    j _isr_default              // CLIC S/W interrupt
    j _isr_reserved             // Reserved
    j _isr_reserved             // Reserved
    j _isr_reserved             // Reserved
    j _isr_reserved             // Reserved
    j _isr_default              // BWEI
    j _isr_default              // PMOVI
    j _isr_default              // WWDGT interrupt
    j _isr_default              // LVD from EXTI interrupt
    j _isr_default              // Tamper interrupt
    j _isr_default              // RTC global interrupt
    j _isr_default              // FMC global interrupt
    j _isr_default              // RCU global interrupt
    j _isr_default              // EXTI line0 interrupt
    j _isr_default              // EXTI line1 interrupt
    j _isr_default              // EXTI line2 interrupt
    j _isr_default              // EXTI line3 interrupt
    j _isr_default              // EXTI line4 interrupt
    j _isr_default              // DMA0 channel0 global interrupt
    j _isr_default              // DMA0 channel1 global interrupt
    j _isr_default              // DMA0 channel2 global interrupt
    j _isr_default              // DMA0 channel3 global interrupt
    j _isr_default              // DMA0 channel4 global interrupt
    j _isr_default              // DMA0 channel5 global interrupt
    j _isr_default              // DMA0 channel6 global interrupt
    j _isr_default              // ADC0 and ADC1 global interrupt
    j _isr_default              // CAN0 TX interrupt
    j _isr_default              // CAN0 RX0 interrupt
    j _isr_default              // CAN0 RX1 interrupt
    j _isr_default              // CAN0 EWMC interrupt
    j _isr_default              // EXTI line[9:5] interrupts
    j _isr_default              // TIMER0 break interrupt
    j _isr_default              // TIMER0 update interrupt
    j _isr_default              // TIMER0 trigger and channel commutation interrupt
    j _isr_default              // TIMER0 channel capture compare interrupt
    j _isr_default              // TIMER1 global interrupt
    j _isr_default              // TIMER2 global interrupt
    j _isr_default              // TIMER3 global interrupt
    j _isr_default              // I2C0 event interrupt
    j _isr_default              // I2C0 error interrupt
    j _isr_default              // I2C1 event interrupt
    j _isr_default              // I2C1 error interrupt
    j _isr_default              // SPI0 global interrupt
    j _isr_default              // SPI1 global interrupt
    j _isr_default              // USART0 global interrupt
    j _isr_default              // USART1 global interrupt
    j _isr_default              // USART2 global interrupt
    j _isr_default              // EXTI line[15:10] interrupts
    j _isr_default              // RTC alarm from EXTI interrupt 
    j _isr_default              // USBFS wakeup from EXTI interrupt
    j _isr_reserved             // Reserved
    j _isr_reserved             // Reserved
    j _isr_reserved             // Reserved
    j _isr_reserved             // Reserved
    j _isr_reserved             // Reserved
    j _isr_reserved             // Reserved
    j _isr_reserved             // Reserved
    j _isr_default              // TIMER4 global interrupt
    j _isr_default              // SPI2 global interrupt
    j _isr_default              // USART3 global interrupt
    j _isr_default              // USART4 global interrupt
    j _isr_default              // TIMER5 global interrupt
    j _isr_default              // TIMER6 global interrupt
    j _isr_default              // DMA1 channel0 global interrupt
    j _isr_default              // DMA1 channel1 global interrupt
    j _isr_default              // DMA1 channel2 global interrupt
    j _isr_default              // DMA1 channel3 global interrupt
    j _isr_default              // DMA1 channel4 global interrupt
    j _isr_reserved             // Reserved
    j _isr_reserved             // Reserved
    j _isr_default              // CAN1 TX interrupt
    j _isr_default              // CAN1 RX0 interrupt
    j _isr_default              // CAN1 RX1 interrupt
    j _isr_default              // CAN1 EWMC interrupt
    j _isr_default              // USBFS global interrupt 

_isr_sync_irq:
    isr_entry
    jal isr_sync_irq
    isr_exit

_isr_default:
    isr_entry
    jal isr_default
    isr_exit

_isr_reserved:
    nop
    j _isr_reserved
