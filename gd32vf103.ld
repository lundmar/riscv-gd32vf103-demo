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
 * Simple linker script for the GD32VF103CBT6 chip
 *
 * Authored by martin.lund@keep-it-simple.com
 *
 * Memory layout:
 *
 *   ----------------  --------------  0x2000 8000
 *   |  stack (8K)  |
 *   ----------------
 *   |  heap  (16K) |
 *   ----------------    ram (32K)
 *   |  bss         |
 *   ----------------
 *   |  data        |
 *   ----------------  --------------  0x2000 0000
 *   |  text        |   flash (128K)
 *   ----------------  --------------  0x0000 0000
 *
 */


OUTPUT_ARCH("riscv")

ENTRY(reset)

MEMORY
{
    flash (rxai!w) : ORIGIN = 0x00000000, LENGTH = 128K
    ram (wxa!ri) : ORIGIN = 0x20000000, LENGTH = 32K
}

SECTIONS
{
    .text :
    {
        _text_start = .;
        KEEP(*(.init))
        . = ALIGN(64);
        _start_vectors = .;
        KEEP(*(.isr_vectors))
        *(.text*)
        *(.rodata*)
        *(.srodata*)
        . = ALIGN(4);
        _text_end = .;
    } > flash

    .data : AT(ADDR(.text) + SIZEOF(.text))
    {
        _data_start = .;
        *(.data*)
        *(.sdata*)
        _global_pointer = . + 0x400;
        . = ALIGN(4);
        _data_end = .;
    } > ram

    .bss :
    {
        _bss_start = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _bss_end = .;
        _end = .;
    } > ram

}

PROVIDE(_heap_start = _end);
PROVIDE(_stack_end = ORIGIN(ram) + LENGTH(ram));
PROVIDE(_data_load = LOADADDR(.data));
