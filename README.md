# How to boot a modern RISC-V chip

![RISC-V Website](https://content.riscv.org/wp-content/uploads/2018/09/riscv-logo.png)

The source of this firmware demo is carefully crafted to demonstrate the
simplest possible way to boot and get a single threaded system up and running
on a modern 32 bit RISC-V microcontroller such as the feature rich GD32VF103
from GigaDevice.

The author is not happy with the complexity nor abstraction level of the
example code already available from various sources. Why make it hard when you
can keep it simple?

The overall goal of this demo is to minimize the use of assembly language in
favor of C and use as little as possible of it to get things moving. Also, no
details are abstracted away or tied to some silly IDE.

The code is open source under the modified BSD license  - happy hacking!

Authored by martin.lund@keep-it-simple.com

## Demo firmware

The demo firmware blinks the D3 LED and writes hello world to USART0!

It also includes use of the official GD32VF103 driver library from GigaDevice
which includes examples on how to use every peripheral on the chip.

Example code:

```c
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
```

## Hardware

![Polos GD32VF103 Alef Board](https://www.analoglamb.com/wp-content/uploads/2020/01/polos_gd32vf103_alef_board_00.png)

Polos GD32VF103 Alef Board from AnalogLamb:

https://www.analoglamb.com/product/polos-gd32v-alef-board-risc-v-mcu-board

GD32VF103CBT6 features the following:

 * 128KB FLASH and 32KB RAM
 * 4 x universal 16-bit timer
 * 2 x basic 16-bit timer
 * 1 x advanced 16-bit timer
 * Watchdog timer
 * RTC
 * Systick
 * 3 x USART
 * 2 x I2C
 * 3 x SPI
 * 2 x I2S
 * 2 x CAN
 * 1 x USBFS(OTG)
 * 2 x ADC(10 channel)
 * 2 x DAC

## Toolchain

Use crosstool-ng (see http://crosstool-ng.github.io) to build a modern gcc
toolchain that supports the RISC-V flavor (ilp32,imac) of the GD32VF103.

Simply install latest version of crosstool-ng and build the "riscv32-unknown-elf" sample.

## How to get source

Use git to clone the main repository but also its submodules.

    $ git clone --recurse-submodules https://github.com/lundmar/riscv-gd32vf103-demo.git

## How to build

Use your favorite gcc riscv toolchain like so:

    $ cd riscv-gd32vf103-demo
    $ make CROSS_COMPILE=riscv32-unknown-elf-

Results in the output file firmware.bin

## How to flash

Install the modified dfu-utils found here:
https://github.com/riscv-mcu/gd32-dfu-utils

Then simply connect the board via USB and enter boot mode by pressing the boot0
key + reset key, then unpress reset.

To flash the firmware simply do:

    $ dfu-util -a 0 -d 28e9:0189 -s 0x8000000:mass-erase:force -D firmware.bin

## Documentation

[GD32VF103 datasheet](https://gd32mcu.21ic.com/data/documents/shujushouce/GD32VF103_Datasheet_Rev%201.1.pdf)

[GD32VF103 user manual](https://gd32mcu.21ic.com/data/documents/shujushouce/GD32VF103_User_Manual_EN_V1.2.pdf)

[RISC-V ISA specification - Vol I](https://riscv.org/specifications/isa-spec-pdf/)

[RISC-V ISA specification - Vol II](https://riscv.org/specifications/privileged-isa/)

