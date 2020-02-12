#
# Makefile for simple RISC-V boot code for the GD32vf103 chip
# 
# Example makes use of the GD32VF103 firmware library
#
# Authored by martin.lund@keep-it-simple.com
#

CROSS_COMPILE=riscv32-unknown-elf-
CC=$(CROSS_COMPILE)gcc
AS=$(CROSS_COMPILE)gcc
LD=$(CROSS_COMPILE)gcc
OBJCOPY=$(CROSS_COMPILE)objcopy
OBJDUMP=$(CROSS_COMPILE)objdump

LIBGCC=${shell ${CC} ${CFLAGS} -print-libgcc-file-name}
LIBC=${shell ${CC} ${CFLAGS} -print-file-name=libc.a}
LIBM=${shell ${CC} ${CFLAGS} -print-file-name=libm.a}

SOURCES= \
	startup.S \
	startup.c \
	main.c \
	retarget.c \
	external/GD32VF103_Firmware_Library/Firmware/GD32VF103_standard_peripheral/Source/gd32vf103_gpio.c \
	external/GD32VF103_Firmware_Library/Firmware/GD32VF103_standard_peripheral/Source/gd32vf103_rcu.c \
	external/GD32VF103_Firmware_Library/Firmware/GD32VF103_standard_peripheral/Source/gd32vf103_usart.c \
	external/GD32VF103_Firmware_Library/Firmware/GD32VF103_standard_peripheral/system_gd32vf103.c \
	external/GD32VF103_Firmware_Library/Firmware/RISCV/drivers/n200_func.c \
	external/GD32VF103_Firmware_Library/Template/systick.c

OBJECTS= \
	$(patsubst %.c,%.c.o,$(patsubst %.S,%.S.o,$(SOURCES))) \
	$(LIBGCC) \
	$(LIBC) \
	$(LIBM)

LSCRIPT=gd32vf103.ld

CFLAGS:=-Wall -g -Wno-main -Wstack-usage=400 -ffreestanding -Wno-unused -nostdlib
CFLAGS+=-fno-builtin-printf -march=rv32ima -mabi=ilp32 -mcmodel=medany
CFLAGS+=-Iexternal/GD32VF103_Firmware_Library/Firmware/GD32VF103_standard_peripheral/Include
CFLAGS+=-Iexternal/GD32VF103_Firmware_Library/Firmware/GD32VF103_standard_peripheral
CFLAGS+=-Iexternal/GD32VF103_Firmware_Library/Template
CFLAGS+=-Iexternal/GD32VF103_Firmware_Library/Firmware/RISCV/drivers
CFLAGS+=-DGD32VF103C_START
ASFLAGS:=$(CFLAGS)
LDFLAGS:=-T $(LSCRIPT) -Wl,-gc-sections -Wl,-Map=firmware.map -nostdlib -march=rv32ima -mabi=ilp32 -mcmodel=medany -ffunction-sections -fdata-sections

all: $(SOURCES) firmware.bin

firmware.bin: firmware.elf
	$(OBJCOPY) -O binary $^ $@

firmware.elf: $(OBJECTS) $(LSCRIPT)
	$(LD) $(LDFLAGS) $(OBJECTS) -o $@ 

%.c.o: %.c	
	$(CC) -c $(CFLAGS) $< -o $@

%.S.o: %.S	
	$(CC) -c $(CFLAGS) $< -o $@

firmware.S: firmware.elf
	$(OBJDUMP) --disassemble-all $< > $@

clean:
	rm -f firmware.bin firmware.elf firmware.map *.o \
	external/GD32VF103_Firmware_Library/Firmware/GD32VF103_standard_peripheral/*.o \
	external/GD32VF103_Firmware_Library/Firmware/GD32VF103_standard_peripheral/Source/*.o \
	external/GD32VF103_Firmware_Library/Firmware/RISCV/drivers/*.o \
	external/GD32VF103_Firmware_Library/Template/*.o \
	firmware.S

.PHONY: all clean
