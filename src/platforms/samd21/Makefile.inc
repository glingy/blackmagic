CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

CFLAGS += -mcpu=cortex-m0plus -mthumb \
	-DSAMD -DSAMD21 -I../libopencm3/include \
	-Iplatforms/samd21 -DDFU_SERIAL_LENGTH=9

LDFLAGS = -lopencm3_samd \
	-Wl,-T,platforms/samd21/samd21.ld -nostartfiles -lc -lnosys \
	-Wl,-Map=mapfile -mthumb -mcpu=cortex-m0plus -Wl,-gc-sections \
	-L../libopencm3/lib

VPATH += platforms/samd21

SRC += timing.c serialno.c platform_timing.c

all: blackmagic.bin blackmagic.hex

blackmagic.dfu: blackmagic.hex
	@echo Creating $@
	@python3 ../scripts/dfu-convert.py -i $< $@

host_clean:
	-$(Q)$(RM) blackmagic.bin blackmagic.hex blackmagic.dfu
