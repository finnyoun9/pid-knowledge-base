# STM32 PID Motor Speed Control
# PWM: PA0 (TIM2_CH1), Direction: PB0/PB1
# Encoder: PB6 (TIM4_CH1), PB7 (TIM4_CH2)
# UART: PA9(TX) PA10(RX) 115200

CROSS = arm-none-eabi-
CC    = $(CROSS)gcc
OBJCOPY = $(CROSS)objcopy
SIZE  = $(CROSS)size

CFLAGS  = -mcpu=cortex-m3 -mthumb -O2 -g -Wall
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += --specs=nosys.specs -nostartfiles

LDFLAGS  = -T linker.ld -Wl,-gc-sections

TARGET = pid-motor

all: $(TARGET).elf

$(TARGET).elf: main.c linker.ld
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ main.c
	$(SIZE) $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

flash: $(TARGET).bin
	st-flash write $(TARGET).bin 0x08000000

clean:
	rm -f $(TARGET).elf $(TARGET).bin
