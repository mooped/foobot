# Makefile
TARGET = firmware
MCU = at90usb162
F_CPU = 16000000

SRCS = $(shell ls *.c)
OBJS = $(SRCS:%.c=$(OBJDIR)/%.o)
LIBS =
CC = avr-gcc
OBJDIR = .build
DEPDIR = .deps
CFLAGS = \
	-O3 -mmcu=$(MCU) -funsigned-char -funsigned-bitfields -ffunction-sections \
	-fpack-struct -fshort-enums -finline-limit=20 -Wall -Wstrict-prototypes \
	-Wundef -std=gnu99 -Wall -pedantic
LDFLAGS = -mmcu=$(MCU) -Wl,--relax -Wl,--gc-sections -lm

all: $(TARGET).hex

$(TARGET).hex: $(OBJS)
	$(CC) $(LDFLAGS) -Wl,-Map=$(OBJDIR)/$(TARGET).map,--cref -o $(OBJDIR)/$(TARGET).elf $(OBJS) $(LIBS)
	avr-objcopy -O ihex -R .eeprom $(OBJDIR)/$(TARGET).elf $(TARGET).hex

$(OBJDIR)/%.o : %.c
	$(CC) -c $(CFLAGS) -MMD -MP -MF $(DEPDIR)/$(@F).d -Wa,-adhlns=$(OBJDIR)/$<.lst $< -o $@ -DF_CPU=$(F_CPU)

clean: FORCE
	rm -rf $(OBJDIR) $(TARGET).hex $(DEPDIR)

-include $(shell mkdir -p $(OBJDIR) $(DEPDIR) 2>/dev/null) $(wildcard $(DEPDIR)/*)

dfu: $(TARGET).hex
	sudo dfu-programmer $(MCU) erase
	sudo dfu-programmer $(MCU) flash --debug 1 $(TARGET).hex
	sudo dfu-programmer $(MCU) reset

