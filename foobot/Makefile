# Makefile
TARGET = firmware
MCU = attiny2313
AVRDUDE_MCU = t2313
F_CPU = 8000000

SRCS = $(shell ls *.c)
OBJS = $(SRCS:%.c=$(OBJDIR)/%.o)
LIBS =
CC = avr-gcc
OBJDIR = .build
DEPDIR = .deps
CFLAGS = \
	-O3 -mmcu=$(MCU) -funsigned-char -funsigned-bitfields -ffunction-sections \
	-fpack-struct -fshort-enums -finline-limit=20 -Wall -Wstrict-prototypes \
	-Wundef -std=gnu99 -Wall -pedantic -D__AVR_ATtiny2313__
LDFLAGS = -mmcu=$(MCU) -Wl,--relax -Wl,--gc-sections -lm

all: $(TARGET).hex

$(TARGET).hex: $(OBJS)
	$(CC) $(LDFLAGS) -Wl,-Map=$(OBJDIR)/$(TARGET).map,--cref -o $(OBJDIR)/$(TARGET).elf $(OBJS) $(LIBS)
	avr-objcopy -O ihex -R .eeprom $(OBJDIR)/$(TARGET).elf $(TARGET).hex

$(OBJDIR)/%.o : %.c
	$(CC) -c $(CFLAGS) -MMD -MP -MF $(DEPDIR)/$(@F).d -Wa,-adhlns=$(OBJDIR)/$<.lst $< -o $@ -DF_CPU=$(F_CPU)

clean:
	rm -rf $(OBJDIR) $(TARGET).hex $(DEPDIR)

-include $(shell mkdir -p $(OBJDIR) $(DEPDIR) 2>/dev/null) $(wildcard $(DEPDIR)/*)

flash: $(TARGET).hex
	sudo avrdude -p $(AVRDUDE_MCU) -c avrispv2 -P /dev/ttyACM0 -U flash:w:$(TARGET).hex

