
# Change this to whatever AVR programmer you want to use.
PROGRAMMER = atmelice_pdi

CHIP = atxmega32e5
OUT=GPSDO_v6

CC = avr-gcc
OBJCPY = avr-objcopy
AVRDUDE = avrdude
OPTS = -Os -g -std=c11 -Wall -Wno-main -fno-tree-switch-conversion

CFLAGS = -mmcu=$(CHIP) $(OPTS)

%.o: %.c Makefile
	$(CC) $(CFLAGS) -c -o $@ $<

%.hex: %.elf
	$(OBJCPY) -j .text -j .data -O ihex $^ $@

%.elf: %.o
	$(CC) $(CFLAGS) -o $@ $^

all:	$(OUT).hex $(OUT).hex

clean:
	rm -f *.hex *.elf *.o

flash:	$(OUT).hex
	$(AVRDUDE) -c $(PROGRAMMER) -p $(CHIP) -U flash:w:$(OUT).hex

init:	flash
