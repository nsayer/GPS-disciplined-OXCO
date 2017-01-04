
# Change this to whatever AVR programmer you want to use.
PROGRAMMER = usbtiny

# Select the hardware version you're using
# v1 or v2
#CHIP = attiny4313
#OUT=GPSDO
# v3
#CHIP = attiny841
#OUT=GPSDO_v3
# FE
#CHIP = attiny841
#OUT=GPSDO_FE
# FE v2
#CHIP = atmega328pb
#OUT=GPSDO_FE
# v4
CHIP = atmega328pb
OUT=GPSDO_v4

CC = avr-gcc
OBJCPY = avr-objcopy
AVRDUDE = avrdude
OPTS = -Os -g -std=c11 -Wall -Wno-main

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

fuse:
	$(AVRDUDE) -c $(PROGRAMMER) -p $(CHIP) -U hfuse:w:0xd4:m -U lfuse:w:0xe0:m -U efuse:w:0xff:m

init:	fuse flash
