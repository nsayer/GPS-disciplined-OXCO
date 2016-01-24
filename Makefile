
# Change this to whatever AVR programmer you want to use.
PROGRAMMER = usbtiny

# Change this if you use a different controller.
CHIP = attiny4313

CC = avr-gcc
OBJCPY = avr-objcopy
AVRDUDE = avrdude
OPTS = -Os -g -ffreestanding -std=c99 -Wall

CFLAGS = -mmcu=$(CHIP) $(OPTS)

%.o: %.c Makefile
	$(CC) $(CFLAGS) -c -o $@ $<

%.hex: %.elf
	$(OBJCPY) -j .text -j .data -O ihex $^ $@

%.elf: %.o
	$(CC) $(CFLAGS) -o $@ $^

all:	GPS_disciplined_ocxo_4313.hex

clean:
	rm -f *.hex *.elf *.o

flash:	GPS_disciplined_ocxo_4313.hex
	$(AVRDUDE) -c $(PROGRAMMER) -p $(CHIP) -U flash:w:GPS_disciplined_ocxo_4313.hex

fuse:
	$(AVRDUDE) -c $(PROGRAMMER) -p $(CHIP) -U hfuse:w:0xdf:m -U lfuse:w:0x62:m -U efuse:w:0xff:m

init:	fuse flash
