# GPS-disciplined-OXCO
Firmware for http://hackaday.io/project/6872-gps-disciplined-ocxo

To build:

    avr-gcc -g -Os -ffreestanding -std=c99 -mmcu=attiny4313 -c GPS_disciplined_ocxo_4313.c
    avr-gcc -g -Os -ffreestanding -std=c99 -mmcu=attiny4313 GPS_disciplined_ocxo_4313.o -o GPS_disciplined_ocxo_4313.elf
    avr-objcopy -j .data -j .text -O ihex GPS_disciplined_ocxo_4313.elf GPS_disciplined_ocxo_4313.hex

To initialize a new chip for this project (it's a good idea to insure that the clock input pin on the chip is getting a square wave before doing this):

    avrdude -c {programmer} -p attiny4313 -U lfuse:w:0xe0:m -U hfuse:w:0xdb:m -U efuse:w:0xff:m
    avrdude -c {programmer} -p attiny4313 -U flash:w:GPS_disciplined_ocxo_4313.hex
