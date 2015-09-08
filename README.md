# GPS-disciplined-OXCO
Firmware for http://hackaday.io/project/6872-gps-disciplined-ocxo

To build:

    avr-gcc -Wall -g -Os -ffreestanding -std=c99 -mmcu=attiny4313 -c GPS_disciplined_ocxo_4313.c
    avr-gcc -Wall -g -Os -ffreestanding -std=c99 -mmcu=attiny4313 GPS_disciplined_ocxo_4313.o -o GPS_disciplined_ocxo_4313.elf
    avr-objcopy -j .data -j .text -O ihex GPS_disciplined_ocxo_4313.elf GPS_disciplined_ocxo_4313.hex

To initialize a new chip for this project (it's a good idea to insure that the clock input pin on the chip is getting a square wave before doing this):

    avrdude -c {programmer} -p attiny4313 -U lfuse:w:0xe0:m -U hfuse:w:0xdb:m -U efuse:w:0xff:m
    avrdude -c {programmer} -p attiny4313 -U flash:w:GPS_disciplined_ocxo_4313.hex

The firmware offers a behavioral choice between an FLL and a PLL. The difference between the two is that the FLL does not take past history into consideration for adjusting the frequency. Only the most recent 100 second error is used to step the oscillator. By contrast, the PLL keeps track of the total error over all time (since the last GPS lock). That means that if the frequency is low for one sample, it must be adjusted high for another period to compensate. It does, however, mean that over very long time periods the error should go down, unlike the FLL which will just stay flat.

With DEBUG turned on, you should see the following items on the serial output:

START - the firmware prints this once at startup. If you see it any other time, it means either the watchdog has rebooted the controller or something else has gone wrong.
G_LK / G_UN - GPS lock and unlock.
SB= - this line contains the entire sample buffer, ordered from oldest to newest. The sample buffer contains the most recent 100 second errors.
TV= - the current trim value - that is, the value written to the DAC. 0x8000 represents 0. Values higher than that will result in lower frequency output, and vice-versa. Each count should represent approximately 200 ppt worth of adjustment.
TP= - (PLL only) the current trim percentage. For the PLL, the trim value is maintained internally with two (base 10) fractional digits so that adjustments can be made finer than just one DAC step. The actual DAC value written, however, drops the fractional part.
ER= - the current sum of the error buffer (SB=). This is used to provide the user lock quality feedback.
TE= - (PLL only) the current total error. The sum of all of the 100 second errors (SB=) since the last GPS lock.
PD= - the PDOP value reported by the GPS module in the last $GPGSA sentence.
