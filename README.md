# GPS-disciplined-OXCO
Firmware for http://hackaday.io/project/6872-gps-disciplined-ocxo

To build:

First, edit GPSDO_v3.c and change any options. In particular, make sure the OH300 option is commented in or out
depending on your hardware.

    avr-gcc -Wall -g -Os -ffreestanding -std=c99 -mmcu=attiny841 -c GPSDO_v3.c
    avr-gcc -Wall -g -Os -ffreestanding -std=c99 -mmcu=attiny841 GPSDO_v3.o -o GPSDO_v3.elf
    avr-objcopy -j .data -j .text -O ihex GPS_disciplined_ocxo_4313.elf GPSDO_v3.hex

To initialize a new chip for this project (it's a good idea to insure that the clock input pin on the chip is getting a square wave before doing this):

    avrdude -c {programmer} -p attiny841 -U lfuse:w:0xe0:m -U hfuse:w:0xd4:m -U efuse:w:0xff:m
    avrdude -c {programmer} -p attiny841 -U flash:w:GPSDO_v3.hex

With DEBUG turned on, you should see the following items on the serial output:

* START - the firmware prints this once at startup. If you see it any other time, it means either the watchdog has rebooted the controller or something else has gone wrong.
* RES_xx - the reason for the restart. Can be either PO for power-up, ER for external reset, BO for brownout, or WD for watchdog.
* XXI - there was an "erroneous" cycle delta. Between two PPS pulses, there should be exactly 10,000,000 cycles of the oscillator. When the count is off by more than the oscillator's basic tolerance window spec, then the unreasonable delta is logged and ignored.
* XXS - Here, an erroneous delta was close to a multiple of 10,000,000. This indicates instead that one or more PPS intervals were skipped. In this case, any delta is scaled over that many seconds, but it's otherwise accepted (unless it's concurrent with an XXI).
* G_LK / G_UN - GPS lock and unlock.
* MOD= - the mode. 0 is FLL, 1 is fast PLL, 2 is slow PLL. This is also reflected on the LEDs.
* SB= - The current cycle count delta.
* CPE= - The current phase error - the ADC reading turned into an error value (that is, subtracted from the midpoint).
* APE= - The phase error averaged over the averaging window
* PPE= - the cycle count error averaged over the averaging window
* AV= - the adjustment value - During the FLL, this is accumulated to form the trim value. During PLL, it's added to the (fixed) trim value.
* TV= - the current trim value - the base against which the AV is applied during PLL operation.
* DAC= - the hex value being written to the DAC.
* ET= - The exit timer. For FLL or fast PLL mode, this counts how long conditions have been acceptable to transition to the next mode.
* iT= - the I term of the PI loop (during PLL)
* pT= - the P term of the PI loop (during PLL)
* PD= - the PDOP value reported by the GPS module in the last $GPGSA sentence.

The author would like to acknowledge the generous assistance of Jim Harman and Tom Van Baak in the development of this project.
