# GPS-disciplined-OXCO
Firmware for http://hackaday.io/project/6872-gps-disciplined-ocxo

To build:

First, edit GPSDO_v6.c and change the include file for the oscillator you want.

    avr-gcc -Wall -Wno-main -g -Os -std=c11 -mmcu=atxmega32e5 -c GPSDO_v6.c
    avr-gcc -Wall -Wno-main -g -Os -std=c11 -mmcu=atxmega32e5 GPSDO_v6.o -o GPSDO_v6.elf
    avr-objcopy -j .data -j .text -O ihex GPSDO_v6.elf GPSDO_v6.hex

To initialize a new chip for this project connect a PDI programmer and:

    avrdude -c {programmer} -p atxmega32e5 -U flash:w:GPSDO_v6.hex

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
* RED= - If the iTerm gets too large, it will be reduced, by off-loading some of its value into TV. Concurrent with this log, B_iT and B_TV will show the values before adjustment, and A_iT and A_TV will show the values after.

The author would like to acknowledge the generous assistance of Jim Harman and Tom Van Baak in the development of this project.
