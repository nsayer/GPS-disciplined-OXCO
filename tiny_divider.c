/*

 ATTiny Divider
 Copyright 2016 Nicholas W. Sayer
 
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License along
 with this program; if not, write to the Free Software Foundation, Inc.,
 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*
 * This will turn an ATTiny25/45/85 into a frequency divider.
 *
 * The basic purpose is to turn a 10 MHz clock source into
 * a PPS source with the same precision as the source.
 *
 * Fuse the controller for an external clock input (which
 * means that you have to clock it during programming).
 * low = 0xE0, high = 0xDF, extended = 0xff
 *
 * The OC1A pin will toggle with the given divide ratio.
 *
 * Pin 1 is RESET, pin 2 is the input clock, pin 4 is ground,
 * pin 6 is the output clock, pin 7 is Vcc. Pins 3 and 5 are NC.
 */

#include <avr/io.h>
#include <avr/cpufunc.h>
#include <avr/power.h>
#include <avr/interrupt.h>

// Set this to the divisor - we will toggle at double this divisor.
#define DIVISOR (10000000L)

#define PRESCALE (32)

// our division cycling consists of cycle_count full 256 count
// interrupts, followed by a single "short" cycle consisting of
// last_cycle counts. At the end of the "short" cycle, we
// toggle OC1A.
volatile unsigned int cycle_number;
volatile unsigned int cycle_count;
volatile unsigned char last_cycle;

ISR(TIMER1_COMPA_vect) {
  cycle_number++;
  if (cycle_number == cycle_count - 1) {
    // This is the last cycle. Set up the timer
    // with the short length for the *next* cycle,
    OCR1C = last_cycle - 1;
    OCR1A = last_cycle - 1;
  } else if (cycle_number == cycle_count - 0) {
    // This is the short cycle. Do the toggle
    // at the end of it, and set up to go back
    // to long cycles.
    OCR1C = 0xff; // go back to long ones after this
    OCR1A = 0xff; // go back to long ones after this
    TCCR1 ^= _BV(COM1A0); // toggle OC1A match state
  } else if (cycle_number == cycle_count + 1) {
    // This is the "zero-eth" cycle.
    cycle_number = 0; // restart
  }
}

void main() {
  ADCSRA = 0; // DIE, ADC!!! DIE!!!
  ACSR = _BV(ACD); // Turn off analog comparator - but was it ever on anyway?
  power_adc_disable();
  power_usi_disable();
  power_timer0_disable();
  DDRB = _BV(DDB0) | _BV(DDB1) | _BV(DDB2); // all our pins are output.
  // Keep the CS1x bits in sync with the value of PRESCALE above!!
  TCCR1 = _BV(CTC1) | _BV(COM1A1) | _BV(CS12) | _BV(CS11); // CTC. Clear OC1A on match. Prescale = 32
  TIMSK = _BV(OCIE1A); // turn on compare interrupt
  OCR1C = 0xff; // start with the timer maxed out.
  OCR1A = 0xff; // start with the timer maxed out.
  PORTB = 0; // Initialize all pins low.

  // Do the math. Divide the DIVISOR by 2 to make up for the fact
  // that it's a toggling rate. Divide by the prescaler
  // and then divide by 256 to get the quotient, which is the
  // number of full 256 count cycles.
  cycle_count = (unsigned int)(DIVISOR / (2L * PRESCALE * 256));
  // The remainder after the above division is the number
  // of counts in the *short* cycle.
  last_cycle = (unsigned char)((DIVISOR / (2 * PRESCALE)) - (cycle_count * 256));

  sei(); // turn on interrupts
  while(1); // And we're done.

}

