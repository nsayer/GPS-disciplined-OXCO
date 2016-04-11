/*

    GPS Disciplined OXCO control sketch
    Copyright (C) 2015 Nicholas W. Sayer

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

// This file is for hardware version 3, which is actually board v2.0 and
// beyond. This version of the hardware has an ATTiny841 controller, a
// 5 volt digital system, a 10 MHz OH300 and the hardware phase detection
// system.

// Fuse settings: lfuse=0xe0, hfuse = 0xd4, efuse = 0x1
// ext osc, long startup time, 4.3v brownout, preserve EEPROM, no self-programming

#include <stdlib.h>  
#include <stdio.h>  
#include <string.h>
#include <math.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>

// 10 MHz.
#define F_CPU (10000000UL)
#include <util/delay.h>

// UBRR?_VALUE macros defined here are Used below in serial initialization in main()
#define BAUD 9600
#include <util/setbaud.h>

/************************************************
 *
 * OPTIONS
 *
 */

// Turn on debug logging. Turn this off if you want to talk to the GPS
// instead of listen to the debug log.
#define DEBUG

// Turn this on to send the WAAS enable sentence at startup.
#define WAAS

#if defined(DEBUG) || defined(WAAS)
// define this to include the serial transmit infrastructure at all
#define SERIAL_TX
#endif

// the PI controller factors. These are as of yet untuned guesses.
//
// To determine the tuning step - that is, the frequency difference between
// adjacent DAC values (in theory), you multiply the control voltage slope
// of the oscillator (ppm per volt) by the DAC step voltage (volts per step).
// The value you get is ppm per step.
//
// Because of the multitude of different error sources, your actual tuning
// granularity target should be at least a half an order of magnitude higher
// than this.
//
// The tuning step for the OH300 variant is approximately 12 ppt.
// The DAC output range is 2.7 volts (82% of 3.3v) and the tuning
// range across that is 0.8ppm. The DAC is 16 bits,
// so .8ppm / 65536 is ~12 ppt. The target here is 0.1 ppb.
//
// We expect F_CPU counts between each PPS interrupt.
// Every count off represents an error of 1e9 / F_CPU ppb (or ns/sec).
//
// The ADC measures a phase range of 1 us, which we arbitrarily
// devide at a midpoint for a range of +/- 512 ns (it's not exactly
// equal to nanoseconds, but it's close enough for us).
//
// The gain is how much we have to nudge the DAC to make 1 ns/sec
// phase change-rate. That is, to alter the frequency by 1 ppb.
#define GAIN 67
//
// In the initial FLL mode, the calculus is different. We multiply
// this value by the error in PPB to determine the adjustment to be
// made to the DAC. When we upshift into PLL, the final trim value
// from the FLL is the base against which the PLL applies adjustment
// values. In principle this value should be 0.015, but we want the
// FLL to be more aggressive.
#define START_GAIN .25
//
// What is our loop time constant? We use two different time constants - a
// faster one when we're outside of a certain range, and a slower
// one when we're dialed in.
#define TC_FAST 50
#define TC_SLOW 200
//
// The damping factor is how much we reduce the influence of the integral
// term - that is, the accumulated error since the last loop startup
// (that is, the last time the FLL determined the starting DAC value).
#define DAMPING 1.75

// our DAC has a positive slope - higher values mean higher frequencies.
// If you have an inverse DAC slope, set this to -1.
#define DAC_SIGN (1)

#define LED_PORT PORTB
#define LED0 _BV(PORTB1)
#define LED1 _BV(PORTB2)

// We don't actually need to read this pin. It triggers a TIMER1_CAPT interrupt.
//#define PPS_PORT PINA
//#define PPS_PIN _BV(PINA7)

#define DAC_PORT PORTA
#define DAC_CS _BV(PORTA3)
#define DAC_DO _BV(PORTA5)
#define DAC_CLK _BV(PORTA4)

// This is an arbitrary midpoint value. We will attempt to coerce the phase error
// to land at this value.
#define PHASE_ADC_MIDPOINT 512

// Note that if you ever want to parse a longer sentence, be sure to bump this up.
// But an ATTiny841 only has 1/2K of RAM, so...
#define RX_BUF_LEN (64)
#define TX_BUF_LEN (96)

// The start mode watches the cycle count error over a 10 second window, and
// adjusts the DAC until a minute goes by without any errors.
#define MODE_START 0

// The FAST and SLOW modes both use the PI loop on phase alone, but with
// the FAST and SLOW time constants, respectively.
#define MODE_FAST 1
#define MODE_SLOW 2

unsigned int last_dac_value;
double iTerm;
double trim_value;
double last_adj_val;
double average_phase_error;
double average_pps_error;
unsigned char mode;
unsigned int exit_timer;
volatile unsigned int timer_hibits;
volatile unsigned long pps_count;
volatile unsigned char gps_status;
volatile unsigned char rx_buf[RX_BUF_LEN];
volatile unsigned char rx_str_len;
volatile unsigned int irq_adc_value;
volatile unsigned long irq_time_span;
#ifdef DEBUG
volatile unsigned char pdop_buf[5];
#endif
#ifdef SERIAL_TX
// serial transmit buffer setup
volatile char txbuf[TX_BUF_LEN];
volatile unsigned int txbuf_head, txbuf_tail;
#endif

// I don't know why my version of AVR libc doesn't have
// fabs() like it's supposed to. Fortunately, it's pretty easy to
// write. It's in the includes, but not the library.
#define fabs MY_fabs
static inline double fabs(double x) { return x<0?-x:x; }

// Write the given 16 bit value to our AD5061 DAC.
// The data format is 6 bits of 0, then two bits of
// shutdown control, which we will set to 0 (because
// we never want to shut down), then 16 bits of
// value, MSB first. Data is clocked on the falling
// edge of the clock pin, and CS must be held low the
// whole time. The output voltage will slew on the rising
// edge of CS. The minimum time between clock transition
// is way faster than our clock speed, so we don't need
// to perform any delays.
static void writeDacValue(unsigned int value) {
  if (value == last_dac_value) return; // don't do useless writes - results in a glitch for no reason
  last_dac_value = value;

  // Start with the clock pin high.
  DAC_PORT |= DAC_CLK;
  // Now we start - Assert !CS
  DAC_PORT &= ~DAC_CS;
  // we're going to write a bunch of zeros. The first six are just
  // padding, and the last two are power-off bits that we want to be
  // always zero.
  DAC_PORT &= ~DAC_DO;
  for(int i = 0; i < 8; i++) {
    // each negative-going pulse of the clock pin shifts in the DO bit.
    DAC_PORT &= ~DAC_CLK;
    DAC_PORT |= DAC_CLK;
  }
  for(int i = 15; i >= 0; i--) {
    if ((value >> i) & 0x1)
      DAC_PORT |= DAC_DO;
    else
      DAC_PORT &= ~DAC_DO;
    DAC_PORT &= ~DAC_CLK;
    DAC_PORT |= DAC_CLK;
  }
  // Raise !CS to end the transfer, which also slews the DAC output.
  DAC_PORT |= DAC_CS;
}

// Timer 1's TCNT register is the low bits. They're ORed
// onto this to make an unsigned long. That gives us more
// than 400 seconds between full overflows (at 10 MHz).
ISR(TIMER1_OVF_vect) {
  timer_hibits++;
}

// When a capture occurs, we calculate the actual number of timer counts
// and the difference between that and the expected value. That
// is added to the sample buffer (rotating it if required), and
// the pps count is incremented (just so we can tell in the main loop
// that the sample buffer has changed). None of this actually matters
// if the GPS receiver is unlocked, though.
ISR(TIMER1_CAPT_vect) {
  static unsigned long last_timer_val;

  // every once in a while, the input capture and timer overflow
  // collide. The input capture interrupt has priority, so when this
  // happens, the high bits won't be incremented, which means the
  // value is 65,536 too low, which wreaks havoc.
  //
  // We can detect this malignancy by seeing if a timer overflow
  // interrupt is pending and if the captured value is "low".
  // If it is, we can simulate the missing overflow interrupt
  // locally. Once we return from this ISR, the overflow one will run next.
  // If the captured low bits are "high," then the overflow happened
  // after the capture, but before the test, in which case
  // we ignore it.
  unsigned int captured_lowbits = ICR1;
  unsigned int local_timer_hibits = timer_hibits;
  if ((TIFR1 & _BV(TOV1)) && (captured_lowbits < 0x8000)) local_timer_hibits++;

  unsigned long timer_val = (((unsigned long)local_timer_hibits) << 16) | captured_lowbits;

  // start ADC operation
  ADCSRA |= _BV(ADSC);
  // wait for ADC to finish
  while(ADCSRA & _BV(ADSC)) ; // don't pet the watchdog - this should never take that long.
  unsigned int adc_value = ADC;

  irq_adc_value = adc_value;

  unsigned long time_span = timer_val - last_timer_val;
  last_timer_val = timer_val;

  irq_time_span = time_span;

  pps_count++;

}

static inline void handleGPS();

ISR(USART0_RX_vect) {
  unsigned char rx_char = UDR0;
  
  if (rx_str_len == 0 && rx_char != '$') return; // wait for a "$" to start the line.
  rx_buf[rx_str_len] = rx_char;
  if (rx_char == 0x0d || rx_char == 0x0a) {
    rx_buf[rx_str_len] = 0; // null terminate
    handleGPS();
    rx_str_len = 0; // now clear the buffer
    return;
  }
  if (++rx_str_len == RX_BUF_LEN) {
    // The string is too long. Start over.
    rx_str_len = 0;
  }
}

const char hexes[] PROGMEM = "0123456789abcdef";

static inline unsigned char charHex(unsigned char h) {
  if (h > 0xf) return ' ';
  return pgm_read_byte(&(hexes[h]));
}

static inline unsigned char hexChar(unsigned char c) {
  if (c >= 'A' && c <= 'F') c += ('a' - 'A'); // make lower case
  const char* outP = strchr_P(hexes, c);
  if (outP == NULL) return 0;
  return (unsigned char)(outP - hexes);
}

#ifdef SERIAL_TX
ISR(USART0_UDRE_vect) {
  if (txbuf_head == txbuf_tail) {
    // the transmit queue is empty.
    UCSR0B &= ~_BV(UDRIE0); // disable the TX interrupt
    return;
  }
  UDR0 = txbuf[txbuf_tail];
  if (++txbuf_tail == TX_BUF_LEN) txbuf_tail = 0; // point to the next char
}

// Note that we're only really going to use the transmit side
// either for diagnostics, or during setup to configure the
// GPS receiver. If the TX buffer fills up, then this method
// will block, which should be avoided.
static inline void tx_char(const char c) {
  int buf_in_use;
  do {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      buf_in_use = txbuf_head - txbuf_tail;
    }
    if (buf_in_use < 0) buf_in_use += TX_BUF_LEN;
    wdt_reset(); // we might be waiting a while.
  } while (buf_in_use >= TX_BUF_LEN - 2) ; // wait for room in the transmit buffer

  txbuf[txbuf_head] = c;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    // this needs to be atomic, because an intermediate state is txbuf_head
    // pointing *beyond* the end of the buffer.
    if (++txbuf_head == TX_BUF_LEN) txbuf_head = 0; // point to the next free spot in the tx buffer
  }
  UCSR0B |= _BV(UDRIE0); // enable the TX interrupt. If it was disabled, then it will trigger one now.
}

static inline void tx_pstr(const char *buf) {
  for(int i = 0; i < strlen_P(buf); i++)
    tx_char(pgm_read_byte(&(buf[i])));
}

static inline void tx_str(const char *buf) {
  for(int i = 0; i < strlen(buf); i++)
    tx_char(buf[i]);
}

// send a NMEA GPS sentence. Takes the sentence
// data between the $ and *, exclusive.
static inline void tx_gps(const char *in) {
  tx_char('$');
  tx_pstr(in);
  tx_char('*');
  unsigned char sum = 0;
  for(int i = 0; i < strlen_P(in); i++) sum += pgm_read_byte(&(in[i]));
  tx_char(charHex(sum >> 8));
  tx_char(charHex(sum & 0xf));
  tx_pstr(PSTR("\r\n"));
}

/*
// lame. But the only available alternative is floating point.
static unsigned long inline pow10(const unsigned int val) {
  unsigned long out = 1;
  for(int i = 0; i < val; i++) out *= 10;
  return out;
}

// Print out a fixed-point value with the given number of decimals
static void tx_fp(const long val, const unsigned int digits) {
  char buf[16];
  tx_char(val<0?'-':'+');
  unsigned long abs_val = labs(val);
  ltoa(abs_val / pow10(digits), buf, 10);
  tx_str(buf);
  if (digits == 0) return;
  tx_char('.');
  unsigned long frac_part = abs_val % pow10(digits);
  for(int i = 1; i < digits; i++)
    if (frac_part < pow10(i)) tx_char('0');
  ltoa(frac_part, buf, 10);
  tx_str(buf);
}
*/

#endif

static void reset_pll();

// When this method is called, we've just received
// a complete NEMA GPS sentence. All we're really
// interested in is whether or not GPS has a 3D fix.
// For that, we'll partially parse the GPGSA sentence.
// Nothing else is of interest.
static inline void handleGPS() {
  unsigned int str_len = rx_str_len; // rx_str_len is where the \0 was written.
 
  if (str_len < 9) return; // No sentence is shorter than $GPGGA*xx
  // First, check the checksum of the sentence
  unsigned char checksum = 0;
  int i;
  for(i = 1; i < str_len; i++) {
    if (rx_buf[i] == '*') break;
    checksum ^= rx_buf[i];
  }
  if (i > str_len - 3) {
    return; // there has to be room for the "*" and checksum.
  }
  i++; // skip the *
  unsigned char sent_checksum = (hexChar(rx_buf[i]) << 4) | hexChar(rx_buf[i + 1]);
  if (sent_checksum != checksum) {
    return; // bad checksum.
  }
  
  if (!strncmp_P((const char*)rx_buf, PSTR("$GPGSA"), 6)) {
    // $GPGSA,A,3,02,06,12,24,25,29,,,,,,,1.61,1.33,0.90*01
    unsigned char *ptr = (unsigned char *)rx_buf;
    for(i = 0; i < 2; i++) {
      ptr = (unsigned char *)strchr((const char *)ptr, ',');
      if (ptr == NULL) {
        return; // not enough commas
      }
      ptr++; // skip over it
    }
    char gps_now_valid = (*ptr == '3')?1:0; // The ?: is just in case some compiler decides true is some value other than 1.
#ifdef DEBUG
    // continue parsing to find the PDOP value
    for(i = 2; i < 15; i++) {
      ptr = (unsigned char *)strchr((const char *)ptr, ',');
      if (ptr == NULL) {
        return; // not enough commas
      }
      ptr++; // skip over it
    }
    unsigned char len = ((unsigned char*)strchr((const char *)ptr, ',')) - ptr;
    if (len > sizeof(pdop_buf) - 1) len = sizeof(pdop_buf) - 1; // truncate if too long
    memcpy((void*)pdop_buf, ptr, len);
    pdop_buf[len] = 0; // null terminate
#endif
    if (gps_now_valid == (gps_status & 1)) { // ignore other than the LSB - it's used by the debug firmware.
      return; // no change in status
    }
    gps_status = gps_now_valid;
    if (!gps_status) {
      // Restart the error window for every GPS lock interval. We don't track drift during holdover.
      reset_pll();
    }
  }
}

static void reset_pll() {
  if (mode != MODE_START) {
    // if we're exiting the PLL, then at least take the most recent
    // adjustment value we had and add it back to the trim value for free-running.
    trim_value -= last_adj_val;
    last_adj_val = 0.;
  }
  iTerm = 0.;
  average_phase_error = 0.;
  average_pps_error = 0.;
  mode = MODE_START;
  exit_timer = 0;
}

void main() {
  // This must be done as early as possible to prevent the watchdog from biting during reset.
  unsigned char mcusr_value = MCUSR;
  MCUSR = 0;
  wdt_enable(WDTO_500MS);

  // We use Timer1, USART0 and the ADC.
  PRR |= _BV(PRTWI) | _BV(PRUSART1) | _BV(PRSPI) | _BV(PRTIM2) | _BV(PRTIM0);

  // set up the serial port
  // uses constants defined above in util/setbaud.h
  UBRR0H = UBRRH_VALUE;
  UBRR0L = UBRRL_VALUE;
#if USE_2X
  UCSR0A = _BV(U2X);
#else
  UCSR0A = 0;
#endif

// If you need to initialize the GPS, then set TXEN, transmit
// whatever is necessary, then clear TXEN. That will make the
// controller's TXD line high impedance so that you can talk
// to the GPS module yourself with the diag port on the board
// if desired. But with DEBUG on, that won't work, since the
// controller transmits whenever. The controller can transmit
// anything it wants - anything that's not a proper NMEA sentence
// will be ignored by the GPS module.
#ifdef DEBUG
  UCSR0B = _BV(RXCIE0) | _BV(RXEN0) | _BV(TXEN0); // transmit always for debug
#else
  UCSR0B = _BV(RXCIE0) | _BV(RXEN0);
#endif
  UCSR0C = _BV(UCSZ00) | _BV(UCSZ01);

  // set up the LED port
  DDRB = _BV(DDRB1) | _BV(DDRB2);
  PORTB = 0; // Turn off both LEDs
  // DDA7 is 0 to make PA7 an input for ICP
  // set up the DAC port
  // set CS high on the DAC *before* setting the direction. 
  DAC_PORT |= DAC_CS;
  DDRA = _BV(DDRA3) | _BV(DDRA4) | _BV(DDRA5);

  // Set up timer1
  TCCR1A = 0; // Normal mode
  TCCR1B = _BV(ICES1) | _BV(CS10); // No noise reduction, rising edge capture, no pre-scale.
  TIMSK1 = _BV(ICIE1) | _BV(TOIE1); // Interrupt on overflow and capture
  TCNT1 = 0; // clear the counter.
  timer_hibits = 0;

  // Set up the ADC
  ACSR0A = _BV(ACD0); // Turn off the analog comparators
  ACSR1A = _BV(ACD1);
  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1); // ADC on, clock scale = 64
  ADMUXA = 0; // ADC0 is A0
  ADMUXB = _BV(REFS1) | _BV(REFS0); // 4.096V is ref, no external connection, no gain
  DIDR0 = _BV(ADC0D); // disable digital I/O on pin A0.

  last_dac_value = 0x8000; // the DAC defaults to powering up at mid-point.
  pps_count = 0;
  mode = MODE_START;
  reset_pll();
  gps_status = 0;
  rx_str_len = 0;
#ifdef SERIAL_TX
  *pdop_buf = 0; // null terminate
  txbuf_head = txbuf_tail = 0; // clear the transmit buffer
#endif

#ifdef WAAS
#ifndef DEBUG
  UCSR0B |= _BV(TXEN0); // turn on the UART TX just for this one operation
#endif
  // brief pause to insure the GPS is listening
  _delay_ms(100);
  wdt_reset(); // that took a while
  // This sentence turns on WAAS reception
  tx_gps(PSTR("PMTK301,2"));
#ifndef DEBUG
  // wait for the transmit buffer to drain.
  char done;
  do {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      done = txbuf_head == txbuf_tail;
    }
    wdt_reset();
  } while(!done);
  UCSR0B &= ~_BV(TXEN0); // turn off the UART TX
#endif
#endif

#ifdef DEBUG
  tx_pstr(PSTR("START\r\n"));
  // one and only one of these should always be printed
  if (mcusr_value & _BV(PORF)) tx_pstr(PSTR("RES_PO\r\n")); // power-on reset
  if (mcusr_value & _BV(EXTRF)) tx_pstr(PSTR("RES_EXT\r\n")); // external reset
  if (mcusr_value & _BV(BORF)) tx_pstr(PSTR("RES_BO\r\n")); // brown-out reset
  if (mcusr_value & _BV(WDRF)) tx_pstr(PSTR("RES_WD\r\n")); // watchdog reset
#endif

  // the default value of the DAC is midpoint, so nothing needs to be done.
  trim_value = 0.;
  last_adj_val = 0.;

  sei();

  while(1) {
    static unsigned long last_pps_count = 0;
 
    // Pet the dog
    wdt_reset();

#ifdef DEBUG
    // with debug firmware, the 0 bit is the GPS status and the 1 bit is
    // whether we've logged that status since the last change or not.
    if (!(gps_status & 0x2)) {
      gps_status |= 0x2;
      if (gps_status & 0x1) {
        tx_pstr(PSTR("G_LK\r\n"));
      } else {
        tx_pstr(PSTR("G_UN\r\n"));
      }
    }
#endif

    // next, take care of the LEDs.
    // If gps_status is 0, then blink them back and forth at 2 Hz.
    // Otherwise, put the binary value of "mode" on the two LEDs.
    if (gps_status & 0x1) {
      if (mode & 1)
        LED_PORT |= LED0;
      else
        LED_PORT &= ~LED0;
      if (mode & 2)
        LED_PORT |= LED1;
      else
        LED_PORT &= ~LED1;
    } else {
      unsigned int blink_pos = timer_hibits % (F_CPU / 65536);
      blink_pos = (4 * blink_pos) / (F_CPU / 65536);
      if (blink_pos & 1) {
        LED_PORT |= LED0;
        LED_PORT &= ~LED1;
      } else {
        LED_PORT |= LED1;
        LED_PORT &= ~LED0;
      }
    }

    // If we haven't had a PPS event since we were last here, we're done.
    if (last_pps_count == pps_count) continue;
    last_pps_count = pps_count;

    if (!(gps_status & 0x1)) {
#ifdef DEBUG
      // FR - Free Running - GPS is unlocked.
      tx_pstr(PSTR("FR\r\n\r\n"));
#endif
      continue;
    }
    long pps_cycle_delta = irq_time_span - F_CPU;

    // round to the nearest second. It's impossible for this to
    // wind up being negative, given reasonably correct GPS behavior.
    unsigned long seconds_delta = (pps_cycle_delta + F_CPU/2) / F_CPU;
    // This is what's left when the whole seconds are accounted for.
    // The result of all this is that a delta of F_CPU - 1 results
    // not in a seconds_delta of 0 and an intracycle_delta of F_CPU-1,
    // but rather a seconds_delta of 1 and an intracycle delta of -1,
    // which is a much better description of the behavior.
    long intracycle_delta = pps_cycle_delta - ((long)(seconds_delta * F_CPU));

    if (labs(intracycle_delta) > (F_CPU / 1000000)) { // this would be an error of 10 ppm - impossible
#ifdef DEBUG
      char buf[16];
      // XXI - an erroneous intracycle delta. A delta of more than 10 ppm is reported, but skipped/ignored.
      tx_pstr(PSTR("XXI="));
      ltoa(intracycle_delta, buf, 10);
      tx_str(buf);
      tx_pstr(PSTR("\r\nXXS="));
      ltoa(seconds_delta, buf, 10);
      tx_str(buf);
      tx_pstr(PSTR("\r\n\r\n"));
#endif
      continue;
    }

    // If the erroneous delta consists only of whole seconds,
    // then it's a "missed PPS." That's far less serious.
    if (seconds_delta != 0) {
#ifdef DEBUG
      char buf[10];
      tx_pstr(PSTR("XXS="));
      ltoa(seconds_delta, buf, 10);
      tx_str(buf);
      tx_pstr(PSTR("\r\n\r\n"));
#endif
    }

    // the time constant in START mode is the same as FAST mode.
    unsigned int time_constant = mode==MODE_SLOW?TC_SLOW:TC_FAST;

    // Since our ADC is 10 bits and the pulse is a microsecond wide we can fudge a little
    // and claim that each ADC count is one nanosecond. So current and average phase error
    // is in nanoseconds and is wrapped.
    int current_phase_error = PHASE_ADC_MIDPOINT - irq_adc_value;

    // This is an approximation of a rolling average, but it's good enough
    // for us, because it should not change very much in 1 second.
    unsigned int filter_time = time_constant / 4;
    average_phase_error -= average_phase_error / filter_time;
    average_phase_error += ((double)current_phase_error) / filter_time;

    // 1 unit here is 1e9/F_CPU ppb, or 100 ppb.
    // A missed PPS means that we have to scale the intracycle delta,
    // because it presumably happened over more than one second.
    average_pps_error -= average_pps_error / filter_time;
    average_pps_error += ((double)intracycle_delta) / ((seconds_delta + 1) * filter_time);

#ifdef DEBUG
    {
      char buf[8];
      tx_pstr(PSTR("MOD="));
      itoa(mode, buf, 10);
      tx_str(buf);
      tx_pstr(PSTR("\r\n"));
    }
#endif

    if (mode == MODE_START) {
      // In the startup mode, we try and convert the average cycle delta
      // into a PPB error
      double adj_val = (1000000000.0 / F_CPU) * average_pps_error * START_GAIN;
      trim_value -= adj_val;
      unsigned int dac_value = (int)(DAC_SIGN * trim_value) + 0x8000;

      writeDacValue(dac_value);
#ifdef DEBUG
      {
        char buf[8];
        tx_pstr(PSTR("SB="));
        ltoa(intracycle_delta, buf, 10);
        tx_str(buf);
        tx_pstr(PSTR("\r\nCPE="));
        ltoa(current_phase_error, buf, 10);
        tx_str(buf);
        tx_pstr(PSTR("\r\nAPE="));
        dtostrf(average_phase_error, 7, 2, buf);
        tx_str(buf);
        tx_pstr(PSTR("\r\nPPE="));
        dtostrf(average_pps_error, 7, 2, buf);
        tx_str(buf);
        tx_pstr(PSTR("\r\nAV="));
        dtostrf(adj_val, 7, 2, buf);
        tx_str(buf);
        tx_pstr(PSTR("\r\nTV="));
        dtostrf(trim_value, 7, 2, buf);
        tx_str(buf);
        tx_pstr(PSTR("\r\nDAC=0x"));
        ltoa(dac_value, buf, 16);
        tx_str(buf);
        tx_pstr(PSTR("\r\nET="));
        ltoa(exit_timer, buf, 10);
        tx_str(buf);
        tx_pstr(PSTR("\r\n"));
        tx_pstr(PSTR("\r\n"));
      }
#endif
      // If the average PPS error stays under 10 ppb for a minute, transition out to phase discipline.
      if (fabs(average_pps_error) <= 0.1) {
        // Once the PPS error is under control, try to get the phase near zero before starting
        // the PLL. But don't try for longer than 20 minutes before giving up.
        if ((++exit_timer >= 60 && fabs(average_phase_error) <= 20.0) || exit_timer >= 1200) {
          mode = MODE_FAST;
          exit_timer = 0;
#ifdef DEBUG
          tx_pstr(PSTR("M_FAST\r\n\r\n"));
#endif
          continue;
        }
      } else {
        exit_timer = 0;
      }
      continue;
    }

    // if we somehow get an error of more than 50 ppb, then it's time to start over.
    if (fabs(average_pps_error) >= 0.5) {
#ifdef DEBUG
      char buf[8];
      tx_pstr(PSTR("PPE="));
      dtostrf(average_pps_error, 7, 2, buf);
      tx_str(buf);
      tx_pstr(PSTR("\r\nM_START\r\n"));
      tx_pstr(PSTR("\r\n"));
#endif
      reset_pll();
      continue;
    }

    if (mode == MODE_FAST) {
#ifdef DEBUG
      {
        char buf[8];
        tx_pstr(PSTR("ET="));
        ltoa(exit_timer, buf, 10);
        tx_str(buf);
        tx_pstr(PSTR("\r\n"));
      }
#endif
      if (fabs(average_phase_error) <= 1.0) {
        if (++exit_timer >= 60) {
          mode = MODE_SLOW;
          time_constant = TC_SLOW;
          exit_timer = 0;
          // translate the iTerm whenever we change the time constant.
          double ratio = ((double)TC_SLOW)/((double)TC_FAST);
          iTerm *= ratio;
#ifdef DEBUG
          tx_pstr(PSTR("M_SLOW\r\n\r\n"));
#endif
        }
      } else {
        exit_timer = 0;
      }
    } else if (mode == MODE_SLOW) {
      if (fabs(average_phase_error) >= 50.0) {
          mode = MODE_FAST;
          time_constant = TC_FAST;
          exit_timer = 0;
          // translate the iTerm whenever we change the time constant.
          double ratio = ((double)TC_FAST)/((double)TC_SLOW);
          iTerm *= ratio;
#ifdef DEBUG
          tx_pstr(PSTR("M_FAST\r\n\r\n"));
#endif
      }
    }
#ifdef DEBUG
    {
      char buf[8];
      // ADC - raw ADC reading from the phase comparator.
      tx_pstr(PSTR("ADC="));
      ltoa(irq_adc_value, buf, 10);
      tx_str(buf);
      tx_pstr(PSTR("\r\nSB="));
      ltoa(intracycle_delta, buf, 10);
      tx_str(buf);
      // PPD - PPS cycle delta - the number of cycles missed/extra since the last PPS.
      tx_pstr(PSTR("\r\nPPE="));
      dtostrf(average_pps_error, 7, 2, buf);
      tx_str(buf);
      tx_pstr(PSTR("\r\nCPE="));
      ltoa(current_phase_error, buf, 10);
      tx_str(buf);
      tx_pstr(PSTR("\r\nAPE="));
      dtostrf(average_phase_error, 7, 2, buf);
      tx_str(buf);
      tx_pstr(PSTR("\r\n"));
    }
#endif

    double pTerm = average_phase_error * GAIN;
    iTerm += pTerm / (time_constant * DAMPING);

    double adj_val = (pTerm + iTerm) / time_constant;

    // For the PLL, the trim_value we calculated during the FLL stays put
    // and the adj_val we've computed will be relative to that.

    // And now, throw away the fractional part for writing to the DAC.
    unsigned int dac_value = (int)(DAC_SIGN * (trim_value - adj_val)) + 0x8000;

    // save this for use when/if we revert to the START mode (GPS unlock).
    last_adj_val = adj_val;

    writeDacValue(dac_value);

#ifdef DEBUG
    {
      char buf[8];
      tx_pstr(PSTR("pT="));
      dtostrf(pTerm, 7, 2, buf);
      tx_str(buf);
      tx_pstr(PSTR("\r\niT="));
      dtostrf(iTerm, 7, 2, buf);
      tx_str(buf);
      // AV = Adjustment Value - the delta being applied right now to the TP
      tx_pstr(PSTR("\r\nAV="));
      dtostrf(adj_val, 7, 2, buf);
      tx_str(buf);
      // TV = Trim Value - the frequency trim factor in DAC units with conventional
      // sign - larger values -> higher frequency
      tx_pstr(PSTR("\r\nTV="));
      dtostrf(trim_value, 7, 2, buf);
      tx_str(buf);
      // DAC = DAC Value - the actual value written to the DAC
      tx_pstr(PSTR("\r\nDAC=0x"));
      itoa(dac_value, buf, 16);
      tx_str(buf);
      // PDOP = Positional Dilution of Precision - the PDOP value reported by the GPS receiver
      tx_pstr(PSTR("\r\nPD="));
      tx_str((const char *)pdop_buf);
      tx_pstr(PSTR("\r\n"));
      // end of the second.
      tx_pstr(PSTR("\r\n"));
    }
#endif
  }
}
