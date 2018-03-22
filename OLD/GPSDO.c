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

// Fuse settings: lfuse=0xe0, hfuse = 0xd5, efuse = 0x1
// ext osc, long startup time, 2.7v brownout, preserve EE, no self-programming

#include <stdlib.h>  
#include <stdio.h>  
#include <string.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>

/************************************************
 *
 * OPTIONS
 *
 */
#define DEBUG

// Comment this out for an FLL instead of a PLL. See the readme for what this means.
#define PLL

//#define OH300

// Hardware version gets incremented with each incompatible change.
// Version 1 is boards up to v1.6
//#define HW_VERSION 1
// Version 2 is boards starting with v1.7
//    Controller is clocked at 20 MHz and powered with +5V
//    Output is derived from controller via timer operating as a divisor
//    Division factor is one-of-four with DIP switches
//    DAC data pin moved from MOSI to MISO to mesh up with SPI functionality.
#define HW_VERSION 2

#if (HW_VERSION == 2)
// This is the divide ratio for the output. The divisor *must* be even and >= 2.
// There are two switches to select different division ratios. Assuming a
// 20 MHz oscillator, these are the most interesting possibilities:
// If the divisor is N, then these values are N/2 - 1
//
//  10 MHz: 0
//   5 MHz: 1
// 2.5 MHz: 3
//   2 MHz: 4
//   1 MHz: 9
// 500 kHz: 19
// 400 kHz: 24
// 250 kHz: 39
// 200 kHz: 49
// 100 kHz: 99
//  50 kHz: 199

// 10 MHz
#define DIVISOR_0 0
// 5 MHz
#define DIVISOR_1 1
// 1 MHz
#define DIVISOR_2 9
// 100 kHz
#define DIVISOR_3 99
#endif

#ifdef PLL
// the PI controller factors. These are as of yet untuned guesses. They represent
// values 100 times higher than they are. That is, imagine moving the decimal point two
// spots left.
//
// To determine the tuning step - that is, the frequency difference between
// adjacent DAC values (in theory), you multiply the control voltage slope
// of the oscillator (ppm per volt) by the DAC step voltage (volts per step).
// The value you get is ppm per step.
//
// Because of the multitude of different error sources, your actual tuning
// granularity target should be at least a half an order of magnitude higher
// than this.
#ifdef OH300
// The tuning step for the OH300 variant is approximately 12 ppt.
// The DAC output range is 2.7 volts (82% of 3.3v) and the tuning
// range across that is 0.8ppm. The target here is 0.1 ppb.
#define K_P (438)
#define K_I (37)
#else
// The tuning step for the DOT050V variant is approximately 180 ppt.
// The DAC output range is 1.65 volts (51% of 3.3v) and the tuning
// range across 2.7v is 10 ppm. The hardware throws away the outer 30%
// of the tuning range because it exceeds the lifetime drift tolerance
// of the oscillator. The target here is 1.0 ppb.
#define K_P (150)
#define K_I (10)
#endif
#endif

#if (HW_VERSION == 2)
// 20 MHz.
#define NOMINAL_CLOCK (20000000L)
#else
// 10 MHz.
#define NOMINAL_CLOCK (10000000L)
#endif

// The clock should never be off by more than 1000 ppm. If it is,
// then something's gone terribly wrong and the best we can do is
// ignore that delta (and log it with DEBUG firmware).
#define MAX_DELTA (10000)

/************************************************/

// our DAC is an inverter.
#define DAC_SIGN (-1)

#define LED_PORT PORTD
#define LED0 _BV(PORTD5)
#define LED1 _BV(PORTD4)

// We don't actually need to read this pin. It triggers a TIMER1_CAPT interrupt.
//#define PPS_PIN _BV(PORTD6)

#if (HW_VERSION == 2)
#define SW_PORT PINB
#define SW0 _BV(PINB0)
#define SW1 _BV(PINB1)
#endif

#define DAC_PORT PORTB
#define DAC_CS _BV(PORTB4)
#if (HW_VERSION >= 2)
#define DAC_DO _BV(PORTB6)
#else
#define DAC_DO _BV(PORTB5)
#endif
#define DAC_CLK _BV(PORTB7)

#define EE_TRIM_LOC ((uint16_t*)0)
// If the stored EEPROM trim value differs by this much from the present value,
// then update it
#ifdef OH300
#define EE_UPDATE_OFFSET (250)
#else
#define EE_UPDATE_OFFSET (10)
#endif

// How many samples do we keep in our rolling window?
#define SAMPLE_COUNT (10)
// How many seconds is one sample? This shouldn't be longer
// than 5 minutes for two reasons: 1. The time counter is only 32 bits,
// and will roll over after ~400 seconds at 10 MHz. 2. At a maximum potential
// error of 10 ppm, that's +/- 30,000 counts, and the range of an int
// is +/-32,768.
#define SAMPLE_SECONDS (100)
// The measurement granularity is 10^9/(NOMINAL_CLOCK * SAMPLE_SECONDS * SAMPLE_COUNT)

#define SERIAL_BAUD (9600)
#define SERIAL_BAUD_CONST ((NOMINAL_CLOCK/(16L * SERIAL_BAUD)) - 1)
// Note that if you ever want to parse a longer sentence, be sure to bump this up.
// But an ATTiny4313 only has 1/4K of RAM, so...
#define RX_BUF_LEN (64)
#define TX_BUF_LEN (32)

volatile int sample_buffer[SAMPLE_COUNT];
volatile char valid_samples;
volatile unsigned char sample_window_pos;
volatile unsigned int timer_hibits;
volatile unsigned long pps_count;
volatile unsigned char gps_status;
volatile unsigned char lock;
volatile unsigned char rx_buf[RX_BUF_LEN];
volatile unsigned char rx_str_len;
#ifdef PLL
volatile long total_error;
volatile long trim_percent;
#else
volatile unsigned int trim_value;
#endif
#ifdef DEBUG
volatile unsigned char pdop_buf[5];
volatile long erroneous_delta;

// serial transmit buffer setup
volatile char txbuf[TX_BUF_LEN];
volatile unsigned int txbuf_head, txbuf_tail;
#endif
#if (HW_VERSION == 2)
unsigned char last_switches;
#endif

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
static void writeDacValue(const unsigned int v) {

  unsigned long value = v;
  // This is the point where we could add in control bits. It turns out
  // we just don't want any.

  // Start with the clock pin high.
  DAC_PORT |= DAC_CLK;
  // Now we start - Assert !CS
  DAC_PORT &= ~DAC_CS;
  for(unsigned long mask = 1L << 23; mask != 0; mask >>= 1) {
    if (value & mask)
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
  if ((TIFR & _BV(TOV1)) && (captured_lowbits < 0x8000)) local_timer_hibits++;

  unsigned long timer_val = (((unsigned long)local_timer_hibits) << 16) | captured_lowbits;
  if ((gps_status & 1) == 0) {
    // at least keep track of the beginning of the second.
    last_timer_val = timer_val;
    return; // we don't care right now.
  }
  if (--sample_window_pos > 0) return; // sample incomplete
  sample_window_pos = SAMPLE_SECONDS; // start a new sample
  unsigned long time_span = timer_val - last_timer_val;
  last_timer_val = timer_val;
  
  // If we have too many, then we're running *fast*.
  long delta = time_span - (SAMPLE_SECONDS * NOMINAL_CLOCK);

  if (labs(delta) > MAX_DELTA && valid_samples >= 0) {
#ifdef DEBUG
    erroneous_delta = delta; // Save this for topside logging
#endif
    pps_count++; // Do the logging
    return; // Skip all of the rest of this
  }
  
  if (valid_samples < 0) {
    valid_samples++; // skip this one
  } else if (valid_samples < SAMPLE_COUNT) {
    sample_buffer[(unsigned char)valid_samples++] = (int)delta;
  } else {
    valid_samples = SAMPLE_COUNT; // it's not ever allowed to be higher than SAMPLE_COUNT.
    for(int i = 0; i < SAMPLE_COUNT - 1; i++) {
      sample_buffer[i] = sample_buffer[i + 1];
    }
    sample_buffer[SAMPLE_COUNT - 1] = delta;
  }
  pps_count++;
}

static inline void handleGPS();

ISR(USART0_RX_vect) {
  unsigned char rx_char = UDR;
  
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

#ifdef DEBUG
ISR(USART0_UDRE_vect) {
  if (txbuf_head == txbuf_tail) {
    // the transmit queue is empty.
    UCSRB &= ~_BV(UDRIE); // disable the TX interrupt
    return;
  }
  UDR = txbuf[txbuf_tail];
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
  } while (buf_in_use >= TX_BUF_LEN - 2) ; // wait for room in the transmit buffer

  txbuf[txbuf_head] = c;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    // this needs to be atomic, because an intermediate state is txbuf_head
    // pointing *beyond* the end of the buffer.
    if (++txbuf_head == TX_BUF_LEN) txbuf_head = 0; // point to the next free spot in the tx buffer
  }
  UCSRB |= _BV(UDRIE); // enable the TX interrupt. If it was disabled, then it will trigger one now.
}

static inline void tx_pstr(const char *buf) {
  for(int i = 0; i < strlen_P(buf); i++)
    tx_char(pgm_read_byte(&(buf[i])));
}

static inline void tx_str(const char *buf) {
  for(int i = 0; i < strlen(buf); i++)
    tx_char(buf[i]);
}

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
#endif

static inline unsigned char hexChar(unsigned char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return 0;
}

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
    char gps_now_valid = (*ptr == '3' || *ptr == '2')?1:0; // The ?: is just in case some compiler decides true is some value other than 1.
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
      valid_samples = -1; // and clear the sample buffer
      sample_window_pos = SAMPLE_SECONDS;
#ifdef PLL
      // Restart the error window for every GPS lock interval. We don't track drift during holdover.
      total_error = 0;
#endif
      lock = 0;
    }
  }
}

void main() {
  // This must be done as early as possible to prevent the watchdog from biting during reset.
  MCUSR = 0;
  wdt_enable(WDTO_500MS);

#if (HW_VERSION == 2)
  PRR |= _BV(PRUSI);
#else
  PRR |= _BV(PRTIM0) | _BV(PRUSI);
#endif

  // set up the serial port
  // On the Tiny4313, there's no UBRR. The two "halves" are not adjacent.
  UBRRH = (SERIAL_BAUD_CONST) >> 8;
  UBRRL = (SERIAL_BAUD_CONST) & 0xff;
  UCSRA = 0;
// If you need to initialize the GPS, then set TXEN, transmit
// whatever is necessary, then clear TXEN. That will make the
// controller's TXD line high impedance so that you can talk
// to the GPS module yourself with the diag port on the board
// if desired. But with DEBUG on, that won't work, since the
// controller transmits whenever. The controller can transmit
// anything it wants - anything that's not a proper NMEA sentence
// will be ignored by the GPS module.
#ifdef DEBUG
  UCSRB = _BV(RXCIE) | _BV(RXEN) | _BV(TXEN); // transmit is for debugging
#else
  UCSRB = _BV(RXCIE) | _BV(RXEN);
#endif
  UCSRC = _BV(UCSZ0) | _BV(UCSZ1);

  // set up the LED port
  // DDD6 is 0 to make PD6 an input for ICP
  DDRD = _BV(DDD4) | _BV(DDD5);
  PORTD = 0; // Turn off both LEDs
  // set up the DAC port
  // set CS high on the DAC *before* setting the direction. 
#if (HW_VERSION == 2)
  // The switch inputs need pull-up.
  PORTB |= DAC_CS | _BV(PORTB0) | _BV(PORTB1);
#else
  PORTB |= DAC_CS;
#endif
#if (HW_VERSION == 2)
  // DB2 is OC0A - the divided clock output.
  DDRB = _BV(DDB7) | _BV(DDB6) | _BV(DDB4) | _BV(DDB2);
#else
  DDRB = _BV(DDB4) | _BV(DDB5) | _BV(DDB7);
#endif
 
  // Restore the DAC to the last written value
#ifdef PLL
  // declare it temporarily in ths context
  {
  unsigned int
#endif
  trim_value = eeprom_read_word(EE_TRIM_LOC);
  if (trim_value == 0xffff) // uninitialized flash
    trim_value = 0x8000; // default to midrange
  writeDacValue(trim_value);
#ifdef PLL
  trim_percent = (((long)trim_value) - 0x8000) * 100;
  }
#endif

#if (HW_VERSION == 2)
  // Set up timer0
  TCCR0A = _BV(COM0A0) | _BV(WGM01); // CTC mode, toggle OC0A on match
  TCCR0B = _BV(CS00); // no prescale - divide by 1
  OCR0A = 0; // Start at minimum divisor.
  last_switches = 0xff; // invalid value - force it to change the first time.
#endif

  // Set up timer1
  TCCR1A = 0; // Normal mode
  TCCR1B = _BV(ICES1) | _BV(CS10); // No noise reduction, rising edge capture, no pre-scale.
  TIMSK = _BV(ICIE1) | _BV(TOIE1); // Interrupt on overflow and capture
  ACSR = _BV(ACD); // also, turn off ACIC so that ICP1/PB0 does the capturing.
  TCNT1 = 0; // clear the counter.
  timer_hibits = 0;

  pps_count = 0;
  gps_status = 0;
  valid_samples = -1;
  sample_window_pos = SAMPLE_SECONDS;
  rx_str_len = 0;
#ifdef PLL
  total_error = 0;
#endif
#ifdef DEBUG
  *pdop_buf = 0; // null terminate
  erroneous_delta = 0; // start with no erroneous delta
  txbuf_tail = txbuf_head = 0; // clear the transmit buffer
#endif

  sei();

#ifdef DEBUG
  tx_pstr(PSTR("START\r\n"));
#endif

  while(1) {
    static unsigned long last_second;
 
    // Pet the dog
    wdt_reset();

#if (HW_VERSION == 2)
    {
      unsigned char switches = SW_PORT & (SW0 | SW1);
      if (last_switches != switches) {
        last_switches = switches;
        switch(switches) {
          case 0:
            OCR0A = DIVISOR_0;
            break;
          case SW0:
            OCR0A = DIVISOR_1;
            break;
          case SW1:
            OCR0A = DIVISOR_2;
            break;
          case SW0 | SW1:
            OCR0A = DIVISOR_3;
            break;
        }
        TCNT0 = 0; // clear it out to force a reset.
      }
    }
#endif

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
    if (erroneous_delta != 0) {
      char buf[10];
      tx_pstr(PSTR("XXX="));
      ltoa(erroneous_delta, buf, 10);
      tx_str(buf);
      tx_pstr(PSTR("\r\n"));
      erroneous_delta = 0; // clear it out
    }
#endif

    // next, take care of the LEDs.
    // If gps_status is 0, then blink them back and forth at 2 Hz.
    // Otherwise, put the binary value of "lock" on the two LEDs.
    if (gps_status & 0x1) {
      if (lock & 1)
        LED_PORT |= LED0;
      else
        LED_PORT &= ~LED0;
      if (lock & 2)
        LED_PORT |= LED1;
      else
        LED_PORT &= ~LED1;
    } else {
      unsigned int blink_pos = timer_hibits % (NOMINAL_CLOCK / 65536);
      blink_pos = (4 * blink_pos) / (NOMINAL_CLOCK / 65536);
      if (blink_pos & 1) {
        LED_PORT |= LED0;
        LED_PORT &= ~LED1;
      } else {
        LED_PORT |= LED1;
        LED_PORT &= ~LED0;
      }
    }

    // If we haven't had a PPS event since we were last here, we're done.
    if (last_second == pps_count) continue;
    last_second = pps_count;

    // Collect the sum total of all of the deltas in the sample buffer.
    long sample_drift = 0;
    for(int i = 0; i < valid_samples; i++) {
      sample_drift += sample_buffer[i];
#ifdef DEBUG
      char buf[8];
      tx_pstr(PSTR("SB="));
      itoa(sample_buffer[i], buf, 10);
      tx_str(buf);
      tx_char(' ');
#endif
    }
#ifdef DEBUG
    if (valid_samples > 0)
      tx_pstr(PSTR("\r\n"));
#endif

    // If the sample buffer is full, claim success if the total drift is under control
    // Each count is 0.1 ppb, but you have to add one to round up.
    if (valid_samples < SAMPLE_COUNT) {
      lock = 0;
    } else if (abs(sample_drift) < 500) { // 50 ppb
      if (abs(sample_drift) < 50) { // 5 ppb
        if (abs(sample_drift) < 10) // 1 ppb
          lock = 3; // best
        else
          lock = 2; // better 
      } else
        lock = 1; // good
    } else
      lock = 0; // bad

    // If we don't have at least one sample yet, we're done.
    if (valid_samples <= 0) continue;

    // What we actually do to the oscillator depends only
    // on the most recent sample. The sample window is only
    // for user feedback
    int latest_sample = sample_buffer[valid_samples - 1];
    
#ifdef PLL
    total_error += latest_sample;

    // For the PLL, use a PI controller (a PID without the D). For us the "P" factor will be the last error,
    // and the "I" factor will be the total error. If we needed to come up with a "D" factor, it would
    // likely be the delta between the first and last sample in the sample buffer, or last and next-to-last.
    long adj_val = DAC_SIGN * ((((long)latest_sample) * K_P) + (total_error * K_I));
    trim_percent -= adj_val;
    // And now, throw away the fractional part for writing to the DAC.
    unsigned int trim_value = (int)(trim_percent / 100) + 0x8000;

#else
    // This FLL code doesn't use PI because PI or PID requires looking back at the history.
    // We are consciously ignoring everything except the most recent sample.

    if (latest_sample == 0) {
      // WOO HOO! Nothing to do!
    } else if (abs(latest_sample) < 4) {
      // When we're close in, just *nudge* the clock one unit at a time
      // but only using the most recent error delta.
      if (latest_sample != 0) {
        trim_value += DAC_SIGN * ((latest_sample<0)?1:-1);
      }
    } else if (abs(latest_sample) < 500) {
      // Try and guestimate from the sample drift how hard to hit the
      // oscillator. Each DAC count value is worth around 0.4 ppb, and
      // each error step is 1 ppb. But we want to under-adjust slightly
      // to avoid oscillation. So let's call it 4 DAC counts per error unit.
      trim_value -= DAC_SIGN * latest_sample * 2;
    } else {
      // WTF? Nothing makes sense anymore. Give it a hard shove.
      trim_value += DAC_SIGN * ((latest_sample < 0)?1000:-1000);
    }
#endif
    writeDacValue(trim_value);

#ifdef DEBUG
    {
      char buf[8];
#ifdef PLL
      tx_pstr(PSTR("TE="));
      itoa(total_error, buf, 10);
      tx_str(buf);
      tx_pstr(PSTR("\r\nAV="));
      tx_fp(adj_val, 2);
      tx_pstr(PSTR("\r\nTP="));
      tx_fp(DAC_SIGN * trim_percent, 2);
      tx_str(buf);
      tx_pstr(PSTR("\r\n"));
#endif
      tx_pstr(PSTR("TV="));
      itoa(trim_value, buf, 16);
      tx_str(buf);
      tx_pstr(PSTR("\r\nER="));
      itoa(sample_drift, buf, 10);
      tx_str(buf);
      tx_pstr(PSTR("\r\nPD="));
      tx_str((const char *)pdop_buf);
      tx_pstr(PSTR("\r\n"));
    }
#endif

    // Only write to EEPROM when we're *exactly* dialed in, and
    // our trim value differs from the recorded one "significantly." 
    if (latest_sample == 0 && abs(eeprom_read_word(EE_TRIM_LOC) - trim_value) > EE_UPDATE_OFFSET) {
      eeprom_write_word(EE_TRIM_LOC, trim_value);
      tx_pstr(PSTR("EEU\r\n"));
    }
  }
}
