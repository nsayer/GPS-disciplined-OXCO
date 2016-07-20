/*

    GPS Disciplined FE5680A control sketch
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

// This file is for the FE5680A hardware. This version of the hardware
// has an ATTiny841 controller, a 5 volt digital system, the hardware
// phase detection system, and power and interface systems for an FE-5680A
// rubidium oscillator. Your FE-5680A must be the kind that generates 10
// MHz only and is tunable via serial commands.

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

// Serial port baud rate. Used below by serial initialization/reinitialization
#define BAUD 9600

/************************************************
 *
 * OPTIONS
 *
 */

// Turn on debug logging. Turn this off if you want to talk to the GPS
// instead of listen to the debug log.
#define DEBUG

// Turn this on to send the WAAS enable sentence at startup.
//#define WAAS

#if defined(DEBUG) || defined(WAAS)
// define this to include the serial transmit infrastructure at all
#define SERIAL_TX
// define this to make transmission interrupt driven and buffered rather than blocking
//#define IRQ_DRIVEN_TX
#endif

// The FE-5680A's specifications are a tuning range of +/- 383 Hz
// over the range of a 32 bit signed integer, or +/- 38.3 ppm. That's
// an individual step size of 17.834E-15.
//
// However, specification and reality aren't always the same. The
// unit I have for testing shows that at least near zero its resolution
// is more like 256 steps per 2E-10 or so - which is something like
// 50 times less resolution.
//
// The ADC measures a phase range of 1 us, which we arbitrarily
// devide at a midpoint for a range of +/- 512 ns (it's not exactly
// equal to nanoseconds, but it's close enough for us).
//
// The gain is how much we have to nudge the DAC to make 1 ns/sec
// phase change-rate. That is, to alter the frequency by 1 ppb.
//
// In this case, however, the gain is too high. A resolution of
// 17E-15 (or even 860E-15) is beyond the hardware we have to
// measure it. Throwing away some of the low resolution bits makes
// the math less problematic (a float can only have so many bits
// of precision).
// Theoretical values:
//#define BIT_REDUCE (8)
//#define GAIN (56070 >> BIT_REDUCE)
// Measured values:
#define BIT_REDUCE (2)
#define GAIN (1466 >> BIT_REDUCE)
//
// In the initial FLL mode, the calculus is slightly different.
// We expect F_CPU counts between each PPS interrupt. Every count off
// represents an error of 1e9 / F_CPU ppb (or ns/sec). At 10 MHz, that's 100.
// When we upshift into PLL, the final trim value from the FLL is the
// base against which the PLL applies adjustment values.
#define START_GAIN (GAIN / 100.0)
//
// What is our loop time constant? We use three different time constants - a
// faster one when we're outside of a certain range, and a slower
// one when we're dialed in.
#define TC_FAST 100
#define TC_MED 1800 // 0.5 hour
#define TC_SLOW 7200 // 2 hours

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
#define LED_DDR_PORT DDRB
#define LED_DDR (_BV(DDRB1) | _BV(DDRB2))

// We don't actually need to read this pin. It triggers a TIMER1_CAPT interrupt.
//#define PPS_PORT PINA
//#define PPS_PIN _BV(PINA7)

// We watch a single bit from the oscillator that indicates whether or not
// it has a physics lock. Doing anything while this bit is high is futile.
#define RDY_PORT PINA
#define OSC_RDY _BV(PINA3)
#define RDY_PU_PORT PUEA
// Oddly enough, the definitions for PUEAx and PUEBx are missing.
#define OSC_RDY_PU _BV(3)

// This is an arbitrary midpoint value. We will attempt to coerce the phase error
// to land at this value.
#define PHASE_ADC_MIDPOINT 512

// Note that if you ever want to parse a longer sentence, be sure to bump this up.
// But an ATTiny841 only has 1/2K of RAM, so...
#define RX_BUF_LEN (96)
#define TX_BUF_LEN (128)

// The start mode watches the cycle count error over a 10 second window, and
// adjusts the DAC until a minute goes by without any errors.
#define MODE_START 0

// The FAST and SLOW modes both use the PI loop on phase alone, but with
// the FAST and SLOW time constants, respectively.
#define MODE_FAST 1
#define MODE_MED 2
#define MODE_SLOW 3

long last_dac_value;
double iTerm;
double trim_value;
double average_phase_error;
double average_pps_error;
unsigned char mode;
unsigned int exit_timer;
unsigned int enter_timer;
volatile unsigned int timer_hibits;
volatile unsigned long pps_count;
volatile unsigned char gps_locked;
volatile unsigned char rx_buf[RX_BUF_LEN];
volatile unsigned char rx_str_len;
volatile unsigned int irq_adc_value;
volatile unsigned long irq_time_span;
unsigned char last_osc_locked;
unsigned char last_gps_locked;
#ifdef DEBUG
volatile unsigned char pdop_buf[5];
volatile unsigned char time_buf[7];
volatile unsigned char date_buf[7];
#endif
#ifdef SERIAL_TX
// serial transmit buffer setup
#ifdef IRQ_DRIVEN_TX
volatile char txbuf[TX_BUF_LEN];
volatile unsigned int txbuf_head, txbuf_tail;
#endif
#endif

// I don't know why my version of AVR libc doesn't have
// fabs() like it's supposed to. Fortunately, it's pretty easy to
// write. It's in the includes, but not the library.
#define fabs MY_fabs
static inline double fabs(double x) { return x<0?-x:x; }

// The oscillator UART isn't done with interrupts. The commands
// are quite brief, so blocking is ok.
static void tx_osc_byte(const unsigned char c) {
  while (!(UCSR1A & _BV(UDRE1))) ; //wdt_reset(); // XXX no WD reset
  wdt_reset();
  _delay_ms(2); // put a gap between chars.
  UDR1 = c;
}

#if 0
static unsigned int rx_osc_byte() {
  unsigned int start_hibits = timer_hibits;
  while(!(UCSR1A & _BV(RXC0))) {
    // wait up to a quarter second, then bail.
    if (timer_hibits - start_hibits > 38) return 0xffff;
    wdt_reset();
  }
  unsigned char out = UDR1;
  return out;
}
#endif

// It's not really a "DAC" per se. It's a serial command
// to the oscillator to set its tuning value.
static void writeDacValue(long value) {
  if (value == last_dac_value) return; // don't do useless writes - results in a glitch for no reason
  last_dac_value = value;

  // Add in the low resolution bits we threw away
  value <<= BIT_REDUCE;

  tx_osc_byte(0x2e); // temporary-write command ID
  tx_osc_byte(0x09); // cmd length LSB
  tx_osc_byte(0x00); // cmd length MSB
  tx_osc_byte(0x27); // cmd checksum
  unsigned char cksum = 0;
  for(int i = 3; i >= 0; i--) {
    unsigned char octet = (unsigned char)(value >> (i * 8));
    cksum ^= octet;
    tx_osc_byte(octet);
  }
  tx_osc_byte(cksum);
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

  irq_time_span = timer_val - last_timer_val;
  last_timer_val = timer_val;

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

#ifdef WAAS
static unsigned char charHex(unsigned char h) {
  if (h > 0xf) return ' ';
  return pgm_read_byte(&(hexes[h]));
}
#endif

static unsigned char hexChar(unsigned char c) {
  if (c >= 'A' && c <= 'F') c += ('a' - 'A'); // make lower case
  const char* outP = strchr_P(hexes, c);
  if (outP == NULL) return 0;
  return (unsigned char)(outP - hexes);
}

#ifdef SERIAL_TX
#ifdef IRQ_DRIVEN_TX
ISR(USART0_UDRE_vect) {
  if (txbuf_head == txbuf_tail) {
    // the transmit queue is empty.
    UCSR0B &= ~_BV(UDRIE0); // disable the TX interrupt
    return;
  }
  UDR0 = txbuf[txbuf_tail];
  if (++txbuf_tail == TX_BUF_LEN) txbuf_tail = 0; // point to the next char
}
#endif

// Note that we're only really going to use the transmit side
// either for diagnostics, or during setup to configure the
// GPS receiver. If the TX buffer fills up, then this method
// will block, which should be avoided.
static void tx_char(const char c) {
#ifdef IRQ_DRIVEN_TX
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
#else
  // wait for TX buf to drain
  while (!(UCSR0A & _BV(UDRE0))) wdt_reset(); 
  UDR0 = c;
#endif
}

static void tx_pstr(const char *buf) {
  for(int i = 0; i < strlen_P(buf); i++)
    tx_char(pgm_read_byte(&(buf[i])));
}

static void tx_str(const char *buf) {
  for(int i = 0; i < strlen(buf); i++)
    tx_char(buf[i]);
}

#endif

// send a NMEA GPS sentence. Takes the sentence
// data between the $ and *, exclusive.
#ifdef WAAS
static void tx_gps(const char *in) {
  tx_char('$');
  tx_pstr(in);
  tx_char('*');
  unsigned char sum = 0;
  for(int i = 0; i < strlen_P(in); i++) sum += pgm_read_byte(&(in[i]));
  tx_char(charHex(sum >> 8));
  tx_char(charHex(sum & 0xf));
  tx_pstr(PSTR("\r\n"));
}
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
  
  char *ptr = (char *)rx_buf;
  if (!strncmp_P((const char*)rx_buf, PSTR("$GPRMC"), 6)) {
    // $GPRMC,172313.000,A,xxxx.xxxx,N,xxxxx.xxxx,W,0.01,180.80,260516,,,D*74\x0d\x0a
#ifdef DEBUG
    ptr = strchr((const char *)ptr, ',');
    if (ptr == NULL) return; // not enough commas
    ptr++; // skip over it
    strncpy((char *)time_buf, ptr, 6);
    time_buf[sizeof(time_buf) - 1] = 0;
    for(i = 0; i < 8; i++) {
      ptr = strchr((const char *)ptr, ',');
      if (ptr == NULL) return; // not enough commas
      ptr++; // skip over it
    }
    strncpy((char *)date_buf, ptr, 6);
    date_buf[sizeof(date_buf) - 1] = 0;
#endif
  } else if (!strncmp_P((const char*)rx_buf, PSTR("$GPGSA"), 6)) {
    // $GPGSA,A,3,02,06,12,24,25,29,,,,,,,1.61,1.33,0.90*01
    for(i = 0; i < 2; i++) {
      ptr = strchr((const char *)ptr, ',');
      if (ptr == NULL) {
        return; // not enough commas
      }
      ptr++; // skip over it
    }
    gps_locked = (*ptr == '3');
#ifdef DEBUG
    // continue parsing to find the PDOP value
    for(i = 2; i < 15; i++) {
      ptr = strchr((const char *)ptr, ',');
      if (ptr == NULL) {
        return; // not enough commas
      }
      ptr++; // skip over it
    }
    unsigned char len = (strchr((const char *)ptr, ',')) - ptr;
    if (len > sizeof(pdop_buf) - 1) len = sizeof(pdop_buf) - 1; // truncate if too long
    memcpy((void*)pdop_buf, ptr, len);
    pdop_buf[len] = 0; // null terminate
#endif
  }
}

static unsigned int mode_to_tc(const unsigned char mode) {
  switch(mode) {
    case MODE_START: // FLL mode, but we use fast averaging times
    case MODE_FAST:
      return TC_FAST;
    case MODE_MED: 
      return TC_MED;
    case MODE_SLOW:
      return TC_SLOW;
    default:
      return 0;
  } 
} 

static void reset_pll() {
  if (mode != MODE_START) {
    // if we're exiting the PLL, then at least take the most recent
    // adjustment value we had and add it back to the trim value for free-running.
    trim_value -= iTerm / mode_to_tc(mode);
  }
  iTerm = 0.;
  average_phase_error = 0.;
  average_pps_error = 0.;
  mode = MODE_START;
  exit_timer = 0;
}

static void downgrade_mode() {
  enter_timer = 100 * mode;
  mode--;
  // translate the iTerm whenever we change the time constant.
  double ratio = ((double)mode_to_tc(mode))/((double)mode_to_tc(mode + 1));
  iTerm *= ratio;
}

void main() {
  // This must be done as early as possible to prevent the watchdog from biting during reset.
  unsigned char mcusr_value = MCUSR;
  MCUSR = 0;
  wdt_enable(WDTO_500MS);

  _delay_ms(250);
  wdt_reset();

  // We use Timer1, USART0, USART1 and the ADC.
  PRR |= _BV(PRTWI) | _BV(PRSPI) | _BV(PRTIM2) | _BV(PRTIM0);

  // set up the serial port baud rates - 9600 for both
  // uses constants defined above in util/setbaud.h
#undef F_CPU
#define F_CPU (8000000UL)
#include <util/setbaud.h>
  UBRR0H = UBRRH_VALUE;
  UBRR0L = UBRRL_VALUE;
  UBRR1H = UBRRH_VALUE;
  UBRR1L = UBRRL_VALUE;
#if USE_2X
  UCSR0A = _BV(U2X0);
  UCSR1A = _BV(U2X1);
#else
  UCSR0A = 0;
  UCSR1A = 0;
#endif
#undef F_CPU
#define F_CPU (10000000UL)

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
  // transmit and receive to the oscillator
  UCSR1B = _BV(RXEN1) | _BV(TXEN1);

  // 8N1 for both
  UCSR0C = _BV(UCSZ00) | _BV(UCSZ01);
  UCSR1C = _BV(UCSZ00) | _BV(UCSZ01);

  // set up the LED port. The two LEDs are outputs.
  DDRB = 0;
  DDRB |= LED_DDR;
  PORTB &= ~(LED0 | LED1); // Turn off both LEDs

  // DDA7 is 0 to make PA7 an input for ICP.
  // DDA6 is unused (other than for programming).
  // DDA3 is 0 to make PA3 an input for OSC_RDRY.
  // The two serial ports will override the DD register.
  // DDA0 will be overridden by the ADC (below).
  // So at the end of all of that...
  DDRA = 0;
  //DDRA |= 0; // pointless
  // Turn on the pull-up on !OSC_RDY
  RDY_PU_PORT |= OSC_RDY_PU;

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

  pps_count = 0;
  mode = MODE_START;
  reset_pll();
  gps_locked = 0;
  rx_str_len = 0;
  last_osc_locked = 0xff; // none of the above
  last_gps_locked = 0xff; // none of the above
#ifdef DEBUG
  // initialize to ""
  *date_buf = 0;
  *time_buf = 0;
#endif
#ifdef SERIAL_TX
  *pdop_buf = 0; // null terminate
#ifdef IRQ_DRIVEN_TX
  txbuf_head = txbuf_tail = 0; // clear the transmit buffer
#endif
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
#ifdef IRQ_DRIVEN_TX
  char done;
  do {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      done = txbuf_head == txbuf_tail;
    }
    wdt_reset();
  } while(!done);
#else
  while (!(UCSR0A & _BV(UDRE0))) wdt_reset(); 
#endif
  UCSR0B &= ~_BV(TXEN0); // turn off the UART TX
#endif
#endif

  sei();

#ifdef DEBUG
  tx_pstr(PSTR("\r\n\r\nSTART\r\n"));
  // one and only one of these should always be printed
  if (mcusr_value & _BV(PORF)) tx_pstr(PSTR("RES_PO\r\n")); // power-on reset
  if (mcusr_value & _BV(EXTRF)) tx_pstr(PSTR("RES_EXT\r\n")); // external reset
  if (mcusr_value & _BV(BORF)) tx_pstr(PSTR("RES_BO\r\n")); // brown-out reset
  if (mcusr_value & _BV(WDRF)) tx_pstr(PSTR("RES_WD\r\n")); // watchdog reset
#endif

  last_dac_value = 0x7fffffffL; // unlikely
  trim_value = 0;
#if 0
  // Read the stored trim_value / DAC value from the oscillator.
  tx_osc_byte(0x2d); // read offset command
  tx_osc_byte(0x04); // length LSB
  tx_osc_byte(0x00); // length MSB
  tx_osc_byte(0x29); // cksum
  wdt_reset();
  if (rx_osc_byte() != 0x2d) goto skip;
  if (rx_osc_byte() != 0x09) goto skip;
  if (rx_osc_byte() != 0x00) goto skip;
  if (rx_osc_byte() != 0x24) goto skip;
  unsigned long tmp = 0;
  unsigned char cksum = 0;
  for(int i = 3; i >= 0; i--) {
    unsigned int val = rx_osc_byte();
    if (val > 0xff) goto skip;
    cksum ^= ((unsigned char)val);
    tmp |= ((unsigned char)val) << (i * 8);
  }
  if (rx_osc_byte() != cksum) goto skip;
  wdt_reset();
  last_dac_value = (long)tmp;
  trim_value = (double)last_dac_value;
skip:

#ifdef DEBUG
  {
    char buf[10];
    // B_TV - boot trim_value
    tx_pstr(PSTR("B_TV="));
    dtostrf(trim_value, 7, 2, buf);
    tx_str(buf);
    tx_pstr(PSTR("\r\n"));
  }
#endif
#endif

  while(1) {
    static unsigned long last_pps_count = 0;
 
    // Pet the dog
    wdt_reset();

    unsigned char osc_locked = (RDY_PORT & OSC_RDY) == 0;

    // with debug firmware, the 0 bit is the GPS status and the 1 bit is
    // whether we've logged that status since the last change or not.
    if (gps_locked != last_gps_locked) {
      last_gps_locked = gps_locked;
      if (gps_locked) {
#ifdef DEBUG
        tx_pstr(PSTR("G_LK\r\n"));
#endif
      } else {
#ifdef DEBUG
        tx_pstr(PSTR("G_UN\r\n"));
#endif
        // Whenever the GPS unlocks, back down one PLL time constant step. We don't
	// attempt to track how long we've held over, but a faster TC means less averaging,
        // which means we'll slew back into correctness faster.
        if (mode > 0) {
          downgrade_mode();
        }
      }
    }
    if (osc_locked != last_osc_locked) {
      last_osc_locked = osc_locked;
      if (osc_locked) {
#ifdef DEBUG
        tx_pstr(PSTR("FE_LK\r\n"));
#endif
      } else {
#ifdef DEBUG
        tx_pstr(PSTR("FE_UN\r\n"));
#endif
        writeDacValue(0);
        reset_pll();
      }
    }

    if (osc_locked && (CLKCR & 0x1f)) {
      // The oscillator is now locked, and we're still running from the 8 MHz
      // internal oscillator. Switch over to clocking from the external
      // oscillator.
#ifdef DEBUG
      tx_pstr(PSTR("\r\nCK_SW\r\n"));
#endif
#ifdef IRQ_DRIVEN_TX
      unsigned char tx_buf_empty = 0;
      do {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          tx_buf_empty = (txbuf_head == txbuf_tail);
        }
        wdt_reset();
      } while(!tx_buf_empty);
#endif
      _delay_ms(20); // clear out the transmit buffer
      wdt_reset();

      CCP = 0xd8;
      CLKCR = _BV(CSTR) | _BV(CKOUTC);

      // And now that we've changed the system clock, we need to fix the USART
      // baud rate dividers.
#undef F_CPU
#define F_CPU (10000000UL)
#include <util/setbaud.h>
      UBRR0H = UBRRH_VALUE;
      UBRR0L = UBRRL_VALUE;
      UBRR1H = UBRRH_VALUE;
      UBRR1L = UBRRL_VALUE;
#if USE_2X
      UCSR0A = _BV(U2X0);
      UCSR1A = _BV(U2X1);
#else
      UCSR0A = 0;
      UCSR1A = 0;
#endif
#ifdef DEBUG
      tx_pstr(PSTR("CK_OK\r\n\r\n"));
#endif
    }

    unsigned char unlocked = (!gps_locked) || (!osc_locked);
    // next, take care of the LEDs.
    // If we're unlocked, then blink them back and forth at 2 Hz.
    // Otherwise, put the binary value of "mode" on the two LEDs.
    if (!unlocked) {
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

#ifdef DEBUG
    {
      char temp_date_buf[7], temp_time_buf[7];
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        strcpy(temp_date_buf, (char *)date_buf);
        strcpy(temp_time_buf, (char *)time_buf);
      }
      if (strlen(temp_date_buf) > 0 && strlen(temp_time_buf) > 0) {
        tx_pstr(PSTR("DT="));
        tx_str(temp_date_buf);
        tx_char(' ');
        tx_str(temp_time_buf);
        tx_pstr(PSTR("\r\n"));
      }
    }
#endif

    if (unlocked) {
#ifdef DEBUG
      // FR - Free Running - GPS or osc is unlocked.
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

    if (labs(intracycle_delta) / (seconds_delta + 1) > (F_CPU / 10000)) { // this would be an error of 1000 ppm - impossible
#ifdef DEBUG
      char buf[16];
      // XXI - an erroneous intracycle delta. A delta of more than 100 ppm is reported, but skipped/ignored.
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

    unsigned int time_constant = mode_to_tc(mode);

    // Since our ADC is 10 bits and the pulse is a microsecond wide we can fudge a little
    // and claim that each ADC count is one nanosecond. So current and average phase error
    // is in nanoseconds and is wrapped.
    int current_phase_error = PHASE_ADC_MIDPOINT - irq_adc_value;

    // This is an approximation of a rolling average, but it's good enough
    // for us, because it should not change very much in 1 second.
    unsigned int filter_time = time_constant / 10;
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
      long dac_value = (long)(DAC_SIGN * trim_value);

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
        tx_pstr(PSTR("\r\nDAC="));
        ltoa((long)trim_value, buf, 10);
        tx_str(buf);
        tx_pstr(PSTR("\r\nET="));
        ltoa(exit_timer, buf, 10);
        tx_str(buf);
        tx_pstr(PSTR("\r\n\r\n"));
      }
#endif
      // If the average PPS error stays under 10 ppb for a minute, transition out to phase discipline.
      if (fabs(average_pps_error) <= 0.1) {
        // Once the PPS error is under control, try to get the phase near zero before starting
        // the PLL. But don't try for longer than 10 minutes before giving up.
        if ((++exit_timer >= 60 && fabs(average_phase_error) <= 20.0) || exit_timer >= 600) {
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
      tx_pstr(PSTR("\r\nM_START\r\n\r\n"));
#endif
      reset_pll();
      continue;
    }

    if (mode != MODE_SLOW) {
      // test for possible upgrade
#ifdef DEBUG
      {
        char buf[8];
        tx_pstr(PSTR("ET="));
        ltoa(exit_timer, buf, 10);
        tx_str(buf);
        tx_pstr(PSTR("\r\n"));
      }
#endif
      if (fabs(average_phase_error) <= 5.0) {
        // stability has to increase with the square of mode
        if (++exit_timer >= 200 * mode * mode) {
          exit_timer = 0;
          mode++;
          time_constant = mode_to_tc(mode);
          // translate the iTerm whenever we change the time constant.
          double ratio = ((double)time_constant)/((double)mode_to_tc(mode - 1));
          iTerm *= ratio;
#ifdef DEBUG
          tx_pstr(PSTR("M_UP\r\n\r\n"));
#endif
        }
      } else {
        exit_timer = 0;
      }
    }
    if (mode != MODE_FAST) {
      // check for downgrade
      if (enter_timer > 0) {
          // if we just downgraded, wait a bit to see if the downgrade "took"
          enter_timer--;
      } else if (fabs(average_phase_error) >= 50.0 * mode) {
          downgrade_mode();
#ifdef DEBUG
          tx_pstr(PSTR("M_DN\r\n\r\n"));
#endif
      }
    }
#ifdef DEBUG
    {
      char buf[8];
      tx_pstr(PSTR("SB="));
      ltoa(intracycle_delta, buf, 10);
      tx_str(buf);
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
    long dac_value = (long)(DAC_SIGN * (trim_value - adj_val) + 0.5);

    writeDacValue(dac_value);

    // If the iTerm is accumulating too much correction, start off-loading
    // some of it to the trim_value.
    double iTerm_modulo = 1000. * time_constant;
    if (fabs(iTerm) > iTerm_modulo) {
#ifdef DEBUG
        tx_pstr(PSTR("RED\r\n"));
#endif
        int sign = (iTerm < 0)?-1:1;
        iTerm -= sign * iTerm_modulo;
        trim_value += sign * 1000;
    }

#ifdef DEBUG
    {
      char buf[8];
      tx_pstr(PSTR("pT="));
      dtostrf(pTerm, 7, 2, buf);
      tx_str(buf);
      tx_pstr(PSTR("\r\niT="));
      dtostrf(iTerm, 7, 2, buf);
      tx_str(buf);
#if 0
      // AV = Adjustment Value - the delta being applied right now to the TP
      tx_pstr(PSTR("\r\nAV="));
      dtostrf(adj_val, 7, 2, buf);
      tx_str(buf);
      // TV = Trim Value - the frequency trim factor in DAC units with conventional
      // sign - larger values -> higher frequency
      tx_pstr(PSTR("\r\nTV="));
      dtostrf(trim_value, 7, 2, buf);
      tx_str(buf);
#endif
      // DAC = DAC Value - the actual value written to the DAC
      tx_pstr(PSTR("\r\nDAC="));
      ltoa(dac_value, buf, 10);
      tx_str(buf);
      // PDOP = Positional Dilution of Precision - the PDOP value reported by the GPS receiver
      tx_pstr(PSTR("\r\nPD="));
      tx_str((const char *)pdop_buf);
      tx_pstr(PSTR("\r\n\r\n"));
      // end of the second.
    }
#endif
  }
}
