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

// This file is for hardware version 4, which is actually board v3.0 and
// beyond. This version of the hardware has an ATMega328P controller, a
// 5 volt digital system, a 10 MHz oscillator, the hardware phase detection
// system, and a timing receiver with quant. error correction messages.

// Fuse settings: lfuse=0xe0, hfuse=0xd7, efuse=0xfc
// ext osc, long startup time, 4.3v brownout, preserve EEPROM

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

// UBRR?_VALUE macros defined here are used below in serial initialization in main()
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

// Define this for the OH300 variant, undef for DOT050V
#define OH300

// Older hardware had the DIN pin of the DAC hooked to MISO. New versions
// have it hooked instead to MOSI, so we can use hardware SPI.
//#define HW_SPI

#ifdef OH300
// The OH300 variant uses the 18 bit AD5680 DAC instead of the AD5061.
#define AD5680
#endif

#if defined(DEBUG)
// define this to include the serial transmit infrastructure at all
#define SERIAL_TX
#endif

#if (defined(HW_SPI) && !defined(__AVR_ATmega328PB__))
#error HW_SPI is only available with the ATMega328PB variant(s).
#endif

// To determine the tuning step - that is, the frequency difference between
// adjacent DAC values (in theory), you multiply the control voltage slope
// of the oscillator (ppm per volt) by the DAC step voltage (volts per step).
// The value you get is ppm per step.
//
// Because of the multitude of different error sources, your actual tuning
// granularity target should be at least a half an order of magnitude higher
// than your desired frequency stability.
//
// The tuning step for the OH300 variant is approximately 3 ppt.
// The DAC output range is 2.7 volts (82% of 3.3v) and the tuning
// range across that is 0.8ppm. The DAC is 18 bits,
// so .8ppm / 262144 is ~3 ppt. The target here is 0.1 ppb.
//
// The tuning step for the DOT050V variant is approximately 200 ppt.
// The DAC output range is 1.65 volts (50% of 3.3v) and the tuning range
// across that is 12.2 ppm (60% of 20ppm). The DAC, again, is 16 bits,
// so that's ~180 ppt. The target here is 1 ppb.
//
// The ADC measures a phase range of 1 us, which we arbitrarily
// devide at a midpoint for a range of +/- 512 ns (it's not exactly
// equal to nanoseconds, but it's close enough for us).
//
// The gain is how much we have to nudge the DAC to make 1 ns/sec
// phase change-rate. That is, to alter the frequency by 1 ppb.
#ifdef OH300
#define GAIN 267
#else
#define GAIN 5
#endif
//
// In the initial FLL mode, the calculus is different.
// We expect F_CPU counts between each PPS interrupt.
// Every count off represents an error of 1e9 / F_CPU ppb (or ns/sec).
// At 10 MHz, that's 100 ppb. We multiply START_GAIN by the error in
// PPB to determine the adjustment to be made to the DAC. When we
// upshift into PLL, the final trim value from the FLL is the base
// against which the PLL applies adjustment values.
#define START_GAIN (GAIN / 100.0)
//
// What is our loop time constant? We use different time constants -
// faster ones when we're outside of a certain range, and slower
// ones when we're dialed in.
#ifdef OH300
#define TC_FAST 100
#define TC_MED 200
#define TC_SLOW 400
#else
#define TC_FAST 50
#define TC_SLOW 100
#endif

#ifdef AD5680
#define DAC_RANGE (0x3ffffl)
#else
#define DAC_RANGE (0xffff)
#endif
#define DAC_MIDPOINT (DAC_RANGE >> 1)

// The damping factor is how much we reduce the influence of the integral
// term - that is, the accumulated error since the last loop startup
// (that is, the last time the FLL determined the starting DAC value).
#define DAMPING 1.75

// our DAC has a positive slope - higher values mean higher frequencies.
// If you have an inverse DAC slope, set this to -1.
#define DAC_SIGN (1)

// The quantization error factor is in ns, but our ADC phase discriminator
// values aren't precise. the QE_COMPENSATION is the quantization error
// scaling value that we apply to the raw QE value to add it to the
// phase discriminator value.
#define QE_COMPENSATION 1.5

#define LED_PORT PORTD
#define LED0 _BV(PORTD2)
#define LED1 _BV(PORTD3)
#define LED_DDR DDRD
#define LED_DDR_MSK (_BV(DDD2) | _BV(DDD3))

// We don't actually need to read this pin. It triggers a TIMER1_CAPT interrupt.
//#define PPS_PORT PINB
//#define PPS_PIN _BV(PINB0)

#define DAC_PORT PORTB
#ifdef HW_SPI
#define DAC_CS _BV(PORTB2)
#else
#define DAC_CS _BV(PORTB1)
#define DAC_DO _BV(PORTB4)
#define DAC_CLK _BV(PORTB5)
#endif
#define DAC_DDR DDRB
#ifdef HW_SPI
// MOSI and SCK are handled by SPI, but must still be
// set to output pins.
#define DAC_DDR_MSK (_BV(DDB2) | _BV(DDB3) | _BV(DDB5))
#else
#define DAC_DDR_MSK (_BV(DDB1) | _BV(DDB4) | _BV(DDB5))
#endif

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
#ifdef TC_MED
#define MODE_MED 2
#define MODE_SLOW 3
#else
#define MODE_SLOW 2
#endif

unsigned long last_dac_value;
double iTerm;
double trim_value;
double average_phase_error;
double average_pps_error;
unsigned char mode;
unsigned char last_gps_locked;
unsigned int exit_timer;
volatile unsigned int timer_hibits;
volatile unsigned long pps_count;
volatile unsigned char gps_locked;
volatile unsigned char rx_buf[RX_BUF_LEN];
volatile unsigned char rx_str_len;
volatile unsigned int irq_adc_value;
volatile unsigned long irq_time_span;
#ifdef DEBUG
volatile unsigned char pdop_buf[5];
volatile unsigned char pps_err_buf[5];
volatile unsigned char time_buf[7];
volatile unsigned char date_buf[7];
#endif
#ifdef SERIAL_TX
// serial transmit buffer setup
volatile char txbuf[TX_BUF_LEN];
volatile unsigned int txbuf_head, txbuf_tail;
#endif

// For the 5680, the data format is 4 bits of 0, 18 bits of big-endian data, and two bits of 0.
// For the 5061, the data format is 8 bits of 0 (the bottom two are shut-down bits that we always
// set to 0), then 16 bits of big-endian data.

// Data is clocked on the falling
// edge of the clock pin, and CS must be held low the
// whole time. The output voltage will slew on the rising
// edge of CS. The minimum time between clock transition
// is way faster than our clock speed, so we don't need
// to perform any delays.

static void writeDacValue(unsigned long value) {
  // Limit the value to the actual range of the DAC
  value &= DAC_RANGE;

  if (value == last_dac_value) return; // don't do useless writes - results in a glitch for no reason
  last_dac_value = value;

#ifdef AD5680
  // The bottom two bits on the 5680A are "don't care". The actual data
  // starts at bit 3.
  value <<= 2;
#endif
  // This is the point where we'd OR in any control bits, but there are none we want.

#ifdef HW_SPI
  // Now we start - Assert !CS
  DAC_PORT &= ~DAC_CS;

  // send it!
  SPDR0 = (unsigned char)(value >> 16);
  while (!(SPSR0 & _BV(SPIF0))) ;
  SPDR0 = (unsigned char)(value >> 8);
  while (!(SPSR0 & _BV(SPIF0))) ;
  SPDR0 = (unsigned char)(value >> 0);
  while (!(SPSR0 & _BV(SPIF0))) ;

#else
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
#endif
  // Raise !CS to end the transfer, which also slews the DAC output.
  DAC_PORT |= DAC_CS;
}

// Timer 1's TCNT register is the low bits. They're ORed
// onto this to make an unsigned long. That gives us more
// than 400 seconds between full overflows (at 10 MHz).
ISR(TIMER1_OVF_vect) {
  timer_hibits++;
}

// When a capture occurs, we figure out how many clock ticks occurred
// since the last one, and we read the phase position (via the ADC).
// We save those as (volatile) globals and then increment a pps counter
// value just so the main loop will know that it happened.
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

  pps_err_buf[0] = 0; // The *next* sawtooth msg applies to *this* pps.

  pps_count++;

}

static inline void handleGPS();

#ifdef __AVR_ATmega328PB__
ISR(USART0_RX_vect) {
#else
ISR(USART_RX_vect) {
#endif
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
#ifdef __AVR_ATmega328PB__
ISR(USART0_UDRE_vect) {
#else
ISR(USART_UDRE_vect) {
#endif
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

#endif

// A repeated strchr, effectively.
static char* skip_commas(char *ptr, int num) {
  for(int i = 0; i < num; i++) {
    ptr = strchr((const char *)ptr, ',');
    if (ptr == NULL) return NULL; // not enough commas
    ptr++; // skip over it
  }
  return ptr;
}

// When this method is called, we've just received
// a complete NEMA GPS sentence.
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
    ptr = skip_commas(ptr, 1);
    if (ptr == NULL) return; // not enough commas
    strncpy((char *)time_buf, ptr, 6);
    time_buf[sizeof(time_buf) - 1] = 0;
    ptr = skip_commas(ptr, 8);
    if (ptr == NULL) return; // not enough commas
    strncpy((char *)date_buf, ptr, 6);
    date_buf[sizeof(date_buf) - 1] = 0;
#endif
  } else if (!strncmp_P((const char*)rx_buf, PSTR("$PSTI,00"), 8)) {
    // $PSTI,00,2,0,5.8,,*3F
    ptr = skip_commas(ptr, 4);
    if (ptr == NULL) return; // not enough commas
    unsigned char len = (strchr((const char *)ptr, ',')) - ptr;
    if (len > sizeof(pdop_buf) - 1) len = sizeof(pdop_buf) - 1; // truncate if too long
    memcpy((void*)pps_err_buf, ptr, len);
    pps_err_buf[len] = 0; // null terminate
  } else if (!strncmp_P((const char*)rx_buf, PSTR("$GPGSA"), 6)) {
    // $GPGSA,A,3,02,06,12,24,25,29,,,,,,,1.61,1.33,0.90*01
    ptr = skip_commas(ptr, 2);
    if (ptr == NULL) return; // not enough commas
    gps_locked = (*ptr == '3' || *ptr == '2');
#ifdef DEBUG
    // continue parsing to find the PDOP value
    ptr = skip_commas(ptr, 13);
    if (ptr == NULL) return; // not enough commas
    unsigned char len = (strchr((const char *)ptr, ',')) - ptr;
    if (len > sizeof(pdop_buf) - 1) len = sizeof(pdop_buf) - 1; // truncate if too long
    memcpy((void*)pdop_buf, ptr, len);
    pdop_buf[len] = 0; // null terminate
#endif
  }
}

// Optimization beyond O2 turns this into a jump table, which is a step backwards
// on a Harvard machine.
static unsigned int __attribute__((optimize("O1"))) mode_to_tc(const unsigned char mode) {
  switch(mode) {
    case MODE_START: // FLL mode, but we use fast averaging times
    case MODE_FAST:
      return TC_FAST;
#ifdef TC_MED
    case MODE_MED: 
      return TC_MED;
#endif
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
  mode--;
  exit_timer = 0;
  // translate the iTerm whenever we change the time constant.
  double ratio = ((double)mode_to_tc(mode))/((double)mode_to_tc(mode + 1));
  iTerm *= ratio;
}

// main() is void, and we never return from it.
void __ATTR_NORETURN__ main() {
  // This must be done as early as possible to prevent the watchdog from biting during reset.
  unsigned char mcusr_value = MCUSR;
  MCUSR = 0;
  wdt_enable(WDTO_500MS);

  // We use Timer1, USART0 and the ADC.
#ifdef __AVR_ATmega328PB__
#ifdef HW_SPI
  PRR0 |= _BV(PRTIM0) | _BV(PRTIM2) | _BV(PRTWI0) | _BV(PRUSART1);
#else
  PRR0 |= _BV(PRSPI0) | _BV(PRTIM0) | _BV(PRTIM2) | _BV(PRTWI0) | _BV(PRUSART1);
#endif
  PRR1 |= _BV(PRTIM3) | _BV(PRSPI1) | _BV(PRTIM4) | _BV(PRPTC) | _BV(PRTWI1);
#else
  PRR |= _BV(PRTWI) | _BV(PRSPI) | _BV(PRTIM2) | _BV(PRTIM0);
#endif

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
  PORTB &= ~(LED0 | LED1); // Turn off both LEDs
#ifdef HW_SPI
  // Configure SPI0. Master, fastest speed possible, inverted clock.
  SPCR0 = _BV(SPE0) | _BV(MSTR0) | _BV(CPOL0);
  SPSR0 = _BV(SPI2X0);
#endif
  LED_DDR = 0;
  LED_DDR |= LED_DDR_MSK;

  // DDA7 is 0 to make PA7 an input for ICP
  // set up the DAC port
  // set CS high on the DAC *before* setting the direction. 
  DAC_PORT |= DAC_CS;
  DAC_DDR = 0;
  DAC_DDR |= DAC_DDR_MSK;

  // Set up timer1
  TCCR1A = 0; // Normal mode
  TCCR1B = _BV(ICES1) | _BV(CS10); // No noise reduction, rising edge capture, no pre-scale.
  TIMSK1 = _BV(ICIE1) | _BV(TOIE1); // Interrupt on overflow and capture
  TCNT1 = 0; // clear the counter.
  timer_hibits = 0;

  // Set up the ADC
  ACSR = _BV(ACD); // Turn off the analog comparators
  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1); // ADC on, clock scale = 64
  ADMUX = _BV(REFS0) | _BV(REFS1); // 1.1V is ref, ADC0 is the input
  DIDR0 = _BV(ADC0D); // disable digital I/O on pin A0.

  last_dac_value = 0xffffffff; // none-of-the-above value
  pps_count = 0;
  mode = MODE_START;
  reset_pll();
  gps_locked = 0;
  last_gps_locked = 0xff; // none of the above
  rx_str_len = 0;
#ifdef DEBUG
  *pdop_buf = 0; // null terminate
  *pps_err_buf = 0;
  *time_buf = 0;
  *date_buf = 0;
#endif
#ifdef SERIAL_TX
  txbuf_head = txbuf_tail = 0; // clear the transmit buffer
#endif

#ifdef DEBUG
  tx_pstr(PSTR("\r\n\r\nSTART\r\n"));
  // one and only one of these should always be printed
  if (mcusr_value & _BV(PORF)) tx_pstr(PSTR("RES_PO\r\n")); // power-on reset
  if (mcusr_value & _BV(EXTRF)) tx_pstr(PSTR("RES_EXT\r\n")); // external reset
  if (mcusr_value & _BV(BORF)) tx_pstr(PSTR("RES_BO\r\n")); // brown-out reset
  if (mcusr_value & _BV(WDRF)) tx_pstr(PSTR("RES_WD\r\n")); // watchdog reset
#endif

  // the default value of the DAC is midpoint, so nothing needs to be done.
  trim_value = 0.;

  sei();

  while(1) {
    static unsigned long last_pps_count = 0;
 
    // Pet the dog
    wdt_reset();

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

    // next, take care of the LEDs.
    // If gps_status is 0, then blink them back and forth at 2 Hz.
    // Otherwise, put the binary value of "mode" on the two LEDs.
    if (gps_locked) {
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

    // If we haven't had a PPS event and gotten a quant error value since we were last here, we're done.
    if (last_pps_count == pps_count || *pps_err_buf == 0) continue;
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

    if (!gps_locked) {
#ifdef DEBUG
      // FR - Free Running - GPS is unlocked.
      tx_pstr(PSTR("FR\r\n\r\n"));
#endif
      pps_err_buf[0] = 0; // clear it out.
      continue;
    }

    double pps_err;
    {
      char temp[5];
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        strcpy(temp, (const char*) pps_err_buf); // it's volatile, so make a copy atomically.
        pps_err_buf[0] = 0; // clear it out.
      }
#ifdef DEBUG
      tx_pstr(PSTR("QE="));
      tx_str(temp);
      tx_pstr(PSTR("\r\n"));
#endif
      pps_err = atof(temp);
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

    if (labs(intracycle_delta) / (seconds_delta + 1) > (F_CPU / 100000)) { // this would be an error of 100 ppm - impossible
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

    // the time constant in START mode is the same as FAST mode.
    unsigned int time_constant = mode_to_tc(mode);

    // Since our ADC is 10 bits and the pulse is a microsecond wide we can fudge a little
    // and claim that each ADC count is one nanosecond. So current and average phase error
    // is in nanoseconds and is wrapped.
    int current_phase_error = PHASE_ADC_MIDPOINT - irq_adc_value;
#ifdef DEBUG
    {
      char buf[8];
      tx_pstr(PSTR("RPE="));
      itoa(current_phase_error, buf, 10);
      tx_str(buf);
      tx_pstr(PSTR("\r\n"));
    }
#endif
    current_phase_error += (int)((QE_COMPENSATION * pps_err) + 0.5); // quant error correction is in ns. Round to nearest

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
      unsigned long dac_value = (long)(DAC_SIGN * trim_value) + DAC_MIDPOINT;

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
        tx_pstr(PSTR("\r\nDAC=0x"));
        ltoa(dac_value, buf, 16);
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
        // the PLL. But don't try for longer than 20 minutes before giving up.
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

    // Test for possible upgrade if we're not maxed out
    if (mode != MODE_SLOW) {
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
        if (++exit_timer >= 200 * mode * mode) {
          mode++;
          time_constant = mode_to_tc(mode);
          exit_timer = 0;
          // translate the iTerm whenever we change the time constant.
          double ratio = ((double)mode_to_tc(mode))/((double)mode_to_tc(mode - 1));
          iTerm *= ratio;
#ifdef DEBUG
          tx_pstr(PSTR("M_UP\r\n\r\n"));
#endif
        }
      } else {
        exit_timer = 0;
      }
    } else if (mode != MODE_FAST) { // Test for possible downgrade
      if (fabs(average_phase_error) >= 50.0 * mode) {
          downgrade_mode();
          time_constant = mode_to_tc(mode);
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
    unsigned long dac_value = (long)(DAC_SIGN * (trim_value - adj_val) + 0.5) + DAC_MIDPOINT;

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
        trim_value -= sign * 1000;
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
      ltoa(dac_value, buf, 16);
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
