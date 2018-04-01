/*

    GPS Disciplined Oscillator sketch
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

// This file is the grand unified firmware for both the FEI board and original
// SPI DAC hardware. This version of the hardware has an ATXMega32E5 controller,
// a 5 volt digital system, the hardware phase detection system, and either the
// power and interface systems for an FE-56x0A rubidium oscillator or FE-405B
// OCXO, or either an AD5061 16 bit or AD5680 18 bit DAT whose output is the
// EFC for a 3.3v TCXO or OCXO.

// Serial DAC boards have a button that sends the command to store the current
// EFC setting as the power-up default. That variant also supports using one
// of the pins as an !OSC_RDY input. The oscillator is not "trusted" until
// that line is asserted.

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

#define F_CPU (30000000UL)

#include <util/delay.h>

// Serial port baud rate constants - 9600 baud at 30 MHz.
#define BSEL (777)
#define BSCALE (-2)

/************************************************
 *
 * OPTIONS
 *
 */

//Pick your oscillator:

//#include "fe405b.h"
//#include "fe56x0a.h"
#include "oh300.h"
//#include "ecoc2522.h"
//#include "dot050v.h"

// The include files define SPI_DAC (or not), AD5680 (or not), GAIN, TC_FAST, TC_SLOW and optionally TC_MED

#ifdef SPI_DAC
#ifdef AD5680
#define DAC_RANGE (0x3ffffl)
#else
#define DAC_RANGE (0xffff)
#endif
#define DAC_MIDPOINT (DAC_RANGE >> 1)
#endif

// Turn on debug logging. There's really no longer any point to turning
// this off.
#define DEBUG

#if defined(DEBUG)
// define this to include the serial transmit infrastructure at all
#define SERIAL_TX
#endif

// In the initial FLL mode, the calculus is slightly different.
// We expect F_CPU counts between each PPS interrupt. Every count off
// represents an error of 1e9 / F_CPU ppb (or ns/sec). At 10 MHz, that's 100.
// When we upshift into PLL, the final trim value from the FLL is the
// base against which the PLL applies adjustment values.
#define START_GAIN (GAIN / 100.0)

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
// phase discriminator value. The ATTiny841 doesn't have enough flash
// for the extra code.
#define QE_COMPENSATION 1.5

// Port A has the phase discriminator output (for ADC) on 1.
#define PD_PIN _BV(1)

// Port C has GPS PPS on pin 0, GPS serial on 2 and 3 and...
#define PPS_BIT _BV(0)
#define SER_RX _BV(2)
#define SER_TX _BV(3)

#ifdef SPI_DAC
// pin 4 is the !SS line
#define DAC_CS _BV(4)
// pin 5 is the SPI clock
#define SPI_CLK _BV(5)
// pin 6 is MISO (but we don't care)
#define SPI_MISO _BV(6)
// pin 7 is MOSI
#define SPI_MOSI _BV(7)
#else
// if it's a serial DAC and it's NOT the FE405 firmware, then there's
// the OSC_RDY line to look at.
#ifndef FE405
// We watch a single bit from the oscillator that indicates whether or not
// it has a physics lock. Doing anything while this bit is high is futile.
// (be sure to pull it up). It's port C pin 7.
#define OSC_RDY _BV(7)
#endif
#endif

// Port D has osc serial on 2 and 3 and...
#define LED0 _BV(6)
#define LED1 _BV(7)
// Serial DAC units have a switch
#ifndef SPI_DAC
#define SW_BIT _BV(5)
#endif

// This is about 500 ms
#define BUTTON_BLINK_TICKS (F_CPU / 2)

// We want this to be around 50 ms.
#define DEBOUNCE_TICKS (F_CPU / 20)

// This is an arbitrary midpoint value. We will attempt to coerce the phase error
// to land at this value.
#ifdef FE405
// The 405 runs at 15 MHz and wraps too low, so we have to shove the sweet spot down a little.
#define PHASE_ADC_MIDPOINT 512
#else
#define PHASE_ADC_MIDPOINT 1024
#endif

// Note that if you ever want to parse a longer sentence, be sure to bump this up.
#define RX_BUF_LEN (96)
#ifdef SERIAL_TX
#define TX_BUF_LEN (128)
#endif

// The start mode watches the cycle count error over a 10 second window, and
// adjusts the DAC until a minute goes by without any errors.
#define MODE_START 0

// The FAST, MED and SLOW modes both use the PI loop on phase alone, but with
// the FAST, MED and SLOW time constants, respectively. MED is optional, depending
// on the oscillator
#define MODE_FAST 1
#ifdef TC_MED
#define MODE_MED 2
#define MODE_SLOW 3
#else
#define MODE_SLOW 2
#endif

double iTerm;
double trim_value;
double average_phase_error;
double average_pps_error;
unsigned char mode;
unsigned int exit_timer;
unsigned int enter_timer;
volatile unsigned long pps_count;
volatile unsigned char gps_locked;
volatile unsigned char rx_buf[RX_BUF_LEN];
volatile unsigned char rx_str_len;
volatile unsigned int irq_adc_value;
volatile unsigned long irq_time_span;
unsigned char last_osc_locked;
unsigned char last_gps_locked;
volatile unsigned char pps_err_buf[5];
#ifdef DEBUG
volatile unsigned char pdop_buf[5];
volatile unsigned char time_buf[7];
volatile unsigned char date_buf[7];
#endif
#ifdef SERIAL_TX
// serial transmit buffer setup
volatile char txbuf[TX_BUF_LEN];
volatile unsigned int txbuf_head, txbuf_tail;
#endif
#ifdef SW_BIT
unsigned long debounce_time;
unsigned char button_down;
unsigned long button_blink_time;
#endif

static inline unsigned long timer_value() __attribute__ ((always_inline));
static inline unsigned long timer_value() {
  // We've configured event block 0-3 for timer C 4/5 capture.
  // CCA causes an interrupt, but CCB doesn't, so use a
  // synthetic capture to grab the current value. This avoids
  // having to deal with overflow propagation issues.
  EVSYS.STROBE = _BV(1); // event channel 1
  while(!((TCC4.INTFLAGS & TC4_CCBIF_bm)) && ((TCC5.INTFLAGS & TC5_CCBIF_bm))) ; // wait for both words
  unsigned long out = (((unsigned long)TCC5.CCB) << 16) | TCC4.CCB;
  TCC4.INTFLAGS = TC4_CCBIF_bm; // XXX why is this necessary?
  TCC5.INTFLAGS = TC5_CCBIF_bm;
  return out;
}

// same as _delay_ms(), but pet the watchdog doing it
static void inline do_delay_ms(unsigned int time) {
	while(time-- > 0) {
		wdt_reset();
		_delay_ms(1);
	}
}

#ifndef SPI_DAC
// The oscillator UART isn't done with interrupts. The commands
// are quite brief, so blocking is ok, provided you pet the watchdogs.
static void tx_osc_byte(const unsigned char c) {
  while (!(USARTD0.STATUS & USART_DREIF_bm)) ; //wdt_reset(); // XXX no WD reset
  wdt_reset();
  do_delay_ms(2); // put a gap between chars.
  USARTD0.DATA = c;
}

#if 0
static unsigned int rx_osc_byte() {
  unsigned long start_time = timer_value();
  while(!(USARTD0.STATUS & USART_STATUS_RXCIF_bm)) {
    // wait up to a quarter second, then bail.
    if ((timer_value() - start_time) > (F_CPU / 4)) return 0xffff;
    wdt_reset();
  }
  unsigned char out = UDR1;
  return out;
}
#endif
#endif

static void writeDacValue(long value, unsigned char non_volatile) {
  static long last_dac_value = 0x7fffffffL; // "none of the above"
  if (value == last_dac_value) return; // don't do useless writes - results in a glitch for no reason
  last_dac_value = value;

#ifdef SPI_DAC
  // Limit the value to the actual range of the DAC
  value &= DAC_RANGE;

#ifdef AD5680
  // The bottom two bits on the 5680A are "don't care". The actual data
  // starts at bit 3.
  value <<= 2;
#endif
  // This is the point where we'd OR in any control bits, but there are none we want.

  // Now we start - Assert !CS
  PORTC.OUTCLR = DAC_CS;

  // send it!
  SPIC.DATA = (unsigned char)(value >> 16);
  while (!(SPIC.STATUS & SPI_IF_bm)) ;
  SPIC.DATA = (unsigned char)(value >> 8);
  while (!(SPIC.STATUS & SPI_IF_bm)) ;
  SPIC.DATA = (unsigned char)(value >> 0);
  while (!(SPIC.STATUS & SPI_IF_bm)) ;

  // Raise !CS to end the transfer, which also slews the DAC output.
  PORTC.OUTSET = DAC_CS;

#else
  // It's not really a "DAC" per se. It's a serial command
  // to the oscillator to set its tuning value.

  // Add in the low resolution bits we threw away
  value <<= BIT_REDUCE;

  tx_osc_byte(non_volatile?0x2c:0x2e); // command ID
  tx_osc_byte(0x09); // cmd length LSB
  tx_osc_byte(0x00); // cmd length MSB
  tx_osc_byte(non_volatile?0x25:0x27); // cmd checksum
  unsigned char cksum = 0;
  for(int i = 3; i >= 0; i--) {
    unsigned char octet = (unsigned char)(value >> (i * 8));
    cksum ^= octet;
    tx_osc_byte(octet);
  }
  tx_osc_byte(cksum);
#endif
}

// When a capture interrupt occurs, we perform an ADC conversion
// to read the phase discriminator, and write that value and the
// current cycle count delta out for the main loop. Then we increment
// a PPS counter so they notice.
ISR(TCC5_CCA_vect) {
  static unsigned long last_timer_val;

  while(!((TCC4.INTFLAGS & TC4_CCAIF_bm)) && ((TCC5.INTFLAGS & TC5_CCAIF_bm))) ; // wait for both words
  unsigned long timer_val = (((unsigned long)TCC5.CCA) << 16) | TCC4.CCA;
  TCC4.INTFLAGS = TC4_CCAIF_bm; // XXX why is this necessary?
  TCC5.INTFLAGS = TC5_CCAIF_bm;

  // Clear the complete flag
  ADCA.CH0.INTFLAGS = ADC_CH_IF_bm;
  // start ADC operation
  ADCA.CH0.CTRL |= ADC_CH_START_bm;
  // wait for ADC to finish
  while(!(ADCA.CH0.INTFLAGS & ADC_CH_IF_bm)) ; // don't pet the watchdog - this should never take that long.
  unsigned int adc_value = ADCA.CH0.RES;

  irq_adc_value = adc_value;

  irq_time_span = timer_val - last_timer_val;
  last_timer_val = timer_val;

  pps_err_buf[0] = 0; // The *next* sawtooth msg applies to *this* pps.

  pps_count++;

}

static inline void handleGPS();

ISR(USARTC0_RXC_vect) {
  unsigned char rx_char = USARTC0.DATA;
  
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

static unsigned char hexChar(unsigned char c) {
  if (c >= 'A' && c <= 'F') c += ('a' - 'A'); // make lower case
  const char* outP = strchr_P(hexes, c);
  if (outP == NULL) return 0;
  return (unsigned char)(outP - hexes);
}

#ifdef SERIAL_TX
ISR(USARTC0_DRE_vect) {
  if (txbuf_head == txbuf_tail) {
    // the transmit queue is empty.
    USARTC0.CTRLA &= ~(USART_DREINTLVL_gm); // disable the TX interrupt
    //USARTC0.CTRLA |= USART_DREINTLVL_OFF_gc; // redundant - off is a zero value
    return;
  }
  USARTC0.DATA = txbuf[txbuf_tail];
  if (++txbuf_tail == TX_BUF_LEN) txbuf_tail = 0; // point to the next char
}

// Note that we're only really going to use the transmit side
// for diagnostics. If the TX buffer fills up, then this method
// will block, which should be avoided.
static void tx_char(const char c) {
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
    //USARTC0.CTRLA &= ~(USART_DREINTLVL_gm); // redundant - it was already zero
    USARTC0.CTRLA |= USART_DREINTLVL_LO_gc; // enable the DRE interrupt
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

static void reset_pll();

static char* skip_commas(char *ptr, int num) {
  for(int i = 0; i < num; i++) {
    ptr = strchr((const char *)ptr, ',');
    if (ptr == NULL) return NULL; // not enough commas
    ptr++; // skip over it
  }
  return ptr;
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
  } else if (!strncmp_P((const char*)rx_buf, PSTR("$PSTI,00"), 8)) {
    // $PSTI,00,2,0,5.8,,*3F
    ptr = skip_commas(ptr, 4);
    if (ptr == NULL) return; // not enough commas
    unsigned char len = (strchr((const char *)ptr, ',')) - ptr;
    if (len > sizeof(pps_err_buf) - 1) len = sizeof(pps_err_buf) - 1; // truncate if too long
    memcpy((void*)pps_err_buf, ptr, len);
    pps_err_buf[len] = 0; // null terminate
  }
}

// Optimization beyond O2 turns this into a jump table, which is a step backwards
// on a Harvard machine.
static unsigned __attribute__((optimize("O1"))) int mode_to_tc(const unsigned char mode) {
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
  enter_timer = 100 * mode;
  mode--;
  // translate the iTerm whenever we change the time constant.
  double ratio = ((double)mode_to_tc(mode))/((double)mode_to_tc(mode + 1));
  iTerm *= ratio;
}

#ifdef SW_BIT
static unsigned char check_buttons() {
	unsigned long now = timer_value();
        if (debounce_time != 0 && now - debounce_time < DEBOUNCE_TICKS) {
                // We don't pay any attention to the buttons during debounce time.
                return 0;
        } else {
                debounce_time = 0; // debounce is over
        }
        unsigned char status = PORTD.IN & SW_BIT;
        status ^= SW_BIT; // invert the buttons - 0 means down.
        if (!((button_down == 0) ^ (status == 0))) return 0; // either the button is still up, or still down

        // Something *changed*, which means we must now start a debounce interval.
        debounce_time = now;
        if (!debounce_time) debounce_time++; // it's not allowed to be zero

	button_down = status;
	return status;
}
#endif

void __ATTR_NORETURN__ main() {
  unsigned char reset_status = RST.STATUS; // grab this early and save it for later
  RST.STATUS = 0xff; // clear them all out

  // set up the watchdog
  //wdt_enable(WDTO_1S); // This is broken on XMegas.
  // This replacement code doesn't disable interrupts (but they're not on now anyway)
  _PROTECTED_WRITE(WDT.CTRL, WDT_PER_256CLK_gc | WDT_ENABLE_bm | WDT_CEN_bm);
  while(WDT.STATUS & WDT_SYNCBUSY_bm) ; // wait for it to take
  // We don't want a windowed watchdog.
  _PROTECTED_WRITE(WDT.WINCTRL, WDT_WCEN_bm);
  while(WDT.STATUS & WDT_SYNCBUSY_bm) ; // wait for it to take

  // At boot, configure the internal RC oscillator for 30 MHz.
  // Run the CPU at 30 MHz using the RC osc and DFLL set for 30 MHz.
  // In principle, we could switch right to the external oscillator
  // on non-Rubidium devices, but just in case there is ever any
  // delay or other wonkiness on startup, we can just put it off
  // to later.
  OSC.CTRL |= OSC_RC32MEN_bm | OSC_RC32KEN_bm;
  while(!(OSC.STATUS & (OSC_RC32KRDY_bm | OSC_RC32MRDY_bm))) ; // wait for them.

  // pick the internel 32 kHz reference for the DFLL.
  OSC.DFLLCTRL = OSC_RC32MCREF_RC32K_gc;
  NVM.CMD = NVM_CMD_READ_CALIB_ROW_gc;
  DFLLRC32M.CALA = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, RCOSC32MA));
  DFLLRC32M.CALB = (unsigned char)((((unsigned int)pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, RCOSC32M))) * 15) / 16); // scale 32 -> 30
  unsigned char adcacal0 = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0));
  unsigned char adcacal1 = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1));
  NVM.CMD = NVM_CMD_NO_OPERATION_gc;
  // Set the target frequency to 30 MHz.
  DFLLRC32M.COMP1 = (F_CPU / 1024) & 0xff;
  DFLLRC32M.COMP2 = (F_CPU / 1024) >> 8;
  DFLLRC32M.CTRL = DFLL_ENABLE_bm; //enable the FLL.

  _PROTECTED_WRITE(CLK.CTRL, CLK_SCLKSEL_RC32M_gc); // switch to it
  OSC.CTRL &= ~(OSC_RC2MEN_bm); // we're done with the 2 MHz osc.

  do_delay_ms(250); // let the FLL do its magic.

  // Leave on only the parts of the chip we use.
  PR.PRGEN = PR_XCL_bm | PR_RTC_bm | PR_EDMA_bm;
  PR.PRPA = PR_DAC_bm | PR_AC_bm;
#ifdef SPI_DAC
  PR.PRPC = PR_TWI_bm | PR_HIRES_bm;
  PR.PRPD = PR_USART0_bm | PR_TC5_bm;
#else
  PR.PRPC = PR_TWI_bm | PR_SPI_bm | PR_HIRES_bm;
  PR.PRPD = PR_TC5_bm;
#endif

  // Event 0 is PPS - it causes a timer capture.
  EVSYS.CH0MUX = EVSYS_CHMUX_PORTC_PIN0_gc;
  EVSYS.CH0CTRL = 0;
  // Event 4 is a carry from timer C4 to timer C5
  EVSYS.CH4MUX = EVSYS_CHMUX_TCC4_OVF_gc;
  EVSYS.CH4CTRL = 0;

  PORTA.DIRCLR = PD_PIN; // the phase discriminator output is an input for us
  PORTA.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc; // disable digital input.

  PORTC.DIRCLR = _BV(0) | SER_RX; // PPS and GPS serial input
  PORTC.PIN0CTRL = PORT_ISC_RISING_gc; // send an event on the PPS rising edge
#ifdef SERIAL_TX
  PORTC.OUTSET = SER_TX; // serial output defaults to high
  PORTC.DIRSET = SER_TX; // serial output
#endif

  PORTD.DIRSET = LED0 | LED1; // output on LEDs

#ifdef OSC_RDY
  PORTC.DIRCLR = OSC_RDY;
  PORTC.PIN7CTRL = PORT_OPC_PULLUP_gc; // pull-up on OSC_RDY
#endif

#ifdef SW_BIT
  PORTD.DIRCLR = SW_BIT; // input on switch
  PORTD.PIN5CTRL = PORT_OPC_PULLUP_gc; // pull-up on switch
#endif

#ifdef SPI_DAC
  PORTC.OUTSET = DAC_CS; // start with it high
  PORTC.DIRSET = DAC_CS | SPI_MOSI | SPI_CLK; // MOSI, SCK and DAC_CS are outputs
  //PORTC.DIRCLR = SPI_MISO; // MISO is ostensibly an input, but we really don't care.
#else
  // serial DAC is on USARTD0.
  PORTD.DIRCLR = SER_RX; // input on serial in
  PORTD.OUTSET = SER_TX; // output on serial out defaults to high
  PORTD.DIRSET = SER_TX; // output on serial out
#endif

  USARTC0.CTRLA = USART_RXCINTLVL_LO_gc; // interrupt on receive (DRE set later).
#ifdef SERIAL_TX
  USARTC0.CTRLB = USART_RXEN_bm | USART_TXEN_bm; // enable transmitter and receiver
#else
  USARTC0.CTRLB = USART_RXEN_bm; // enable receiver
#endif
  USARTC0.CTRLC = USART_CHSIZE_8BIT_gc; // 8N1 async.
  USARTC0.CTRLD = 0; // no special processing
  USARTC0.BAUDCTRLA = (BSEL & 0xff);
  USARTC0.BAUDCTRLB = (BSCALE << USART_BSCALE_gp) | (BSEL >> 8);

#ifdef SPI_DAC
  SPIC.CTRL = SPI_CLK2X_bm | SPI_ENABLE_bm | SPI_MASTER_bm | SPI_MODE_0_gc | SPI_PRESCALER_DIV4_gc; // master, enable, mode 0 and FAST
  SPIC.INTCTRL = SPI_INTLVL_OFF_gc; // no interrupts
  SPIC.CTRLB = SPI_BUFMODE_OFF_gc | SPI_SSD_bm; // no buffering, disable the !SS pin as an abort input
#else
  // serial D is oscillator I/O. It's polled (because it's not used much).
  USARTD0.CTRLA = 0; // all interrupts off
  USARTD0.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
  USARTD0.CTRLC = USART_CHSIZE_8BIT_gc; // 8N1 async.
  USARTD0.CTRLD = 0; // no special processing
  USARTD0.BAUDCTRLA = (BSEL & 0xff);
  USARTD0.BAUDCTRLB = (BSCALE << USART_BSCALE_gp) | (BSEL >> 8);
#endif

  // set up the ADC
  ADCA.CTRLA = ADC_ENABLE_bm; // enable.
  ADCA.CALL = adcacal0; // Load factory calibration into the ADC
  ADCA.CALH = adcacal1;
  ADCA.CTRLB = ADC_CONMODE_bm; // signed mode
  ADCA.PRESCALER = ADC_PRESCALER_DIV256_gc; // ~116 kHz
  ADCA.REFCTRL = ADC_REFSEL_INTVCC2_gc; // Vcc/2 reference
  ADCA.EVCTRL = 0; // no event control
  ADCA.CH0.CTRL = ADC_CH_INPUTMODE_DIFFWGAINL_gc; // differential, unity gain.
  ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN1_gc | ADC_CH_MUXNEGL_GND_gc; // input on pin A1, negative pad ground.
  ADCA.CH0.INTCTRL = 0; // no interrupts
  ADCA.CH0.AVGCTRL = (2 << ADC_CH_RIGHTSHIFT_gp) | ADC_SAMPNUM_4X_gc; // 4x oversampling

  // TCC4 and 5 are a 32 bit cascaded counter with cascaded capture (on PPS).
  TCC4.CTRLA = TC45_CLKSEL_DIV1_gc; // 30 MHz timer clocking
  TCC4.CTRLB = 0;
  TCC4.CTRLC = 0;
  TCC4.CTRLD = TC45_EVSEL_CH0_gc; // capture on event 0
  TCC4.CTRLE = TC45_CCBMODE_CAPT_gc | TC45_CCAMODE_CAPT_gc;
  TCC4.INTCTRLA = 0;
  TCC5.INTCTRLB = 0; // we're going to interrupt from TC5

  TCC5.CTRLA = TC45_CLKSEL_EVCH4_gc; // Clock from timer 4's overflow
  TCC5.CTRLB = 0;
  TCC5.CTRLC = 0;
  TCC5.CTRLD = TC5_EVDLY_bm | TC45_EVSEL_CH0_gc; // We're cascading 32 bits - we must delay capture events 1 cycle
  TCC5.CTRLE = TC45_CCBMODE_CAPT_gc | TC45_CCAMODE_CAPT_gc;
  TCC5.INTCTRLA = 0;
  TCC5.INTCTRLB = TC45_CCAINTLVL_MED_gc;

  pps_count = 0;
  mode = MODE_START;
  reset_pll();
  gps_locked = 0;
  rx_str_len = 0;
  last_osc_locked = 0xff; // none of the above
  last_gps_locked = 0xff; // none of the above
  *pps_err_buf = 0; // null terminate
#ifdef SW_BIT
  debounce_time = 0;
  button_down = 0;
  button_blink_time = 0;
#endif
#ifdef DEBUG
  // initialize to ""
  *date_buf = 0;
  *time_buf = 0;
  *pdop_buf = 0;
#endif
#ifdef SERIAL_TX
  txbuf_head = txbuf_tail = 0; // clear the transmit buffer
#endif

  PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
  sei();

#ifdef DEBUG
  tx_pstr(PSTR("\r\n\r\nSTART\r\n"));
  // only one of these should ever print out.
  if (reset_status & RST_SDRF_bm) tx_pstr(PSTR("RST_SPIKE\r\n"));
  if (reset_status & RST_SRF_bm) tx_pstr(PSTR("RST_SW\r\n"));
  if (reset_status & RST_PDIRF_bm) tx_pstr(PSTR("RST_PDI\r\n"));
  if (reset_status & RST_WDRF_bm) tx_pstr(PSTR("RST_WD\r\n"));
  if (reset_status & RST_BORF_bm) tx_pstr(PSTR("RST_BO\r\n"));
  if (reset_status & RST_EXTRF_bm) tx_pstr(PSTR("RST_EXT\r\n"));
  if (reset_status & RST_PORF_bm) tx_pstr(PSTR("RST_PO\r\n"));
#endif

  trim_value = 0;

#ifdef SPI_DAC
  // the SPI DAC powers-up to a default value of zero, but
  // that's not great for the oscillator. Jump straight to the
  // midpoint.
  writeDacValue(DAC_MIDPOINT, 0);
#endif

  while(1) {
    static unsigned long last_pps_count = 0;
 
    // Pet the dog
    wdt_reset();

#ifdef OSC_RDY
    unsigned char osc_locked = (PORTC.IN & OSC_RDY) == 0;
#else
    unsigned char osc_locked = 1; // if there's no OSC_RDY, then it's always ready
#endif

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

#ifdef OSC_RDY
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
        writeDacValue(0, 0);
        reset_pll();
      }
    }
#endif

    if (osc_locked && (CLK.CTRL & CLK_SCLKSEL_gm) == CLK_SCLKSEL_RC32M_gc) {
      // The oscillator is now locked, and we're still running from the 32 MHz
      // internal oscillator. Switch over to clocking from the external
      // oscillator.
#ifdef DEBUG
      tx_pstr(PSTR("\r\nCK_SW\r\n"));
#endif
      unsigned char tx_buf_empty = 0;
      do {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          tx_buf_empty = (txbuf_head == txbuf_tail);
        }
        wdt_reset();
      } while(!tx_buf_empty);
      do_delay_ms(20); // clear out the transmit buffer
      wdt_reset();

      // Switch to the PLL tripling the external oscillator input to 30 MHz.

// The FE405 runs at 15 MHz instead of 10 - so double it instead of triple.
#ifndef FE405
      OSC.XOSCCTRL = OSC_FRQRANGE_9TO12_gc | OSC_XOSCSEL_EXTCLK_gc;
#else
      OSC.XOSCCTRL = OSC_FRQRANGE_12TO16_gc | OSC_XOSCSEL_EXTCLK_gc;
#endif
      OSC.CTRL |= OSC_XOSCEN_bm; // enable it.
      while(!(OSC.STATUS & OSC_XOSCRDY_bm)) ; // wait for it.

#ifndef FE405
      OSC.PLLCTRL = OSC_PLLSRC_XOSC_gc | (3 << OSC_PLLFAC_gp); // 10 -> 30 MHz.
#else
      OSC.PLLCTRL = OSC_PLLSRC_XOSC_gc | (2 << OSC_PLLFAC_gp); // 15 -> 30 MHz.
#endif
      OSC.CTRL |= OSC_PLLEN_bm; // enable it.
      while(!(OSC.STATUS & OSC_PLLRDY_bm)) ; // wait for it.

      // _PROTECTED_WRITE by itself does not disable interrupts
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        _PROTECTED_WRITE(CLK.CTRL, CLK_SCLKSEL_PLL_gc); // switch to it
      }

      DFLLRC32M.CTRL &= ~DFLL_ENABLE_bm; //disable the FLL.
      OSC.CTRL &= ~(OSC_RC32KEN_bm | OSC_RC32MEN_bm) ; // disable the 32 MHz and 32 kHz oscillators

#ifdef DEBUG
      tx_pstr(PSTR("CK_OK\r\n\r\n"));
#endif
      continue;
    }

    unsigned char unlocked = (!gps_locked) || (!osc_locked);
    // next, take care of the LEDs.
    // If we're unlocked, then blink them back and forth at 2 Hz.
    // Otherwise, put the binary value of "mode" on the two LEDs.
    // BUT... if we're an ATMega328, then we have a button. If that button was pushed,
    // then give a distinctive blink pattern. This one is two blinks in a half second.
#ifdef SW_BIT
    if (button_blink_time != 0) {
      unsigned long blink_pos = timer_value() - button_blink_time;
      if (blink_pos > BUTTON_BLINK_TICKS) {
        button_blink_time = 0;
      } else {
        blink_pos = (unsigned int)((blink_pos * 4L) / BUTTON_BLINK_TICKS);
        if (blink_pos & 1) {
          PORTD.OUTSET = LED0 | LED1;
        } else {
          PORTD.OUTCLR = LED0 | LED1;
        }
      }
    } else
#endif
      if (!unlocked) {
      if (mode & 1)
        PORTD.OUTSET = LED0;
      else
        PORTD.OUTCLR = LED0;
      if (mode & 2)
        PORTD.OUTSET = LED1;
      else
        PORTD.OUTCLR = LED1;
    } else {
      unsigned long blink_pos = timer_value() % F_CPU;
      blink_pos = (4 * blink_pos) / F_CPU;
      PORTD.OUTSET = (blink_pos & 1)?LED1:LED0;
      PORTD.OUTCLR = (blink_pos & 1)?LED0:LED1;
    }

#ifdef SW_BIT
    if (check_buttons() && mode == MODE_SLOW && button_blink_time == 0) {
      // Do a non-volatile write and blink the LEDs to celebrate
#ifdef DEBUG
      tx_pstr(PSTR("\r\nEE_WR\r\n\r\n"));
#endif
      long dac_value = (long)(DAC_SIGN * trim_value);
      writeDacValue(dac_value, 1);
      button_blink_time = timer_value();
      if (!button_blink_time) button_blink_time++; // it cannot be set to 0.
    }
#endif

    // If we haven't had a PPS event since we were last here, we're done.
    if (last_pps_count == pps_count) continue;
    // if there hasn't been a quantization error sentence for this second, wait for it.
    if (pps_err_buf[0] == 0) continue;
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

    // Since our ADC is 11 bits and the pulse is a microsecond wide we can fudge a little
    // and claim that each ADC count is 500 picoseconds. So left shift once and then
    // current and average phase error is in nanoseconds and is wrapped.
    int current_phase_error = (PHASE_ADC_MIDPOINT - ((int)irq_adc_value)) / 2;

    current_phase_error += (int)((QE_COMPENSATION * pps_err) + 0.5); // quant error correction is in ns. Round to nearest

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
      tx_pstr(PSTR("ADC="));
      itoa(irq_adc_value, buf, 10);
      tx_str(buf);
      tx_pstr(PSTR("\r\nMOD="));
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
#ifdef SPI_DAC
      dac_value += DAC_MIDPOINT;
#endif

      writeDacValue(dac_value, 0);
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
#ifdef SPI_DAC
        tx_pstr(PSTR("\r\nDAC=0x"));
        ltoa((long)dac_value, buf, 16);
#else
        tx_pstr(PSTR("\r\nDAC="));
        ltoa((long)dac_value, buf, 10);
#endif
        tx_str(buf);
        tx_pstr(PSTR("\r\nET="));
        ltoa(exit_timer, buf, 10);
        tx_str(buf);
        tx_pstr(PSTR("\r\n\r\n"));
      }
#endif
      // If the average PPS error stays under 25 ppb for a minute, transition out to phase discipline.
      // This used to be 10 ppb, but the PLL tripling that adds some artifical jitter.
      if (fabs(average_pps_error) <= 0.25) {
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
#ifdef SPI_DAC
    dac_value += DAC_MIDPOINT;
#endif

    writeDacValue(dac_value, 0);

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
#ifdef SPI_DAC
      tx_pstr(PSTR("\r\nDAC=0x"));
      ltoa((long)dac_value, buf, 16);
#else
      tx_pstr(PSTR("\r\nDAC="));
      ltoa((long)dac_value, buf, 10);
#endif
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
