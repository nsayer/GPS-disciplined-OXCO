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

// Fuse settings: lfuse=0xe0, hfuse = 0xdb, efuse = 0x1
// ext osc, long startup time, 2.7v brownout, no self-programming

#include <stdlib.h>  
#include <stdio.h>  
#include <string.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

//#define DEBUG

#define LED_PORT PORTD
#define GPS_LED _BV(PORTD5)
#define LOCK_LED _BV(PORTD4)

// We don't actually need to read this pin. It triggers a TIMER1_CAPT interrupt.
//#define PPS_PIN _BV(PORTD6)

#define DAC_PORT PORTB
#define DAC_CS _BV(PORTB4)
#define DAC_DO _BV(PORTB5)
#define DAC_CLK _BV(PORTB7)

#define EE_TRIM_LOC ((uint16_t*)0)
// If the stored EEPROM trim value differs by this much from the present value,
// then update it
#define EE_UPDATE_OFFSET (20)

// 10 MHz
#define NOMINAL_CLOCK (10000000L)

// How many samples do we keep in our rolling window?
#define SAMPLE_COUNT (10)
// How many seconds is one sample? This shouldn't be longer
// than 5 minutes for two reasons: 1. The time counter is only 32 bits,
// and will roll over after ~400 seconds at 10 MHz. 2. At a maximum potential
// error of 10 ppm, that's +/- 30,000 counts, and the range of an int
// is +/-32,768.
#define SAMPLE_SECONDS (50)
// The measurement granularity is 10^9/(NOMINAL_CLOCK * SAMPLE_SECONDS * SAMPLE_COUNT)

#define SERIAL_BAUD (9600)
#define SERIAL_BAUD_CONST ((NOMINAL_CLOCK/(16L * SERIAL_BAUD)) - 1)
// Note that if you ever want to parse a longer sentence, be sure to bump this up.
// But an ATTiny4313 only has 1/4K of RAM, so...
#define RX_BUF_LEN (64)

volatile int sample_buffer[SAMPLE_COUNT];
volatile char valid_samples;
volatile unsigned char sample_window_pos;
volatile unsigned int trim_value;
volatile unsigned int timer_hibits;
volatile unsigned long pps_count;
volatile unsigned char gps_status;
volatile unsigned char lock;
volatile unsigned char rx_buf[RX_BUF_LEN];
volatile unsigned char rx_str_len;

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
  // Do this quickly!
  unsigned long timer_val = (((unsigned long)timer_hibits) << 16) | ICR1;
  if (gps_status == 0) {
    // at least keep track of the beginning of the second.
    last_timer_val = timer_val;
    return; // we don't care right now.
  }
  if (--sample_window_pos > 0) return; // sample incomplete
  sample_window_pos = SAMPLE_SECONDS; // start a new sample
  unsigned long time_span = timer_val - last_timer_val;
  last_timer_val = timer_val;
  
  // If we have too many, then we're running *fast*.
  int delta = (int)(time_span - (SAMPLE_SECONDS * NOMINAL_CLOCK));
  if (valid_samples < 0) {
    valid_samples++; // skip this one
  } else if (valid_samples < SAMPLE_COUNT) {
    sample_buffer[(unsigned char)valid_samples++] = delta;
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
  if (rx_char == 0x0d) {
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
// Note that we're only really going to use the transmit side
// either for diagnostics, or during setup to configure the
// GPS receiver. So we don't have to get too fancy, but these
// methods will block, so the messages should be kept short.
static inline void tx_char(const char c) {
  while ( !( UCSRA & _BV(UDRE))) ; // wait for empty tx reg
  UDR = c;
}

static inline void tx_str(const char *buf) {
  for(int i = 0; i < strlen(buf); i++)
    tx_char(buf[i]);
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
    //tx_char('0'); // diagnostic
    return; // there has to be room for the "*" and checksum.
  }
  i++; // skip the *
  unsigned char sent_checksum = (hexChar(rx_buf[i]) << 4) | hexChar(rx_buf[i + 1]);
  if (sent_checksum != checksum) {
    //tx_char('1'); // diagnostic
    return; // bad checksum.
  }
  
  if (strncmp_P((const char*)rx_buf, PSTR("$GPGSA"), 6)) {
    //tx_char('2'); // diagnostic
    return; // wrong sentence
  }
  unsigned char *ptr = (unsigned char *)rx_buf;
  for(i = 0; i < 2; i++) {
    ptr = (unsigned char *)strchr((const char *)ptr, ',');
    if (ptr == NULL) {
      //tx_char('3'); // diagnostic
      return; // not enough commas
    }
    ptr++; // skip over it
  }
  char gps_now_valid = *ptr == '3';
  if (gps_now_valid == gps_status) {
    //tx_char('4'); // diagnostic
    return; // no change in status
  }
  gps_status = gps_now_valid;
  if (gps_status)
    LED_PORT |= GPS_LED;
  else {
    LED_PORT &= ~GPS_LED;
    LED_PORT &= ~LOCK_LED;
    valid_samples = -1; // and clear the sample buffer
    sample_window_pos = SAMPLE_SECONDS;
    lock = 0;
  }
  //tx_char('*'); // diagnostic
}

void main() {
  // This must be done as early as possible to prevent the watchdog from biting during reset.
  MCUSR = 0;
  wdt_enable(WDTO_500MS);
 
  PRR |= _BV(PRTIM0) | _BV(PRUSI); 

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
// anything it wants - anything that's not a proper NEMA sentence
// will be ignored by the GPS module.
#ifdef DEBUG
  UCSRB = _BV(RXCIE) | _BV(RXEN) | _BV(TXEN); // transmit is for debugging
#else
  UCSRB = _BV(RXCIE) | _BV(RXEN);
#endif
  UCSRC = _BV(UCSZ0) | _BV(UCSZ1);

  // set up the LED port
  DDRD = _BV(DDD4) | _BV(DDD5);
  PORTD = 0; // Turn off both LEDs
  // set up the DAC port
  PORTB |= DAC_CS; // set CS high on the DAC *before* setting the direction.
  // DDRB0 is 0 to make PB0 an input for ICP
  DDRB = _BV(DDB4) | _BV(DDB5) | _BV(DDB7);
  
  // Restore the DAC to the last written value
  trim_value = eeprom_read_word(EE_TRIM_LOC);
  if (trim_value == 0xffff) // uninitialized flash
    trim_value = 0x8000; // default to midrange
  writeDacValue(trim_value);
  
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

  sei();

  while(1) {
    static unsigned long last_second;
  
    // Pet the dog
    wdt_reset();
  
    // next, blink the LOCK LED if we're locked so people know we're alive
    if (lock) {
      // 152 is 10 MHz / 65536 - timer_hibits frequency
      unsigned char blink_speed = 152 / lock; // better lock -> faster blink
      unsigned char blink_on = (timer_hibits % (blink_speed * 2)) > blink_speed; // 50% duty cycle
      if (blink_on)
        LED_PORT |= LOCK_LED;
      else
        LED_PORT &= ~LOCK_LED;
    } else {
        // lock == 0, turn off the LED.
        LED_PORT &= ~LOCK_LED;
    }
    // If the GPS sample buffer isn't full, we're done.
    if (valid_samples < SAMPLE_COUNT) continue;
    // If we haven't had a PPS event since we were last here, we're done.
    if (last_second == pps_count) continue;
    last_second = pps_count;
  
    // Collect the sum total of all of the deltas in the sample buffer.
    long sample_drift = 0;
    for(int i = 0; i < valid_samples; i++) {
      sample_drift += sample_buffer[i];
#ifdef DEBUG
      char buf[14];
      snprintf(buf, sizeof(buf), "SB=%d ", sample_buffer[i]);
      tx_str(buf);
#endif
    }
#ifdef DEBUG
    tx_char('\r');
    tx_char('\n');
#endif
  
    // Claim success if the total drift is under control
    // Each count is 0.2 ppb, but you have to add one
    // to round up.
    if (abs(sample_drift) < 250) { // 50 ppb
      if (abs(sample_drift) < 25) { // 5 ppb
        if (abs(sample_drift) < 5) // 1 ppb
          lock = 5; // best
        else
          lock = 2; // better 
      } else
        lock = 1; // good
    } else
      lock = 0; // bad

    if (abs(sample_drift) == 0) {
      // WOO HOO! Nothing to do!
    } else if (abs(sample_drift) < 20) {
      // When we're close in, just *nudge* the clock one unit at a time
      // but only using the most recent error delta.
      int latest_delta = sample_buffer[valid_samples - 1];
      if (latest_delta != 0) {
        trim_value += (latest_delta<0)?1:-1;
      }
    } else if (abs(sample_drift) < 1000) {
      // Try and guestimate from the sample drift how hard to hit the
      // oscillator. Each DAC count value is worth around 0.2 ppb, and
      // each error step is 0.2 ppb. But we want to under-adjust because
      // it'll take 10 samples to average out the impact, so let's call it
      // 0.05 DAC units per error.
      trim_value -= sample_drift / 20;
    } else {
      // WTF? Nothing makes sense anymore. Give it a hard shove.
      trim_value += (sample_drift < 0)?100:-100;
    }
    writeDacValue(trim_value);

#ifdef DEBUG
    {
      char buf[16];
      snprintf(buf, sizeof(buf), "TV=%04x\r\n", trim_value);
      tx_str(buf);
      snprintf(buf, sizeof(buf), "ER=%d\r\n", sample_drift);
      tx_str(buf);
    }
#endif

    // Only write to EEPROM when we're *exactly* dialed in, and
    // our trim value differs from the recorded one "significantly." 
    if (trim_value == 0 && sample_drift == 0 && abs(eeprom_read_word(EE_TRIM_LOC) - trim_value) > EE_UPDATE_OFFSET) {
      eeprom_write_word(EE_TRIM_LOC, trim_value);
    }
  }
}
