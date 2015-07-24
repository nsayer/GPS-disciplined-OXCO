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
  
#include <avr/eeprom.h>

#define GPS_LED 6
#define LOCK_LED 7
#define PPS_PIN 8
#define DAC_CS 9
#define DAC_DO 11
#define DAC_CLK 13

#define EE_TRIM_LOC ((uint16_t*)0)

// 20 MHz
#define NOMINAL_CLOCK (20000000L)

// How many samples do we keep in our rolling window?
#define SAMPLE_COUNT (10)

// Update the trim value in EEPROM every 4 hours
#define EE_UPDATE_TIME (3600*4)

// How far out are we allowed to be and still claim victory? 4=20 ppb
#define MAXIMUM_LOCK_ERROR (4)

int sample_buffer[SAMPLE_COUNT];
unsigned char valid_samples;

volatile unsigned int trim_value;
volatile unsigned long timer_hibits;
volatile unsigned long pps_count;
volatile unsigned char gps_status;
volatile unsigned char lock;

// Write the given 16 bit value to our AD5061 DAC.
// The data format is 6 bits of 0, then two bits of
// shutdown control, which we will set to 0 (because
// we never want to shut down), then 16 bits of
// value, MSB first. Data is clocked on the falling
// edge of the clock pin, and CS must be held low the
// whole time. The output voltage will slew on the rising
// edge of CS. The minimum time between clock transition
// is on the order of some number of nanoseconds, so
// delaying one microsecond is more than glacial.
static void writeDacValue(unsigned int value) {
  digitalWrite(DAC_CLK, HIGH);
  delayMicroseconds(1); // our DAC is quite fast
  digitalWrite(DAC_CS, LOW);
  digitalWrite(DAC_DO, LOW);
  // we're going to write a bunch of zeros. The first six are just
  // padding, and the last two are power-off bits that we want to be
  // always zero.
  for(int i = 0; i < 8; i++) {
    digitalWrite(DAC_CLK, LOW);
    delayMicroseconds(1);
    digitalWrite(DAC_CLK, HIGH);
    delayMicroseconds(1);
  }
  for(int i = 15; i >= 0; i--) {
    digitalWrite(DAC_DO, ((value >> i) & 0x1)?HIGH:LOW);
    digitalWrite(DAC_CLK, LOW);
    delayMicroseconds(1);
    digitalWrite(DAC_CLK, HIGH);
    delayMicroseconds(1);
  }
  digitalWrite(DAC_CS, HIGH); // And we're done.
}

// Timer 1's TCNT register is the low bits. They're ORed
// onto this to make an unsigned long. That gives us more
// than 200 seconds between full overflows (at 20 MHz).
ISR(TIMER1_OVF_vect) {
  timer_hibits += 0x10000L;
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
  unsigned long timer_val = timer_hibits | TCNT1;
  if (gps_status == 0) {
    // at least keep track of the beginning of the second.
    last_timer_val = timer_val;
    return; // we don't care right now.
  }
  unsigned long time_span = timer_val - last_timer_val;
  last_timer_val = timer_val;
  
  // If we have too many, then we're running *fast*.
  int delta = (int)(time_span - NOMINAL_CLOCK);
  if (valid_samples < SAMPLE_COUNT) {
    sample_buffer[valid_samples++] = delta;
  } else {
    for(int i = 0; i < SAMPLE_COUNT - 1; i++) {
      sample_buffer[i] = sample_buffer[i + 1];
    }
    sample_buffer[SAMPLE_COUNT - 1] = delta;
  }
  pps_count++;
}

static inline unsigned char hexChar(unsigned char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
}

// When this method is called, we've just received
// a $ on the serial port. We'll assume that this is
// the start of a NEMA GPS sentence. All we're really
// interested in is whether or not GPS has a 3D fix.
// For that, we'll partially parse the GPGSA sentence.
// Nothing else is of interest.
static inline void handleGPS() {
  char buf[96]; // no GPS sentence should be that long
  int count = Serial.readBytesUntil(0x0d, buf, sizeof(buf)); // carriage return
  
  // When we came in, we had already eaten the '$'
  if (count < 8) return; // No sentence is shorter than GPGGA*xx
  // First, check the checksum of the sentence
  unsigned char checksum = 0;
  int i;
  for(i = 0; i < count; i++) {
    if (buf[i] == '*') break;
    checksum ^= buf[i];
  }
  if (i > count - 3) return; // there has to be room for the "*" and checksum.
  i++; // skip the *
  unsigned char sent_checksum = hexChar(buf[i]) << 4 | hexChar(buf[i + 1]);
  if (sent_checksum != checksum) return; // bad checksum.
  
  if (strncmp(buf, "GPGSA", 5)) return; // wrong sentence
  char *ptr = buf;
  for(i = 0; i < 2; i++) {
    ptr = strchr(ptr, ',');
    if (ptr == NULL) return; // not enough commas
    ptr++; // skip over it
  }
  char gps_now_valid = *ptr == '3';
  if (gps_now_valid == gps_status) return; // no change in status
  // First, hit the led
  gps_status = gps_now_valid;
  digitalWrite(GPS_LED, gps_status?HIGH:LOW);
  if (!gps_status) {
    digitalWrite(LOCK_LED, LOW); //unlock as well.
    valid_samples = 0; // and clear the sample buffer
    lock = 0;
  }
}

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(10); // at 9600 bps, we should never have to wait >1 ms
  pinMode(PPS_PIN, INPUT);
  pinMode(GPS_LED, OUTPUT);
  digitalWrite(GPS_LED, LOW); // LED off
  pinMode(LOCK_LED, OUTPUT);
  digitalWrite(LOCK_LED, LOW); // LED off
  pinMode(DAC_CS, OUTPUT);
  digitalWrite(DAC_CS, HIGH); // disabled.
  pinMode(DAC_CLK, OUTPUT);
  pinMode(DAC_DO, OUTPUT);
  
  // Restore the DAC to the last written value
  trim_value = eeprom_read_word(EE_TRIM_LOC);
  if (trim_value == 0xffff) // uninitialized flash
    trim_value = 0x8000; // default to midrange
  writeDacValue(trim_value);
  
  // Set up timer1
  TCCR1A = 0; // Normal mode
  TCCR1B = _BV(ICES1) | _BV(CS10); // No noise reduction, rising edge capture, no pre-scale.
  TIMSK1 = _BV(ICIE1) | _BV(TOIE1); // Interrupt on overflow and capture
  ACSR = 0; // in particular, turn off ACIC so that ICP1/PB0 does the capturing.  
  TCNT1 = 0; // clear the counter.
  timer_hibits = 0;

  pps_count = 0;
  gps_status = 0;
  valid_samples = 0;

}

void loop() {
  static unsigned long last_second;
  static unsigned long seconds_since_last_flash_update = 0;
  
  // first, service the serial port.
  while(Serial.available() > 0) {
    if (Serial.read() == '$')
      handleGPS();
      return;
  }
  
  // next, blink the LOCK LED if we're locked so people know we're alive
  if (lock) {
    unsigned char blink_on = (timer_hibits % 80) > 40; // this should be about every quarter second
    digitalWrite(LOCK_LED, blink_on?HIGH:LOW);
  }
  // If the GPS sample buffer isn't full, we're done.
  if (valid_samples < SAMPLE_COUNT) return;
  // If we haven't had a PPS event since we were last here, we're done.
  if (last_second == pps_count) return;
  last_second = pps_count;
  
  // Collect the sum total of all of the deltas in the sample buffer.
  long sample_drift = 0;
  for(int i = 0; i < valid_samples; i++) {
    sample_drift += sample_buffer[i];
  }
  
  // Claim success if the total drift is under control
  lock = abs(sample_drift) <= MAXIMUM_LOCK_ERROR;
  //digitalWrite(LOCK_LED,lock?HIGH:LOW);
  
  if (abs(sample_drift) == 0) {
    // WOO HOO! Nothing to do!
  } else if (abs(sample_drift) < 10) {
    // When we're close in, just *nudge* the clock one unit at a time
    trim_value += (sample_drift<0)?-1:1;
  } else if (abs(sample_drift) < 100) {
    // Try and guestimate from the sample drift how hard to hit the
    // oscillator. Each DAC count value is worth around 1/5 ppb, and
    // each error step is 5 ppb. But we want to under-adjust because
    // it'll take 10 samples to average out the impact, so let's call it
    // 10 DAC units per error.
    trim_value += sample_drift * 10;
  } else {
    // WTF? Nothing makes sense anymore. Give it a hard shove.
    trim_value += (sample_drift < 0)?-1000:1000;
  }
  writeDacValue(trim_value);
  
  if (seconds_since_last_flash_update++ > EE_UPDATE_TIME) {
    seconds_since_last_flash_update = 0;
    eeprom_write_word(EE_TRIM_LOC, trim_value);
  }
}
