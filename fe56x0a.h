
// This configuration works either for FE-5680As that have 10 MHz output
// and the binary tuning protocol or FE-5650As that have been reconditioned
// by RDR Electronics with 5680A compatible firmware.
//
// The FE oscillator specifications are a tuning range of +/- 383 Hz
// over the range of a 32 bit signed integer, or +/- 38.3 ppm. That's
// an individual step size of 17.834E-15.
//
// However, specification and reality aren't always the same. The
// units I have for testing shows that at least near zero their resolution
// is far, far lower than that.
//
// The ADC measures a phase range of 1 us, which we arbitrarily
// devide at a midpoint for a range of +/- 512 ns (it's not exactly
// equal to nanoseconds, but it's close enough for us).
//
// The gain is how much we have to nudge the DAC to make 1 ns/sec
// phase change-rate. That is, to alter the frequency by 1 ppb (10^9).
//
// In this case, however, the gain is too high. A resolution of
// in the 10^-14 or 10^-15 is beyond the hardware we have to
// measure it. Throwing away some of the low resolution bits makes
// the math less problematic (a float can only have so many bits
// of precision).
//
// We want the actual GAIN value used to be no higher than a few hundred.
// We use BIT_REDUCE to throw low order bits away to make it so.
// Theoretical values:
//#define BIT_REDUCE (8)
//#define GAIN (56070 >> BIT_REDUCE)
// Measured values:
#define BIT_REDUCE (2)
#define GAIN (1466 >> BIT_REDUCE)

// What is our loop time constant? We use three different time constants - a
// faster one when we're outside of a certain range, and a slower
// one when we're dialed in.
#define TC_FAST 100
#define TC_MED 1800 // 0.5 hour
#define TC_SLOW 7200 // 2 hours

// the FEI oscillators have a serial DAC.
#undef SPI_DAC
