// To determine the tuning step - that is, the frequency difference between
// adjacent DAC values (in theory), you multiply the control voltage slope
// of the oscillator (ppm per volt) by the DAC step voltage (volts per step).
// The value you get is ppm per step.
//
// Because of the multitude of different error sources, your actual tuning
// granularity target should be at least a half an order of magnitude higher
// than your desired frequency stability.
//
// The tuning step for the DOT050V variant is approximately 200 ppt.
// The DAC output range is 1.65 volts (50% of 3.3v) and the tuning range
// across that is 12.2 ppm (60% of 20ppm). The DAC is 16 bits,
// so that's ~180 ppt. The target here is 1 ppb.
//
// The gain is how much we have to nudge the DAC to make 1 ns/sec
// phase change-rate. That is, to alter the frequency by 1 ppb.
#define GAIN 5

// What is our loop time constant? We use different time constants -
// faster ones when we're outside of a certain range, and slower
// ones when we're dialed in.
#define TC_FAST 50
#define TC_SLOW 100

// The DOT050V variant uses an AD5061 SPI DAC.
#define SPI_DAC
#undef AD5680

