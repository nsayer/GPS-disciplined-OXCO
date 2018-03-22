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
// The gain is how much we have to nudge the DAC to make 1 ns/sec
// phase change-rate. That is, to alter the frequency by 1 ppb.
#define GAIN 267

// What is our loop time constant? We use different time constants -
// faster ones when we're outside of a certain range, and slower
// ones when we're dialed in.
#define TC_FAST 100
#define TC_MED 200
#define TC_SLOW 400

// The OH300 variant uses an AD5680 SPI DAC.
#define SPI_DAC
#define AD5680

