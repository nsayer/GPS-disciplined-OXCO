// To determine the tuning step - that is, the frequency difference between
// adjacent DAC values (in theory), you multiply the control voltage slope
// of the oscillator (ppm per volt) by the DAC step voltage (volts per step).
// The value you get is ppm per step.
//
// Because of the multitude of different error sources, your actual tuning
// granularity target should be at least a half an order of magnitude higher
// than your desired frequency stability.
//
// The tuning step for the ECOC2522 variant is approximately 4.3 ppt.
// The DAC output range is 2.7 volts and the tuning
// range across 3.3 volts is 1.4ppm. The DAC is 18 bits,
// so (1.4ppm * (2.7 / 3.3v)) / 262144 is ~4.3 ppt. The target here is 0.1 ppb.
//
// The gain is how much we have to nudge the DAC to make 1 ns/sec
// phase change-rate. That is, to alter the frequency by 1 ppb.
#define GAIN 232

// What is our loop time constant? We use different time constants -
// faster ones when we're outside of a certain range, and slower
// ones when we're dialed in.
#define TC_FAST 100
#define TC_MED 200
#define TC_SLOW 400

// The ECS2522 variant uses an AD5680 SPI DAC.
#define SPI_DAC
#define AD5680

