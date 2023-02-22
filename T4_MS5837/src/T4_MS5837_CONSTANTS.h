// Test for newer mode, this enables averaging as well
#define OPTIMIZE_MS5837_LOOP

#ifndef MS5837_ISR_PRIORITY
#define MS5837_ISR_PRIORITY     253             // Low default ISR prio
#endif

// Time in microseconds to wait between stages
#define UPDATE_TIME_PER_STAGE   21000           // 21ms

#define MS5837_ADDRESS          0x76
#define MS5837_RESET            0x1E
#define MS5837_ADC_READ         0x00
#define MS5837_PROM_READ        0xA0
#define MS5837_CONVERT_D1_8192  0x4A
#define MS5837_CONVERT_D2_8192  0x5A

#define MS5837_Pa_CONSTANT      100.0f
#define MS5837_bar_CONSTANT     0.001f
#define MS5837_mbar_CONSTANT    1.0f

#define MS5837_30BA_ID          0
#define MS5837_02BA_ID          1
#define MS5837_UNRECOGNIZED_ID  255

#define PROM_LEN                7
// Sensor version defined in PROM word 0
#define MS5837_02BA01_PROMID    0x00
#define MS5837_02BA21_PROMID    0x15
#define MS5837_30BA26_PROMID    0x1A

// ATM pressure (Pa) at measurement location
//  101300 Pa ~= Average ATM pressure
#define RESTING_ATMOSPHERIC_PRESSURE    101300.0f // Pa

// Default Grav accel
#define AVG_GRAV_ACCEL                  9.80665f // m/s^2

// Default water density at measurement location
//  1029 kg/m^3 == Salt Water
//  997 kg/m^3  == Fresh Water
#ifdef FRESH_WATER_OPERATION
#define FLUID_DENSITY   997.0f  // kg/m^3
#else
#define FLUID_DENSITY   1029.0f // kg/m^3
#endif