#ifndef T4_MS5837_H
#define T4_MS5837_H

#include <inttypes.h>
#include <i2c_device.h>
#include <i2c_driver_wire.h>            // Richard-Gemmell T4 i2C
#include "T4_MS5837_CONSTANTS.h"

//  In the overall sensor arch Wire will be 400kHz
//      and Wire1 will be 100kHz
#ifndef FAST_I2C
#define FAST_I2C        Wire
#define FAST_I2C_FREQ   400000
#endif


struct MS5837_SENS_DATA{
    uint8_t new_data_ready = 0;
    volatile float   depth = 0.0f;
    volatile float   temperature = 0.0f;
};

struct MS5837_SENS_INFO{
    uint8_t     model;
    uint8_t     _read_stage = 0;
    volatile int32_t     P;
    volatile int32_t     TEMP;
    volatile uint32_t    D1_pres;
    volatile uint32_t    D2_temp;
    uint16_t    C[8];
    float       sens_fluid_density = FLUID_DENSITY;
};

struct MS5837_SENSOR{
    MS5837_SENS_DATA data;
    MS5837_SENS_INFO info;
};

// Once Sensor has been initialized successfully
//  this will begin the ISR triggered loop to update
//  sensor values.
void ms5837_loop_begin();

// Terminates the ISR polling loop, can be restarted
//  with loop_begin
void ms5837_loop_terminate();

// Wrapper for writing to the sensor over FAST_I2C
void inline sendToMS5837(uint8_t val_);

// Wrapper for Requesting Bytes from Sensor over FAST_I2C
void inline requestFromMS5837(uint8_t bytes_rqd_);

// Wrapper for Reading a Byte from FAST_I2C
uint8_t read_uint8_WIRE();

// Wrapper for reading 2 bytes that are assembled into
//  a uint16 from FAST_I2C
uint16_t read_uint16_WIRE(uint8_t addr__);

// Initiate Sensor. This must be called, and must
//  return 1 (true) before any other actions may take
//  place.
bool ms5837_init();

// This will return the model of the sensor. This
//  is detected after successful initialization (init)
uint8_t ms5837_getModel();

// This can be called after initialization to
//  change from saltwater to fresh, or vise versa.
// Default is Saltwater.
void ms5837_setFluidType(float density_kgper_mcub);


// User callable functions to return values from the
//  pSensor struct.

// Returns true if Temp or Pressure has updated
//  since last user read
bool ms5837_Data_ready();

// Returns most recent depth reading in meters
//  Clears Data Ready Flag
float ms5837_Read_Depth();
// Returns the average of the last 4 samples taken evenly over 0.25s, in meters
//  Clears Data Ready Flag
float ms5837_Avg_Depth();

// Returns most recent temperature reading, in celcius
float ms5837_Read_Temp();
// Returns the average of the last 4 samples taken evenly over 0.25s, in celcius
float ms5837_Avg_Temp();

// Clears data ready flag. This is automatically done
//  upon a Read_Depth()
void ms5837_Data_Clear();



// Update and recalculate values
#ifdef OPTIMIZE_MS5837_LOOP
void __ms5837_first_conversion();
#endif
// Send appropriate D1 or D2 conversion start
//  store read values to appropriate location
//  begins the conversion upon D1 and D2 read.
void ms5837_update();

// Does the conversion math, this is slow
void ms5837_calculate_();

// Minor intermediary conversions
float ms5837_get_pressure(float conversion_);
float ms5837_get_depth();

// Verify PROM contents
uint8_t ms5837_crc4(uint16_t n_prom[]);

#endif