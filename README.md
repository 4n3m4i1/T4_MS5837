# T4_MS5837
This is a port of the Blue Robotics MS5837 Library to work with the enhanced Teensy 4.X i2C library written by Richard Gemmell.
`https://github.com/Richard-Gemmell/teensy4_i2c`  
This port is non-blocking, using the Interval Timer provided by Paul Stoffregen's IntervalTimer.h provided in the Teensyduino Core.  
`https://github.com/PaulStoffregen/cores/blob/master/teensy4/IntervalTimer.h`  
By doing this the 40ms of downtime per pressure+temp read is avoided. New Data flags are provided by library functions.  
In addition a signifigant amount of arithmetic has been replaced with logical operations with no observable effect on  
precision.  
An example program is provided outlining basic functionality. At this time altitude is not included as an output.
  
## Useful Functions
- Returns true on detection and initialization of sensor.
`bool ms5837_init()`
- Returns the model of the sensor. `0x00` for 30 bar, `0x01` for 2 bar, `0xFF` for unknown
`uint8_t ms5837_getModel()`
- Sets the density of the medium. Default is saltwater. `#define FRESH_WATER_OPERATION` to change default to fresh.
`void ms5837_setFluidType()`
- Begin the IntervalTimer and polling of the sensor
`void ms5837_loop_begin()`
- Returns true when unread or new data is ready to be read
`bool ms5837_Data_ready()`
- Returns the newest depth reading (meters)
`float ms5837_Read_Depth()`
- Returns the average of the last 0.25s of depth readings (meters)
`float ms5837_Avg_Depth()`
- Returns the newest temperature reading (celcius)
`float ms5837_Read_Temp()`
- Returns the average of the last 0.25s of temp readings (celcius)
`float ms5837_Avg_Temp()`
