#include "T4_MS5837.h"
#include <i2c_driver.h>         // Richard-Gemmell T4 i2C Library
#include <i2c_driver_wire.h>    // ^^

// This is a sample program showing
//  the use of all the functions that matter
//  for reading depth and temperature.
//  Altitude is not considered here.

// Default operation is configured for salt water, uncomment for fresh
//#define FRESH_WATER_OPERATION

const char B02_STR[] = "MS5837-02BA\0";
const char B30_STR[] = "MS5837-30BA\0";

void setup() {
  // FAST_I2C is representative of Wire, running at 400kHz
  FAST_I2C.begin();
  //FAST_I2C.setClock(400000);
  FAST_I2C.setClock(FAST_I2C_FREQ);
  Serial.begin(115200);

  delay(100);

  // Run initiation until successful
  uint16_t e_ct = 0;
  while(!ms5837_init()){
    Serial.printf("%d\tError on Init!!\n\n", e_ct++);
    delay(1000);
  }

  // Query and print the model detected
  Serial.printf("Model ID:\t%2X found on bus!\n", ms5837_getModel());
  if(ms5837_getModel() == MS5837_02BA_ID){
    Serial.printf("Model Decoded: %s\n", B02_STR);
  } else {
    Serial.printf("Model Decoded: %s\n", B30_STR);
  }
  
  Serial.printf("Setting up 20ms interval ISR.\n\n");  
  ms5837_loop_begin();
}

void loop() {
  // If data isn't ready this is just freewheeling

  // Check if data is ready and unique
  //    (Conversion and Calculation is done, data has not been read yet)
  if(ms5837_Data_ready()){
    // Reading the depth or average depth will clear the New Data Ready flag, however reading Temp will not
    Serial.printf("Depth:\t\t%.4f meters\n", ms5837_Read_Depth());    // Read Depth Value
    Serial.printf("Avg Depth:\t%.4f meters\n", ms5837_Avg_Depth());   // Read Average of last 4 Depth Values
    Serial.printf("Temp:\t\t%.2f deg C\n", ms5837_Read_Temp());       // Read Temp value
    Serial.printf("Avg Temp:\t%.2f deg C\n\n", ms5837_Avg_Temp());    // Read Temp value
  }
}
