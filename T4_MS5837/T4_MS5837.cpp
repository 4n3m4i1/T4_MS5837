#include "T4_MS5837.h"
#include <i2c_device.h>
#include <i2c_driver_wire.h> 
#include <IntervalTimer.h>

// There is a fair deal of optimization and cleaning up
// however this is still functional for the time being

MS5837_SENSOR pSensor;

// More data used for averaging over 4
MS5837_SENS_DATA pSensor_1;
MS5837_SENS_DATA pSensor_2;
MS5837_SENS_DATA pSensor_3;

IntervalTimer pTimer;

void ms5837_loop_begin(){
    // If the loop is optimized down we must call the first init
    //  before the interval spaced loop begins
#ifdef OPTIMIZE_MS5873_LOOP
    __ms5873_first_conversion();
#endif
    // This starts an interval timer.
    //  Once D1 conversion (1) -> (D1) takes 20ms
    //  Requesting D2 conversion (2) -> (D2) takes 20ms
    //  Thus our interval should be 20ms, and at the end of
    //  one request cycle the next should be started.
    pTimer.begin(ms5873_update, UPDATE_TIME_PER_STAGE);
    pTimer.priority(MS5873_ISR_PRIORITY);
}

void ms5837_loop_terminate(){
    pTimer.end();
}


void inline sendToMS5837(uint8_t val_){
    FAST_I2C.beginTransmission(MS5837_ADDRESS);
    FAST_I2C.write(val_);
    FAST_I2C.endTransmission(1);
}

void inline requestFromMS5837(uint8_t bytes_rqd_){
    FAST_I2C.requestFrom(MS5837_ADDRESS, bytes_rqd_);
}

uint8_t read_uint8_WIRE(){
    return FAST_I2C.read();
}

uint16_t read_uint16_WIRE(uint8_t addr__){
    FAST_I2C.requestFrom(addr__, 2);
    uint16_t qq = FAST_I2C.read();
    return ((qq << 8) | FAST_I2C.read());
}


bool ms5837_init(){
    sendToMS5837(MS5837_RESET);
    delay(20);                      // One time blocking delay

    for(uint8_t n = 0; n < PROM_LEN; n++){
        sendToMS5837(MS5837_PROM_READ + (2 * n));
        pSensor.info.C[n] = read_uint16_WIRE(MS5837_ADDRESS);
    }

    // Check PROM read with a CRC4
    //  returns if CRC fails
    if((uint8_t)(pSensor.info.C[0] >> 12) != ms5837_crc4(pSensor.info.C))    return false;

    // Determine and store model number
    //  This determines how we interpret input data
    //  either 30 bar low res, or 2 bar hi res
    switch((pSensor.info.C[0] >> 5) & 0x7F){
        case MS5837_02BA01_PROMID:
            pSensor.info.model = MS5837_02BA_ID;
        break;
        case MS5837_02BA21_PROMID:
            pSensor.info.model = MS5837_02BA_ID;
        break;
        case MS5837_30BA26_PROMID:
            pSensor.info.model = MS5837_30BA_ID;
        break;
        default:
            pSensor.info.model = MS5837_UNRECOGNIZED_ID;
        break;
    }

    return true;
}

// Check which model you have, 
//  not super useful unless you multiplex a few of these
uint8_t ms5837_getModel(){
    return pSensor.info.model;
}

// To dynamically set the fluid density,
//  if you were enterprising you could find this from
//  temp + salinity
void ms5837_setFluidType(float density_kgper_mcub){
    pSensor.info.sens_fluid_density = density_kgper_mcub;
}

#ifdef OPTIMIZE_MS5873_LOOP                     // Cuts out a fair bit of overhead logic
void __ms5873_first_conversion(){
    sendToMS5837(MS5837_CONVERT_D1_8192);
}
void ms5873_update(){
    static volatile bool state__ = 0;
    static volatile uint8_t ct__ = 0;
    if(!state__){
        sendToMS5837(MS5837_ADC_READ);
        requestFromMS5837(3);
        pSensor.info.D1_pres = read_uint8_WIRE();
        pSensor.info.D1_pres |= ((pSensor.info.D1_pres << 8) | read_uint8_WIRE());
        pSensor.info.D1_pres |= ((pSensor.info.D1_pres << 8) | read_uint8_WIRE());
        
        // Request D2 Conversion
        sendToMS5837(MS5837_CONVERT_D2_8192);       // 20 ms duration
    } else {
        sendToMS5837(MS5837_ADC_READ);
        requestFromMS5837(3);
        pSensor.info.D2_temp = read_uint8_WIRE();
        pSensor.info.D2_temp |= ((pSensor.info.D2_temp << 8) | read_uint8_WIRE());
        pSensor.info.D2_temp |= ((pSensor.info.D2_temp << 8) | read_uint8_WIRE());

        ms5873_calculate_();

        // Store values for averaging
        if(ct__ == 0){
            pSensor_1.depth = pSensor.data.depth;
            pSensor_1.temperature = pSensor.data.temperature;
            ct__ += 1;
        } else if(ct__ == 6){
            pSensor_2.depth = pSensor.data.depth;
            pSensor_2.temperature = pSensor.data.temperature;
            ct__ += 1;
        } else if(ct__ == 12){
            pSensor_3.depth = pSensor.data.depth;
            pSensor_3.temperature = pSensor.data.temperature;
            ct__ = 0xFF;
        }

        ct__ += 1;

        pSensor.data.depth = ms5873_get_depth();
        pSensor.data.temperature = pSensor.info.TEMP / 100.0f;
        pSensor.data.new_data_ready = 1;
        sendToMS5837(MS5837_CONVERT_D1_8192);       // Re issue start
    }
    state__ = !state__;
}
#else
void ms5873_update(){
    // Read from Sensor
    if(&FAST_I2C == NULL) return;
    
    switch(pSensor.info._read_stage){
        case 0:     // Init ADC conversion, 20ms duration
            sendToMS5837(MS5837_CONVERT_D1_8192);
            pSensor.info._read_stage = 1;
        break;
        case 1:     // Read D1 from ADC
            sendToMS5837(MS5837_ADC_READ);
            requestFromMS5837(3);
            pSensor.info.D1_pres = read_uint8_WIRE();
            pSensor.info.D1_pres |= ((pSensor.info.D1_pres << 8) | read_uint8_WIRE());
            pSensor.info.D1_pres |= ((pSensor.info.D1_pres << 8) | read_uint8_WIRE());
        
            // Request D2 Conversion
            sendToMS5837(MS5837_CONVERT_D2_8192);       // 20 ms duration
            pSensor.info._read_stage = 2;
        break;
        case 2:     // Read D2 from ADC
            sendToMS5837(MS5837_ADC_READ);
            requestFromMS5837(3);
            pSensor.info.D2_temp = read_uint8_WIRE();
            pSensor.info.D2_temp |= ((pSensor.info.D2_temp << 8) | read_uint8_WIRE());
            pSensor.info.D2_temp |= ((pSensor.info.D2_temp << 8) | read_uint8_WIRE());

            ms5873_calculate_();
            pSensor.data.depth = ms5873_get_depth();
            pSensor.data.temperature = pSensor.info.TEMP / 100.0f;
            pSensor.data.new_data_ready = 1;
            sendToMS5837(MS5837_CONVERT_D1_8192);       // Re issue start
            pSensor.info._read_stage = 1;               //
        break;
        default:
            pSensor.info._read_stage = 0;
        break;
    }
}
#endif


// Optimized down the standard maths
//  removed most MUL and DIV, replaced with shifts
void ms5873_calculate_(){
	// Given C1-C6 and D1, D2, calculated TEMP and P
	// Do conversion first and then second order temp compensation

	int32_t dT = 0;
	int64_t SENS = 0;
	int64_t OFF = 0;
	int32_t SENSi = 0;
	int32_t OFFi = 0;
	int32_t Ti = 0;
	int64_t OFF2 = 0;
	int64_t SENS2 = 0;

	// Terms called
	//dT = pSensor.info.D2_temp-uint32_t(pSensor.info.C[5])*256l;
	dT = pSensor.info.D2_temp-((uint32_t)(pSensor.info.C[5]) << 8);

    if ( pSensor.info.model == MS5837_02BA_ID ) {
		//SENS = int64_t(pSensor.info.C[1])*65536l + (int64_t(pSensor.info.C[3])*dT)/128l;
		//SENS = (int64_t)(pSensor.info.C[1])*65536l + (((int64_t)(pSensor.info.C[3])*dT) >> 7);
        SENS = ((int64_t)(pSensor.info.C[1]) << 16) + (((int64_t)(pSensor.info.C[3])*dT) >> 7);
        //OFF = int64_t(pSensor.info.C[2])*131072l + (int64_t(pSensor.info.C[4])*dT)/64l;
		//OFF = (int64_t)(pSensor.info.C[2])*131072l + (((int64_t)(pSensor.info.C[4])*dT) >> 6);
        OFF = ((int64_t)(pSensor.info.C[2]) << 17) + (((int64_t)(pSensor.info.C[4])*dT) >> 6);
        //pSensor.info.P = (pSensor.info.D1_pres*SENS / (2097152l) - OFF) / (32768l);
        pSensor.info.P = ((pSensor.info.D1_pres * SENS >> 21) - OFF) >> 15;
	} else {
		//SENS = int64_t(pSensor.info.C[1])*32768l + (int64_t(pSensor.info.C[3])*dT)/256l;
		SENS = ((int64_t)(pSensor.info.C[1]) << 15) + (((int64_t)(pSensor.info.C[3])*dT) >> 8);
        //OFF = int64_t(pSensor.info.C[2])*65536l + (int64_t(pSensor.info.C[4])*dT)/128l;
		OFF = ((int64_t)(pSensor.info.C[2]) << 16) + (((int64_t)(pSensor.info.C[4])*dT) >> 7);
        //pSensor.info.P = (pSensor.info.D1_pres * SENS/(2097152l) - OFF) / (8192l);
        pSensor.info.P = ((pSensor.info.D1_pres * SENS >> 21) - OFF) >> 13;
	}

	// Temp conversion, shifts break this :^(
	pSensor.info.TEMP = 2000l + (int64_t)(dT) * pSensor.info.C[6]/8388608LL;
    //pSensor.info.TEMP = 2000l + ((int64_t)(dT) * (((int64_t)pSensor.info.C[6]) >> 23));

	//Second order compensation
	if ( pSensor.info.model == MS5837_02BA_ID ) {
		if((pSensor.info.TEMP / 100)<20){         //Low temp
			//Ti = (11*int64_t(dT)*int64_t(dT))/(34359738368LL);
            Ti = (11*int64_t(dT)*int64_t(dT)) >> 35;
			//OFFi = (31*(pSensor.info.TEMP-2000)*(pSensor.info.TEMP-2000))/8;
            OFFi = (31*(pSensor.info.TEMP-2000)*((pSensor.info.TEMP-2000)) >> 3);
			//SENSi = (63*(pSensor.info.TEMP-2000)*(pSensor.info.TEMP-2000))/32;
            SENSi = (63*(pSensor.info.TEMP-2000)*((pSensor.info.TEMP-2000)) >> 5);
		}
	} else {
		if((pSensor.info.TEMP / 100) < 20){         //Low temp
			//Ti = (3*int64_t(dT)*int64_t(dT))/(8589934592LL);
            Ti = (3*int64_t(dT)*int64_t(dT)) >> 33;
			//OFFi = (3*(pSensor.info.TEMP-2000)*(pSensor.info.TEMP-2000))/2;
            OFFi = (3*(pSensor.info.TEMP-2000)*((pSensor.info.TEMP-2000)) >> 1);
			//SENSi = (5*(pSensor.info.TEMP-2000)*(pSensor.info.TEMP-2000))/8;
            SENSi = (5*(pSensor.info.TEMP-2000)*((pSensor.info.TEMP-2000)) >> 3);
			
            if((pSensor.info.TEMP / 100) < -15){    //Very low temp
				OFFi = OFFi+7*(pSensor.info.TEMP+1500l)*(pSensor.info.TEMP+1500l);
				//SENSi = SENSi+4*(pSensor.info.TEMP+1500l)*(pSensor.info.TEMP+1500l);
                SENSi = SENSi + (((pSensor.info.TEMP+1500l) * (pSensor.info.TEMP+1500l)) << 2);
			}
		}
		else if((pSensor.info.TEMP / 100) >= 20){    //High temp
			//Ti = 2*(dT*dT)/(137438953472LL);
            Ti = (int32_t)((int64_t)(dT * dT) >> 36);
			//OFFi = (1*(pSensor.info.TEMP-2000)*((pSensor.info.TEMP-2000))/16);
			OFFi = ((pSensor.info.TEMP-2000)*((pSensor.info.TEMP-2000)) >> 4);
            SENSi = 0;
		}
	}

	OFF2 = OFF - OFFi;           //Calculate pressure and temp second order
	SENS2 = SENS - SENSi;

	pSensor.info.TEMP = (pSensor.info.TEMP - Ti);

	if (pSensor.info.model == MS5837_02BA_ID) {
		//pSensor.info.P = (((pSensor.info.D1_pres*SENS2)/2097152l-OFF2)/32768l);
        pSensor.info.P = (((pSensor.info.D1_pres*SENS2)/2097152l-OFF2) >> 15);
	} else {
		//pSensor.info.P = (((pSensor.info.D1_pres*SENS2)/2097152l-OFF2)/8192l);
        pSensor.info.P = (((pSensor.info.D1_pres*SENS2)/2097152l-OFF2) >> 13);
	}
}

// Pa
float ms5873_get_pressure(float conversion_){
  if(pSensor.info.model == MS5837_02BA_ID){
    return (pSensor.info.P * conversion_) / 100.0f;
  } else {
    return (pSensor.info.P * conversion_) / 10.0f;
  }
}

// Meters
float ms5873_get_depth(){
    float _tmp__ = ms5873_get_pressure(MS5837_Pa_CONSTANT) - RESTING_ATMOSPHERIC_PRESSURE;
    _tmp__ /= (FLUID_DENSITY * AVG_GRAV_ACCEL);
    return _tmp__;
}

bool ms5873_Data_ready(){
    return pSensor.data.new_data_ready;
}

float ms5873_Read_Depth(){
    pSensor.data.new_data_ready = 0;            // Indicate we've read new data
    return pSensor.data.depth;
}

float ms5873_Read_Temp(){
    return pSensor.data.temperature;
}

float ms5873_Avg_Depth(){
    pSensor.data.new_data_ready = 0;
    return (pSensor.data.depth + pSensor_1.depth + pSensor_2.depth + pSensor_3.depth) / 4.0f;
}

float ms5873_Avg_Temp(){
    return (pSensor.data.temperature + pSensor_1.temperature + pSensor_2.temperature + pSensor_3.temperature) / 4.0f;
}

// This should never be needed, however it can be useful
void ms5873_Data_Clear(){
    pSensor.data.new_data_ready = 0;
}

// Almost entirely taken from the datasheet,
//  except people need to stop doing (foo % 2) to check
//  if something is even or odd..
uint8_t ms5837_crc4(uint16_t n_prom[]){
    uint16_t n_rem = 0;

	n_prom[0] = ((n_prom[0]) & 0x0FFF);
	n_prom[7] = 0;

	for (uint32_t n = 0; n < 16; n++){
        if (n & 1) {
			n_rem ^= (uint16_t)((n_prom[n>>1]) & 0x00FF);
		} else {
			n_rem ^= (uint16_t)(n_prom[n>>1] >> 8);
		}
		for(uint8_t n_bit = 8; n_bit > 0; n_bit-- ){
			if ( n_rem & 0x8000 ) {
				n_rem = (n_rem << 1) ^ 0x3000;
			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	n_rem = ((n_rem >> 12) & 0x000F);

	return n_rem ^ 0x00;
}
