/*
LidarLite - C++ wrapper for LIDAR Lite interfacing with RaspberryPi
Created by Produce Consume Robot 2015.
http://produceconsumerobot.com/

This work is licensed under the Creative Commons 
Attribution-ShareAlike 3.0 Unported License. 
To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/3.0/.

Leverages WiringPi by drogon
Derived from https://github.com/PulsedLight3D/LIDARLite_v2_Arduino_Library/tree/master/LIDARLite
	
Requirements:
	Install wiring pi - http://wiringpi.com/
	Add PROJECT_LDFLAGS += -lwiringPi to the PROJECT LINKER FLAGS section of config.make
	
See LIDAR Lite documentation for more info
http://kb.pulsedlight3d.com/
https://github.com/PulsedLight3D/
*/

#include "LidarLite.hpp"
#include <wiringPiI2C.h>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <unistd.h>

//--------------------------------------------------------------
LidarLite::LidarLite() {
	fd = -1;
	errorReporting = false;
	logLevel = NONE;
	REG_STATUS = REG_STATUS_V21;
	hwVersion = 0;
	swVersion = 0;
}

/* =============================================================================
  Begin
  Starts the sensor and I2C
  Process
  ------------------------------------------------------------------------------
  1.  Turn on error reporting, off by default
  2.  Start Wire (i.e. turn on I2C)
  3.  Enable 400kHz I2C, 100kHz by default
  4.  Set configuration for sensor
  Parameters
  ------------------------------------------------------------------------------
  - configuration: set the configuration for the sensor
    - default or 0 = equivelent to writing 0x00 to 0x00, i.e. full reset of
      sensor, if you write nothing for configuration or 0, the sensor will init-
      iate normally
    - 1 = high speed setting, set the aquisition count to 1/3 the default (works
      great for stronger singles) can be a little noisier
  - fasti2c: if true i2c frequency is 400kHz, default is 100kHz
  - showErrorReporting: if true reads with errors will print the value of 0x40,
    used primarily for debugging purposes by PulsedLight
  - LidarLiteI2cAddress (optional): Default: 0x62, the default LIDAR-Lite
    address. If you change the address, fill it in here.
============================================================================= */
void LidarLite::begin(int configuration, bool fasti2c, bool showErrorReporting, char LidarLiteI2cAddress){
	if (logLevel <= VERBOSE) cout << "LidarLite::begin" << endl;
	
	errorReporting = showErrorReporting;
	
	if (fasti2c){
		// fast I2C not yet supported, come again soon
		cout << "fast I2C not yet supported, come again soon" << endl;
	}
	
	// initialize the LidarLite
	fd = wiringPiI2CSetup(LidarLiteI2cAddress);
	
	hwVersion = readByte(fd, REG_HARDWARE_VERSION, false);
	swVersion = readByte(fd, REG_SOFTWARE_VERSION, false);
	
	if (hardwareVersion() < 21) {
		REG_STATUS = REG_STATUS_V20;
		status();
		//ofSleepMillis(100);
		usleep(100000);
	}

	if (fd > -1) {
			configure(configuration);
			//ofSleepMillis(100);
			usleep(100000);
		//}
	}
	
	return;
}

/* =============================================================================
  hasBegun
	Returns whether begin successfully initialized the LIDAR Lite
	============================================================================= */
bool LidarLite::hasBegun() {
	if (logLevel <= VERBOSE) cout << "LidarLite::hasBegun" << endl;
	if (fd > -1) {
		return true;
	}
	return false;
}

/* =============================================================================
  Configure
  Sets the configuration for the sensor, typically this is done in the begin()
  command, but sometimes (especially for multi-sensor applications) you will
  need to do this separately.
  Parameters
  ------------------------------------------------------------------------------
  - configuration: set the configuration for the sensor
    - default or 0 = equivelent to writing 0x00 to 0x00, i.e. full reset of
      sensor, if you write nothing for configuration or 0, the sensor will init-
      iate normally
    - 1 = high speed setting, set the aquisition count to 1/3 the default (works
      great for stronger singles) can be a little noisier
============================================================================= */
void LidarLite::configure(int configuration){
	if (logLevel <= VERBOSE) cout << "LidarLite::configure" << endl;
	int writeSuccess = 0;
  switch (configuration){
    case 0: //  Default configuration
			writeSuccess = wiringPiI2CWriteReg8(fd, 0x00, 0x00);
			//ofSleepMillis(1);
			usleep(1000);
    break;
    case 1: //  Set aquisition count to 1/3 default value, faster reads, slightly
            //  noisier values
			writeSuccess = wiringPiI2CWriteReg8(fd, 0x04,0x00);
			//ofSleepMillis(1);
			usleep(1000);
    break;
    case 2: //  Low noise, low sensitivity: Pulls decision criteria higher
            //  above the noise, allows fewer false detections, reduces
            //  sensitivity
      writeSuccess = wiringPiI2CWriteReg8(fd, 0x1c,0x20);
			//ofSleepMillis(1);
			usleep(1000);
    break;
    case 3: //  High noise, high sensitivity: Pulls decision criteria into the
            //  noise, allows more false detections, increses sensitivity
      writeSuccess = wiringPiI2CWriteReg8(fd, 0x1c,0x60);
			//ofSleepMillis(1);
			usleep(1000);
    break;
  }
	if (logLevel <= INFO) cout << "writeSuccess = " << writeSuccess << endl;
}

/* =============================================================================
  Distance
  Read the distance from LIDAR-Lite
  Process
  ------------------------------------------------------------------------------
  1.  Write 0x04 to register 0x00 to initiate an aquisition.
  2.  Read register 0x01 (this is handled in the read() command)
      - if the first bit is "1" then the sensor is busy, loop until the first
        bit is "0"
      - if the first bit is "0" then the sensor is ready
  3.  Read two bytes from register 0x8f and save
  4.  Shift the FirstValueFrom0x8f << 8 and add to SecondValueFrom0x8f This new
      value is the distance.
  Parameters
  ------------------------------------------------------------------------------
  - stablizePreampFlag (optional): Default: true, take aquisition with DC
    stabilization/correction. If set to false, it will read
  - faster, but you will need to sabilize DC every once in awhile (ex. 1 out of
    every 100 readings is typically good).
  - LidarLiteI2cAddress (optional): Default: 0x62, the default LIDAR-Lite
    address. If you change the address, fill it in here.
  Example Arduino Usage
  ------------------------------------------------------------------------------
  1.  // take a reading with DC stabilization and the 0x62 default i2c address
      // the distance variable will hold the distance
      int distance = 0
      distance = myLidarLiteInstance.distance();
  2.  // take a reading without DC stabilization and the 0x62 default i2c address
      int distance = 0
      distance = myLidarLiteInstance.distance(false);
  3.  // take a reading with DC stabilization and a custom i2c address of 0x66
      int distance = 0
      distance = myLidarLiteInstance.distance(true,0x66);
  Notes
  ------------------------------------------------------------------------------
    Autoincrement: A note about 0x8f vs 0x0f
    Set the highest bit of any register to "1" if you set the high byte of a
    register and then take succesive readings from that register, then LIDAR-
    Lite automatically increments the register one for each read. An example: If
    we want to read the high and low bytes for the distance, we could take two
    single readings from 0x0f and 0x10, or we could take 2 byte read from reg-
    ister 0x8f. 0x8f = 10001111 and 0x0f = 00001111, meaning that 0x8f is 0x0f
    with the high byte set to "1", ergo it autoincrements.
============================================================================= */
int LidarLite::distance(bool stablizePreampFlag, bool takeReference){
	if (logLevel <= VERBOSE) cout << "LidarLite::distance" << endl;
	int loVal, hiVal, writeSuccess = 0;
	
  if(stablizePreampFlag){
    // Take acquisition & correlation processing with DC correction
		writeSuccess = wiringPiI2CWriteReg8(fd, REG_MEASURE, VAL_MEASURE);
		//ofSleepMillis(1);
		usleep(1000);
  }else{
    // Take acquisition & correlation processing without DC correction
		writeSuccess = wiringPiI2CWriteReg8(fd, REG_MEASURE, VAL_MEASURE_NO_DC_CRCT);
		//ofSleepMillis(1);
		usleep(1000);
  }
	
	if (logLevel <= DEBUG) cout << "writeSuccess = " << writeSuccess << endl;
	
	// Get the low byte, return -1 if error occurred
	loVal = readByte(fd, REG_LO_DISTANCE, true);
	if (loVal == -1) return -1;
	if (logLevel <= VERBOSE) cout << "loVal = " << loVal << endl;
	
	// Get the high byte, return -1 if error occurred
	hiVal = readByte(fd, REG_HI_DISTANCE, true);
	if (hiVal == -1) return -1;
	if (logLevel <= VERBOSE) cout << "hiVal = " << hiVal << endl;
	
	return ( (hiVal << 8) + loVal);
	//return lidar_read(fd);
}

/* =============================================================================
  Signal Strength
  The sensor transmits a focused infrared beam that reflects off of a target,
  with a portion of that reflected signal returning to the receiver. Distance
  can be calculated by taking the difference between the moment of signal trans-
  mission to the moment of signal reception. But successfully receiving a ref-
  lected signal is heavily influenced by several factors. These factors include:
  target distance, target size, aspect, reflectivity
  The relationship of distance (D) to returned signal strength is an inverse
  square. So, with increase in distance, returned signal strength decreases by
  1/D^2 or the square root of the distance.
  Additionally, the relationship of a target's Cross Section (C) to returned
  signal strength is an inverse power of 4.  The LIDAR-Lite sensor transmits a
  focused near-infrared laser beam that spreads at a rate of approximately .5º
  as distance increases. Up to 1 meter it is about the size of the lens. Beyond
  1 meter, approximate beam spread in degrees can be estimated by dividing the
  distance by 100, or ~8 milliradians. When the beam overfills (is larger than)
  the target, the signal returned decreases by 1/C^4 or the fourth root of the
  target's cross section.
  The aspect of the target, or its orientation to the sensor, affects the obser-
  vable cross section and, therefore, the amount of returned signal decreases as
  the aspect of the target varies from the normal.
  Reflectivity characteristics of the target's surface also affect the amount of
  returned signal. In this case, we concern ourselves with reflectivity of near
  infrared wavelengths.
  Process
  ------------------------------------------------------------------------------
  1.  Read one byte from 0x0e
  Parameters
  ------------------------------------------------------------------------------
  - LidarLiteI2cAddress (optional): Default: 0x62, the default LIDAR-Lite
    address. If you change the address, fill it in here.
  Example Usage
  ------------------------------------------------------------------------------
  1.  //  Basic usage with default i2c address, the signalStrength variable will
      //  hold the signalStrength measurement
      int signalStrength = 0;
      signalStrength = myLidarLiteInstance.signalStrength();
  =========================================================================== */
int LidarLite::signalStrength(){
	if (logLevel <= VERBOSE) cout << "LidarLite::signalStrength" << endl;
	int sigStrength = readByte(fd, REG_SIGNAL_STRENGTH, false);
	if (sigStrength == -1) return -1;
	else return ((int)((unsigned char) sigStrength));
}

//--------------------------------------------------------------	
int LidarLite::maxNoise(){
	if (logLevel <= VERBOSE) cout << "LidarLite::maxNoise" << endl;
	int maxNoise = readByte(fd, REG_MAX_NOISE, false);
	if (maxNoise == -1) return -1;
	else return ((int)((unsigned char) maxNoise));
}

//--------------------------------------------------------------	
int LidarLite::correlationPeakValue(){
	if (logLevel <= VERBOSE) cout << "LidarLite::correlationPeakValue" << endl;
	int corrPeakVal = readByte(fd, REG_CORR_PEAK_VAL, false);
	if (corrPeakVal == -1) return -1;
	else return ((int)((unsigned char) corrPeakVal));
}

//--------------------------------------------------------------	
int LidarLite::transmitPower(){
	if (logLevel <= VERBOSE) cout << "LidarLite::transmitPower" << endl;
	int transPow = readByte(fd, REG_TRANSMIT_POWER, false);
	if (transPow == -1) return -1;
	else return ((int)((unsigned char) transPow));
}

//--------------------------------------------------------------	
int LidarLite::eyeSafetyOn(){
	if (logLevel <= VERBOSE) cout << "LidarLite::eyeSafe" << endl;
	int eyeSafety = -1;
	int stat = status(); // Read from the Mode/Status register
	if (logLevel <= VERBOSE) cout << "status = " << stat << endl;
	if (stat != -1) {
		// If bit8 of stat == 1, eyeSafety is on
		eyeSafety = (((unsigned char) stat ) & STATUS_EYE_SAFETY_ON);
		if (logLevel <= VERBOSE) cout << "eyeSafety = " << eyeSafety << endl;
	}
	return eyeSafety;
}

//--------------------------------------------------------------	
int LidarLite::hardwareVersion() {
	if (logLevel <= VERBOSE) cout << "LidarLite::hardwareVersion" << endl;
	return hwVersion;
}

//--------------------------------------------------------------	
int LidarLite::softwareVersion() {
	if (logLevel <= VERBOSE) cout << "LidarLite::hardwareVersion" << endl;
	return swVersion;
}

//--------------------------------------------------------------	
int LidarLite::status() {
	if (logLevel <= VERBOSE) cout << "LidarLite::status" << endl;
	// return the status register result
	return wiringPiI2CReadReg8(fd, REG_STATUS) ;
}

//--------------------------------------------------------------	
string LidarLite::statusString(int statusInt) {
	stringstream out;
	
	if (statusInt == -1) {
		out << "STATUS: -1 error;";
	} else {
		unsigned char stat = (unsigned char) statusInt;
		
		// Print the hex status byte
		out << "STATUS BYTE: 0x";
		out << setw(2) << setfill('0') << hex << (int) stat;

		if (stat & STATUS_BUSY) out << " busy;";              
		if (stat & STATUS_REFERENCE_OVERFLOW) out << " reference overflow;";            
		if (stat & STATUS_SIGNAL_OVERFLOW) out << " signal overflow;";            
		if (stat & STATUS_PIN) out << " mode select pin;";                 
		if (stat & STATUS_SECOND_PEAK) out << " second peak;";         
		if (stat & STATUS_TIMESTAMP) out << " active between pairs;";                
		if (stat & STATUS_SIGNAL_INVALID) out << " no signal;";             
		if (stat & STATUS_EYE_SAFETY_ON) out << " eye safety;";  	
	}
	return out.str();
}  

//--------------------------------------------------------------	
int LidarLite::readByte(int fd, int reg, bool monitorBusyFlag) {
	if (logLevel <= VERBOSE) cout << "LidarLite::readByte" << endl;
	int busyFlag = 0;
    if(monitorBusyFlag){
    busyFlag = 1;
    }
    int busyCounter = 0;
    while(busyFlag != 0){
        int stat = status(); // Read from the Mode/Status register
        if (logLevel <= VERBOSE) cout << "status = " << stat << endl;
        if (stat != -1) {
            // If bit0 of stat == 1, the LIDAR Lite is busy
            busyFlag = (((unsigned char) stat ) & STATUS_BUSY);
            if (logLevel <= VERBOSE) cout << "busyFlag = " << busyFlag << endl;
        }

        busyCounter++;
        if(busyCounter > 9999){
          if(errorReporting){
                    // errorReporting not yet supported, come again soon
                    cout << "errorReporting not yet supported, come again soon" << endl;
          }
                // Soooo busy, need to bail
                busyCounter = 0;
                cout << "> Bailout" << endl;
          break;
        }
    }
    if(busyFlag == 0){
		if (hardwareVersion() < 21) usleep(1000); //ofSleepMillis(1); 
		
		int output = wiringPiI2CReadReg8(fd, reg);
		if (hardwareVersion() < 21) {
			// Attempt to get LidarLite V1 working with new V2 code
			int i = 0;
			while (true) {
				if (output == -1) {
					// output 
					//ofSleepMillis(20);
					usleep(20000); 
					output = wiringPiI2CReadReg8(fd, reg);
					if (i++ > 20) { // Originally 50
						// Timeout
						if (logLevel <= INFO) cout << "Timeout" << endl;
						return -1;
					}
				} else {
					break;
				}
			}
		}
		return output;
		
  } else {
		return -1;
	}
}			
		

	