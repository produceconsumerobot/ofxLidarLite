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

#pragma once

#include <string>
using namespace std;

class LidarLite 
{
	public:
		// Status constants
		static const unsigned char STATUS_BUSY = 0x01;	// Busy, indicates that the processor is actively performing an acquisition process.
		static const unsigned char STATUS_REFERENCE_OVERFLOW = 0x02;	// Overflow detected in correlation process associated with the reference  acquisition.
		static const unsigned char STATUS_SIGNAL_OVERFLOW = 0x04;	// Overflow detected in correlation process associated with a signal acquisition.
		static const unsigned char STATUS_PIN = 0x08;	// Indicates that the signal correlation peak is equal to or below correlation record noise threshold
		static const unsigned char STATUS_SECOND_PEAK = 0x10;	// Indicates a second peak was detected. 2nd peak value compared to noise floor.
		static const unsigned char STATUS_TIMESTAMP = 0x20;	// Active between velocity measurement pairs.
		static const unsigned char STATUS_SIGNAL_INVALID = 0x40;	// Signal Invalid – “1” No signal detected, “0’ signal detected.
		static const unsigned char STATUS_EYE_SAFETY_ON = 0x80;	// Indicates that eye safety average power limit has been exceeded and power reduction is in place.
		
		// Constructor
		LidarLite();					
		
		// Initialize the LidarLite
		void begin(int configuration = 0, bool fasti2c = false, bool showErrorReporting = false, char LidarLiteI2cAddress = 0x62);
		
		// Returns whether or not the LidarLite was successfully initialized
		bool hasBegun();			
		
		// configure the LidarLite
		void configure(int configuration = 0);
		
		// Read the distance on the LidarLite
		int distance(bool stablizePreampFlag = true, bool takeReference = true); 
		
		// Read the signal strength of the lidarLite
		int signalStrength();
		
		// Get the status of the LidarLite
		int status();		
		
		// Returns a human readable string describing the status
		static string statusString(int status);
					
		int hardwareVersion();	// Get the Hardware Version of the LidarLite
		
	private:
		int fd;									// file descriptor for I2C interface
		bool errorReporting;		// Not yet implemented
		
		// readByte does the register reading heavy lifting
		int readByte(int fd, int reg, bool monitorBusyFlag); 
	
		static const bool DEBUG_PRINT = false;		// Toggles on and off cout debug messages
		
		// Write register constants
		static const unsigned char REG_MEASURE = 0x00;
		static const unsigned char REG_STATUS = 0x01;
		static const unsigned char REG_HI_DISTANCE = 0x0f;
		static const unsigned char REG_LO_DISTANCE = 0x10;
		static const unsigned char REG_VERSION = 0x41;
		static const unsigned char REG_SIGNAL_STRENGTH = 0x0e;
		
		// Write values
		static const unsigned char VAL_MEASURE = 0x04;
		static const unsigned char VAL_MEASURE_NO_DC_CRCT = 0x03;
};

	
	