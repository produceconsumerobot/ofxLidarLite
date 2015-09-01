/*
LidarLite - C++ wrapper for LIDAR Lite interfacing with RaspberryPi
Created by Produce Consume Robot 2015.
http://produceconsumerobot.com/

This work is licensed under the Creative Commons 
Attribution-ShareAlike 3.0 Unported License. 
To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/3.0/.

Leverages WiringPi by drogon
	
Requirements:
	Install wiring pi - http://wiringpi.com/
	Add PROJECT_LDFLAGS += -lwiringPi to the PROJECT LINKER FLAGS section of config.make
	
See LIDAR Lite documentation for more info
http://kb.pulsedlight3d.com/support/solutions/articles/5000549565-detailed-register-descriptions-external
*/

#pragma once

#include <string>
#include <wiringPiI2C.h>

class LidarLite 
{
	public:
		// Status constants
		static const unsigned char STATUS_READY = 0x00;	// All systems go!
		static const unsigned char STATUS_BUSY = 0x01;	// Busy, indicates that the processor is actively performing an acquisition process.
		static const unsigned char STATUS_REFERENCE_OVERFLOW = 0x02;	// Overflow detected in correlation process associated with the reference  acquisition.
		static const unsigned char STATUS_SIGNAL_OVERFLOW = 0x04;	// Overflow detected in correlation process associated with a signal acquisition.
		static const unsigned char STATUS_PIN = 0x08;	// Indicates the state of the Mode Select external pin.  De-bounced, masked from output signals, inverted.
		static const unsigned char STATUS_SECOND_PEAK = 0x10;	// Indicates a second peak was detected. 2nd peak value compared to noise floor.
		static const unsigned char STATUS_TIMESTAMP = 0x20;	// Active between velocity measurement pairs.
		static const unsigned char STATUS_SIGNAL_INVALID = 0x40;	// Signal Invalid – “1” No signal detected, “0’ signal detected.
		static const unsigned char STATUS_EYE_SAFETY_ON = 0x80;	// Indicates that eye safety average power limit has been exceeded and power reduction is in place.
		
		LidarLite();					// Constructor
		bool isInitialized();			// Returns whether or not the LidarLite was successfully initialized
		int getStatus();		// Get the status of the LidarLite
		static std::string getStatusString(int status);	// Returns a string describing the status
		int readDistance();				// Read the distance on the LidarLite
		int getHardwareVersion();	// Get the Hardware Version of the LidarLite
		
	private:
		int _fd;
		
		int readByte(int fd, int reg,  bool allowZero, unsigned char *val); 
		
		// Lidar Lite address 
		static const unsigned char ADRS_LIDAR_LITE = 0x62;
		
		// Write register constants
		static const unsigned char REG_MEASURE = 0x00;
		static const unsigned char REG_STATUS = 0x47;
		static const unsigned char REG_HI_DISTANCE = 0x0f;
		static const unsigned char REG_LO_DISTANCE = 0x10;
		static const unsigned char REG_VERSION = 0x41;
		
		// Write values
		static const unsigned char VAL_MEASURE = 0x04;
		
		// constants
		static const int ERROR_READ = -1;
		static const int MAX_TRIES = 100;
};

	
	