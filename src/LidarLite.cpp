/*
LidarLite - C++ wrapper for LIDAR Lite interfacing with RaspberryPi
Created by Produce Consume Robot 2015.
http://produceconsumerobot.com/

This work is licensed under the Creative Commons 
Attribution-ShareAlike 3.0 Unported License. 
To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/3.0/.

See LIDAR Lite documentation for more info
http://kb.pulsedlight3d.com/support/solutions/articles/5000549565-detailed-register-descriptions-external
*/

#pragma once

#include "LidarLite.hpp"
#include <sstream>

//--------------------------------------------------------------
LidarLite::LidarLite() {
	_fd = -1;
	
	// initialize the LidarLite
	_fd = wiringPiI2CSetup(ADRS_LIDAR_LITE);
	if (_fd > -1) {
		getStatus();  // Dummy request to wake up device
		usleep(100000);
	}
	
	return;
}

//--------------------------------------------------------------	
bool LidarLite::isInitialized() {
	if (_fd > -1) {
		return true;
	}
	return false;
}

//--------------------------------------------------------------	
int LidarLite::getStatus() {
	// return the status register result
	return wiringPiI2CReadReg8(_fd, REG_STATUS) ;
}

//--------------------------------------------------------------	
int LidarLite::readDistance() {
	int hiVal, loVal, i=0;
	unsigned char val;
	
	unsigned char nackack = 100; // Setup variable to hold ACK/NACK resopnses
	
	// trigger the measure lidar lite to measure a value until an ack is received
	while (nackack != 0) {
		nackack = wiringPiI2CWriteReg8(_fd, REG_MEASURE, VAL_MEASURE);
		usleep(1000);
		//delay(1);
		if (++i > MAX_TRIES) return -1; // after many tries return an error
	}
	
	// Get the low byte, return -1 if error occurred
	if (readByte(_fd, REG_LO_DISTANCE, false, &val) == ERROR_READ) return -1;
	loVal = val;
	if (readByte(_fd, REG_HI_DISTANCE, true, &val) == ERROR_READ) return -1;
	hiVal = val;
	
	return ( (hiVal << 8) + loVal);
	//return lidar_read(_fd);
}

//--------------------------------------------------------------	
int LidarLite::getHardwareVersion() {
	unsigned char val;
	
	// return zero if we got an error reading
	if (readByte(_fd, REG_VERSION, false, &val) == ERROR_READ) return -1;
	
	// otherwise return the value
	return (int) val;
	
	//return lidar_version(_fd);
}

//--------------------------------------------------------------	
std::string LidarLite::getStatusString(int status) {
	std::stringstream out;
	
	unsigned char stat = (unsigned char) status;
	
	// Print the hex status byte
	out << "STATUS BYTE: 0x";
	// add a zero if necessary to maintain correct formatting
	if ((int) stat < 10) out << "0";
	out << (int) stat;

	if (stat & STATUS_BUSY) out << " busy";              
	if (stat & STATUS_REFERENCE_OVERFLOW) out << " reference overflow";            
	if (stat & STATUS_SIGNAL_OVERFLOW) out << " signal overflow";            
	if (stat & STATUS_PIN) out << " mode select pin";                 
	if (stat & STATUS_SECOND_PEAK) out << " second peak";         
	if (stat & STATUS_TIMESTAMP) out << " active between pairs";                
	if (stat & STATUS_SIGNAL_INVALID) out << " no signal";             
	if (stat & STATUS_EYE_SAFETY_ON) out << " eye safety";  	
	
	return out.str();
}     
   
//--------------------------------------------------------------	
int LidarLite::readByte(int fd, int reg,  bool allowZero, unsigned char *val) {
	int i=0, tempVal;
	
	usleep(1000);
	//delay(1); // ms
	while (true) {
		// read from the register
		tempVal = wiringPiI2CReadReg8(fd, reg);
		
		// check for error conditions and try again
		if (tempVal == ERROR_READ || (tempVal==0 && !allowZero) ) {
			usleep(1000);
			//delay (1) ;		// ms
			*val = tempVal;
			if (++i > 100) return ERROR_READ; // after many tries return an error
		} else {
			// success!
			*val = tempVal;
			return 0;
		}
	}
}
		

		
	
	