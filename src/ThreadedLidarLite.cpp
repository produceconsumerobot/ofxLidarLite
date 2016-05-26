/*
ThreadedLidarLite.h
For use with OpenFrameworks Addon ofxLidarLite

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

#include "ThreadedLidarLite.h"

// *************************************************** 
// Constructor 
// ***************************************************
ThreadedLidarLite::ThreadedLidarLite() {
    _newOutputAvailable = false;
    _readStarted = false;
    inputCount = 0;					// debug counter
	outputCount = 0;				// debug counter
    
    LidarLite();
}
// END Constructor 
// ***************************************************

// *************************************************** 
// Destructor 
// ***************************************************
ThreadedLidarLite::~ThreadedLidarLite() {
    stop();
}
// END Destructor 
// ***************************************************

// ***************************************************  
// Starts a thread, defaults to non-blocking to allow avoid slowing down main thread
// ***************************************************
void ThreadedLidarLite::start(bool blocking) {
	if (!isThreadRunning()) {
		startThread(blocking);
	}
}
// END start
// ***************************************************

// *************************************************** 
// Stops the thread
// ***************************************************
void ThreadedLidarLite::stop() {
	if (isThreadRunning()) {
		waitForThread();
	}
}
// END stop
// ***************************************************

// *************************************************** 
// Threaded loop to perform asyncronous processing.
// Only performs once after setInput set a new input.
// Calling getOutput internally sets isOutputNew() to false.
// ***************************************************
void ThreadedLidarLite::threadedFunction() {
    while (isThreadRunning())
	{
		if (!_readStarted) {
			// Read hasn't been started 
			// so go to sleep
			sleep(4); // >>60Hz to avoid unecessary delays
		}
		else if (lock()) {
			// We got a mutex lock!

			// Read data from the LidarLite
            _distance = distance();
            _signalStrength = signalStrength();

			// Set flag to indicate a new processed frame is available
			_newOutputAvailable = true;

			// Set flag to indicate we've processed the current inputFrame
			_readStarted = false;

			// Unlock the mutex
			unlock();

			// Stop the thread if we've processed everything
			//stop();
		}
	}
}
// END threadedFunction
// ***************************************************

// *************************************************** 
// Starts a read from the LidarLite.
// Returns whether it was successful (if a mutex lock was acquired).
// ***************************************************
bool ThreadedLidarLite::startDistanceRead() {
	if (lock()) {
		// We got a mutex lock!

		_readStarted = true;

		// Unlock the mutex
		unlock();

		inputCount++;

		// Return true if we copied the passed input
		return true;
	}
	else {
		// Return false if we didn't get a mutex lock and couldn't start a read
		return false;
	}
}
// END setInput 
// ***************************************************


// *************************************************** 
// Gets a copy of the latest asynchronously processed input.
// Returns whether set was successful (if a mutex lock was acquired).
// isOutputNew() will return false after calling getOutput until
// the process successfully processes a new output.
// ***************************************************
bool ThreadedLidarLite::getOutput(int & mDistance, int & mSignalStrength) {

	if (lock()) {
		// We got a mutex lock!

		// Deep copy _output to output
		mDistance = _distance;
        mSignalStrength = _signalStrength;

		// Set flag to indicate a new output is NOT available
		_newOutputAvailable = false;

		outputCount++;

		// Unlock the mutex
		unlock();

		// Return true if we copied _output into the passed variable
		return true;
	}
	else {
		// Return false if we didn't get a mutex lock and couldn't set _vidFrame
		return false;
	}
}
// END getOutput
// ***************************************************


// *************************************************** 
// Returns true if a new output is ready.
// ***************************************************
bool ThreadedLidarLite::isOutputNew() {
	return _newOutputAvailable;
}
// ** END isOutputNew **
// ***************************************************