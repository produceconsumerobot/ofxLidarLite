/*
ThreadedLidarLite.h
For use with OpenFrameworks Addon ofxLidarLite

Created by Produce Consume Robot 2015.
http://produceconsumerobot.com/

This work is licensed under the Creative Commons 
Attribution-ShareAlike 3.0 Unported License. 
To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/3.0/.
	
See LIDAR Lite documentation for more info
http://kb.pulsedlight3d.com/
https://github.com/PulsedLight3D/
*/

#pragma once
#include "LidarLite.hpp"
#include "ofMain.h"

class ThreadedLidarLite : public ofThread
{
    private:
    int _distance;                          // Stores the output locally to permit thread-safe processing
    int _signalStrength;                    // Stores the output locally to permit thread-safe processing
    bool _newOutputAvailable;               // Tracks whether a new output is available from getOutput(); 
    bool _readStarted;                      // Tracks whether a LidarLite distance read has been initiated 
    unsigned int inputCount;				// debug counter
	unsigned int outputCount;				// debug counter
    
    public:
    ThreadedLidarLite();
    ~ThreadedLidarLite();
    void start(bool blocking = false);		// Start a thread, defaults to non-blocking to allow avoid slowing down main thread
	void stop();							// Stop the thread
	void threadedFunction();                // Threaded loop
    bool startRead();                            // initiates a distance and signal strength read
    bool isOutputNew();                     // Returns whether new output data is available
    bool getOutput(int & distance, int & signalStrength);
   
}