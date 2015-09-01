#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	myLidarLite = LidarLite();
	
	// Exit if the lidar lite didn't initialize properly
	if (!myLidarLite.isInitialized()) ofApp::exit();
	
	// Print the hardware version of the Lidar Lite
	cout << "LIDAR Lite hardware version: " << myLidarLite.getHardwareVersion() << endl;
}

//--------------------------------------------------------------
void ofApp::update(){

}

//--------------------------------------------------------------
void ofApp::draw(){
	ofSleepMillis(1);
	
	cout << "Frame rate = " << std::fixed << std::setw( 5 ) << std::setprecision( 2 ) 
          << std::setfill( '0' ) <<ofGetFrameRate() << " Hz, ";
		
	// Read the distance
	int distance = myLidarLite.readDistance();
	cout << "Distance = " << distance << " cm, ";

	// Check the status of the lidar lite
	unsigned char status = myLidarLite.getStatus();
	cout << LidarLite::getStatusString(status) << endl;
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}