# ofxLidarLite
C++ object for interfacing with the LIDAR Lite on Raspberry PI wrapped up as an OpenFrameworks Addon with example project.

-----------
## Configure your hardware
![alt LidarLite pinout](https://github.com/produceconsumerobot/ofxLidarLite/blob/master/PinoutI2CWiring.png)

### Pin Connections
- Double-check these with your latest hardware documentation.
- http://www.element14.com/community/docs/DOC-73950/l/raspberry-pi-2-model-b-gpio-40-pin-block-pinout
- Svr is RaspberryPi

Svr |  Desc |  LIDAR
:--:|:-----:|:------:
 2  |   5V  |  1 (Red)
 3  |   SDA | 5
    |       | 
 5  |   SCL |   4
 6  |   GND |   6

## Install wiring pi
- http://wiringpi.com/ 
 
### In brief:
- sudo apt-get -y install libi2c-dev
- mkdir ~/src
- cd ~/src
- git clone git://git.drogon.net/wiringPi
- cd wiringPi
- ./build

## Setup Linux options to use I2C
- Turn on I2C by calling sudo raspi-config 
- printf 'i2c_bcm2708\ni2c-dev\n' | sudo tee --append /etc/modules
 
## Setup OpenFrameworks
http://openframeworks.cc/setup/raspberrypi/raspberry-pi-getting-started/
 
## Rock & Roll
- Add ofxLidarLite folder to your OF Addons folder
- Copy example-LidarLite to myApps folder
- Compile, run and measure distance!
