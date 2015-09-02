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
http://forum.openframeworks.cc/t/raspberry-pi-2-setup-guide/18690
### In brief:
- cd ~
- curl -O http://www.openframeworks.cc/versions/v0.8.4/of_v0.8.4_linuxarmv7l_release.tar.gz
- tar xvf of_v0.8.4_linuxarmv7l_release.tar.gz
- curl https://raw.githubusercontent.com/openframeworks/openFrameworks/master/libs/openFrameworksCompiled/project/linuxarmv7l/config.linuxarmv7l.rpi2.mk -o of_v0.8.4_linuxarmv7l_release/libs/openFrameworksCompiled/project/linuxarmv7l/config.linuxarmv7l.rpi2.mk
- cd ~/of_v0.8.4_linuxarmv7l_release/scripts/linux/debian/
- cp -f ~/scripts/install_dependencies.sh .
- sudo ./install_dependencies.sh
- printf 'export MAKEFLAGS=-j4 PLATFORM_VARIANT=rpi2\n' | sudo tee --append ~/.profile
- cp ~/of_v0.8.4_linuxarmv7l_release/examples/3d/3DPrimitivesExample/ ~/of_v0.8.4_linuxarmv7l_release/apps/myApps/ -r
 
## Rock & Roll
- Add ofxLidarLite folder to your OF Addons folder
- Copy example-LidarLite to myApps folder
- Compile, run and measure distance!
