This folder is for all arduino and microntroller (AVR/ARM Coretx-M) code/project files

In order to set this as your arduino path run:
* `sudo arduino` 
* Click on preferences 
* Change the `/root/Arduino` sketchbook location to `/home/"username"/Desktop/ROV_Test_Bench/ros_workspace/sketchbook`
* Click ok
* Open arduino and check that the libraries folder is detected
* `sudo apt-get install ros-kinetic-rosserial-arduino`
* `sudo apt-get install ros-kinetic-rosserial`

For updating ros_lib with new .h messages
* `rm -R ros_lib`
* `rosrun rosserial_arduino make_libraries.py .` Make sure you are in your libraries directory

Add `#define USE_USBCON` before including the ros/ros.h file when using arduino DUE (still use programming port)

https://learn.adafruit.com/adafruit-all-about-arduino-libraries-install-use/how-to-install-a-library

http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup



