# tcu_board

## Description

This code provides a rosserial_arduino interface to the TCU's arduino DUE. The DUE subscirbes to 3 topics for controlling the main relay, main solenoid, and RGBW LEDs. The DUE publishes one topic which contains tempurature, humidity, voltage, and current data. 

## Goal

Make an expandible and modular interface for quickly adding hardware based features onto the TCU. 

## Instructions 

* Follow the README.md in the ros_workspace/sketchbook directory 
* Upload to an arduino DUE through the programming port
* Keep the DUE plugged into the arduino DUE's programming port when running the ros node
  * `rosrun rosserial_python serial_node.py /dev/ttyACM0` 
  * OR 
  * `roslaunch launch_files topside.launch`

## Communication Architecture
* rosserial
* Publishers
  * `tcu/tcu_data`
* Subscirbers
  * `tcu/leds`
  * `tcu/main_relay`
  * `tcu/main_solenoid`

## Troubleshooting
* Make sure the ros_lib file in the sketchook/libraries folder is up to date

## Contributors 

* Current maintaner: Michael Equi

* Contirbutors:
  * Michael Equi - inital work

## Helpful Resources

* sketchbook/README.md

