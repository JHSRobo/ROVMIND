# rov_control_interface

## Description

Edit the command vector from the joystick. Serve as the backup copilot page server, handle camera switching, and handle the tcu board. Manages features such as bilinear threshold, 4-way inversion, and joystick sensitivity.

## Goal

Edit the joystick command vectors before being sent to PID and the vector drive node. Allow for more intuitive control vector manipulation.

## Build Instructions

* `sudo apt-get install ros-kinetic-joy`

## Nodes

### drive_control

file: drive_control.cpp

Node name:
* drive_control

Topics:


* `joy/joy1`:
  Subscribes `sensor_msgs/Joy` gives the input from the Logitech Extreme 3D Pro joystick to the node for initial processing

* `joy/joy2`
  Subscribes `sensor_msgs/Joy` gives the input from the Thrustmaster TWCS Throttle to the node for initial processing

* `rov/cmd_vel`:
  Publishes `geometry_msgs/Twist` Outputs 2 command velocity vectors. Not a tradition vector3 implementation: linear.x = linear axis left-right, linear.y = linear axis front-back, linear.z = vertical axis, angular.x = rotational axis, angular.y and angular.z = 0

* `rov/camera_select`:
  Publishes `std_msgs/UInt8` Camera mux control (1-8).

* `tcu/main_relay`:
  Publishes `std_msgs/Bool` If true then the main power relay is on else off.

* `tcu/main_solenoid`:
  Publishes `std_msgs/Bool` If true then the main solenoid is on else off.

Parameters/Reconfigs:

*  `copilot_interface/copilotControlParamsConfig`: Secondary copilot interface through a dynamic reconfigure that contains all the params for a fully functional ROV. See copilot_interface/cfg dir for more info.

* `/useJoyVerticalAxis`: Parameter used to select whether the throttle or the joystick is responsible for controlling verticals. Good for when the throttle is not readily available.



## Launch Information
Launch the rov_control_interface/drive_control node:
`<node pkg="rov_control_interface" type="drive_control" name="drive_control"/>`

Launch the joy node:
`<node respawn="true" pkg="joy" type="joy_node" name="rov_joy" >`
`  <param name="dev" type="string" value="/dev/input/js0" />`
`  <param name="deadzone" value="0.12" />`
`</node>`

## Troubleshooting

If the tcu messages are not immediatly responsive wait for the `serial_node` or `tcu_board` to restart and properly connect to the arduino DUE.

## Contributors

* Current maintaner: Michael Equi

* Contirbutors:
  * Michael Equi - initial work

## Helpful Resources

* http://wiki.ros.org/joy
* Previous vector drive implementation: https://github.com/JHSRobo/Programming/blob/Development/Mako/Bottomside/bottomside/VectorDrive.py
