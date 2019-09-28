# hardware_interface

## Description

ROS to "packet" peripheral interface for communicating with non-ROS hardware peripherals. 

## Goal

Communicate with non-ROS integrated hardware such as the uart_thrusters_arduino and other devices using hardware interfaces and do not use rosserial. 


## Build Instructions 

* `sudo apt-get install python-serial`
* No build or source needed

## Nodes

### hw_thruster_controller_interface

file: hw_thruster_controller_interface.py

Node name:
* hw_thruster_controller_interface

Topics:

* `rov/cmd_horizontal_vdrive`:
  Subscribes `vector_drive/thrusterPercents` gives the thruster setting from -1000 to 1000 for thrusters T1,2,3,4.
  * `rov/cmd_vertical_vdrive`:
  Subscribes `vector_drive/thrusterPercents` gives the thruster setting from -1000 to 1000 for thrusters T5,6,7,8.

Parameters/Reconfigs:
*  `thrusterControllerPort`: ROS parameter that gets the port to use. By default the port is `/dev/ttyACM0`.
 

## Launch Information
 
`<node respawn="true" pkg="hardware_interface" type="hw_thruster_controller_interface.py"`
	`name="hw_thruster_controller_interface">`
	`<param name="thrusterControllerPort" value="/dev/ttyACM0"/>`
`</node>`

## Troubleshooting

## Contributors 

* Current maintaner: Michael Equi

* Contirbutors:
  * Michael Equi - initial work

## Helpful Resources

* https://pythonhosted.org/pyserial/


