# vector_drive

## Description

Convert translational, rotational, and vertical vectors into thruster values from -1000 to 1000. 

## Goal 

Translate vectors into individual thruster percents for easy manipulation by hardware interface (CAN or other serial device).

## Build Instructions 

* catkin_make
* make sure devel is sourced for custom thrusterPercents message

## Nodes

### horiz_drive

file: horiz_drive.cpp

Node name:
* horiz_drive

Topics:

* `rov/cmd_vel`:
  Subscribes `geometry_msgs/Twist` Inputs 2 command velocity vectors. Not a tradition vector3 implementaton: linear.x = linear axis left-right, linear.y = linear axis front-back, linear.z = vertical axis, angular.x = rotational axis, angular.y and angular.z = 0
  
* `rov/cmd_horizontal_vdrive`:
  Publishes `vector_drive/thrusterPercents` Sends out 4 thrusterPercents (-1000 to 1000) for T1, T2, T3, and T4.


### vert_drive

file: vert_drive.cpp

Node name:
* vert_drive

Topics:

* `rov/cmd_vel`:
  Subscribes `geometry_msgs/Twist` Inputs 2 command velocity vectors. Not a tradition vector3 implementaton: linear.x = linear axis left-right, linear.y = linear axis front-back, linear.z = vertical axis, angular.x = rotational axis, angular.y and angular.z = 0

* `rov/cmd_vertical_vdrive`:
  Publishes `vector_drive/thrusterPercents` Sends out 4 thrusterPercents (-1000 to 1000) for T5, T6, T7, and T8.
 

## Launch Information
 
Launched on bottomside

horizontal vector drive node
`<node pkg="vector_drive" type="horiz_drive" name="horiz_drive"/>`

vertical vector drive node
`<node pkg="vector_drive" type="vert_drive" name="vert_drive"/>`

## Troubleshooting

## Contributors 

* Current maintaner: Michael Equi

* Contirbutors:
  * Michael Equi - initial work

## Helpful Resources

* Previous vector drive implementation: https://github.com/JHSRobo/Programming/blob/Development/Mako/Bottomside/bottomside/VectorDrive.py

