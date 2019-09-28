# tcu_touchscreen

## Description

Angular webpage for TCU touchscreen controls

## Goal and User Experience

To provide an easy to use ui for controlling key TCU functions and viewing TCU data such as voltage.

## Build Instructions

* `catkin_make`
* `source devel.setup.bash`
* `cd src`
* `npm i` Install Dependencies
* `npm i -g @angular/cli` Install angular command line interface
* `ng s` (Start Development Server for Webpage)

## Nodes

### NA

Publishers

* `rov/cmd_horizontal_vdrive`:
  Subscribes `vector_drive/thrusterPercents` gives the thruster setting from -1000 to 1000 for thrusters T1,2,3,4.
  * `rov/cmd_vertical_vdrive`:
  Subscribes `vector_drive/thrusterPercents`gives the thruster setting from -1000 to 1000 for thrusters T5,6,7,8.
* `topic_name`:
  Publishes `message_type` info.

## Troubleshooting

## Contributors

* Current maintaner: Caelin Sutch

* Contributors:
  * Alden Parker - Front End Development
  * Caelin Sutch - UI Design, ROS Integration

## Helpful Resources

* http://wiki.ros.org/roslibjs
* https://angular.io