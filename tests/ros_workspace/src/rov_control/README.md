# rov_control

## Overview Description

This package contains code and launch files in order to run the necessary odometry, sensor fusion, and PID algorithms to augment pilot ability.

## Goal and User Experience

What is the goal of the code and why it was done the way it was. How does this add to the user experience. What should the user experience with this code be like (some packages will have a lot more on this than others), provide examples and walk through the features. This will help with tech report.


## Requirements

List the steps that should be taken to make sure this package is meeting its requirements and functioning as intended.

## Build Instructions

* Anything extra steps for building this package and running the nodes (ex. install)
  * `sudo apt-get install ...`

## Nodes

### node_name

* Node Information
 * What exactly deos this Node do
 * Where does it run
 * What are its dependencies
 * Any other extra information that should be known about this node (dynamic reconfigs, params, etc.)

example ---------------------------------------------------------------------------------------------

file: hw_thruster_controller_interface.py

Node name:
* hw_thruster_controller_interface

Topics:

* `rov/cmd_horizontal_vdrive`:
  Subscribes `vector_drive/thrusterPercents` gives the thruster setting from -1000 to 1000 for thrusters T1,2,3,4.
  * `rov/cmd_vertical_vdrive`:
  Subscribes `vector_drive/thrusterPercents`gives the thruster setting from -1000 to 1000 for thrusters T5,6,7,8.
* `topic_name`:
  Publishes `message_type` info.

Services:
* `service_name`: info

Parameters/Reconfigs:
*  `parameter_name`: info


### other_node_name (if applicable)

* Node Information
 * What exactly does this Node do
 * Where does it run
 * What are its dependencies
 * Any other extra information that should be known about this node (dynamic reconfigs, params, etc.)


## Launch Information

Any details about launch files and what they do goes here.
Any remapping information goes here.

## Troubleshooting

## Contributors

* Current maintainer:

* Contributors:
  * name (anyone who writes stuff into this package) - role (what was the work said person did)

## Helpful Resources

* Links, information, external articles that were helpful in creating anything in this package
