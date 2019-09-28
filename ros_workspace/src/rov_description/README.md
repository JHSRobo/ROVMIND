# rov_description

## Overview Description

Provides the URDF descriptions, the world models, and the simulation interface programs in order to launch a simulation of all the ROV systems.

## Goal and User Experience

The goal of this package is to provide a hardware abstracted environment to test core software elements including drive mechanics, user interface, and autonomous systems. The entire simulation and software environments can be launched by a `full_systems_launch`. The hydrodynamic model of the ROV has been tuned to feel as close to the actual ROV as possible to allow for sufficient proof of concept testing of PID algorithms.

## Build Instructions

* Run `sudo apt-get install protobuf-compiler protobuf-c-compiler`
* catkin_make

## Nodes
* `simulation_interface`

### node_name

* Node Information
 * What exactly does this Node do
 * Where does it run
 * What are its dependencies
 * Any other extra information that should be known about this node (dynamic reconfigures, params, etc.)

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

* Node Inforation
 * What exactly deos this Node do
 * Where does it run
 * What are its dependencies
 * Any other extra information that should be known about this node (dynamic reconfigures, params, etc.)


## Launch Information

Any details about launch files and what they do goes here.
Any remapping information goes here.

## Troubleshooting

* If updating the mesh from solid works make sure that you select the option "Do not translate stl output data to positive space"
* Note: The default axis in solid works are flipped -> In order to fix it add a new coordinate system from the `add reference geometry` tool and flip the y and z axis
* If the simulation fails to load try running the default gazebo empty_world (this may take a while to load initially) and then try the pool_world again.

## Contributors

* Current maintainer: Michael Equi

* Contributors:
  * Michael Equi - Initial simulation setup and programming

## Helpful Resources

* https://github.com/uuvsimulator/uuv_simulator/wiki/Explaining-the-configuration-of-the-ROV-model-description
* https://uuvsimulator.github.io/
* http://www.fossen.biz/wiley/
