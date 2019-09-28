# copilot_interface

## Description

Place holder to the actual copilot_interface. copilot_interface.cpp is a test dynamic recofigure server for the copilot_interface cfg.

## Goal

Allow for full control of the ROV without a dedicated copilot page. This allows for multiple features to be worked on without a copilot page and for a temporary backup in case the copilot page fails.

## Build Instructions 

* catkin_make 
* source devel/setup.bash

## Nodes

### copilot_control

file: copilot_interface.cpp

Node name:
* copilot_control

Topics:

Parameters/Reconfigs:
*  `copilot_interface/copilotControlParamsConfig`: provides the temporary interface for the copilot until the full copilot page is tested and ready.


## Launch Information

Test only so no launch file is provided. Do a simple rosrun in order to run the test node. 

## Troubleshooting

## Contributors 

* Current maintaner: Michael Equi

* Contirbutors:
  * Michael Equi - initial work

## Helpful Resources

* http://wiki.ros.org/dynamic_reconfigure


