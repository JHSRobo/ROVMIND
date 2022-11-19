#!/bin/bash

# Checks sudo perms
if [[ "$(id -u)" == 0 ]]
  then echo "Please don't run as root"
  exit
fi

cd ~/ROVMIND/ros_workspace/src/
git clone https://github.com/JHSRobo/RPICamera.git
git clone https://github.com/JHSRobo/copilot_interface.git
git clone https://github.com/JHSRobo/rov_control_interface.git
git clone https://github.com/JHSRobo/launch_files.git

cd ~/ROVMIND/ros_workspace

rosdep install --from-paths src --ignore-src -r -y

catkin_make

source devel/setup.bash
