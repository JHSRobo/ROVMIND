#!/bin/bash

# Checks sudo perms
if [[ "$(id -u)" == 0 ]]
  then echo "Please don't run as root"
  exit
fi

cd ~/Github

rm -rf ./ROVMIND
git clone --branch V2.13 https://github.com/JHSRobo/ROVMIND.git

cd ~/Github/ROVMIND/ros_workspace/src/
git clone https://github.com/JHSRobo/RPICamera.git
git clone https://github.com/JHSRobo/copilot_interface.git
git clone https://github.com/JHSRobo/rov_control_interface.git

cd ~/Github/ROVMIND/ros_workspace

rosdep install --from-paths src --ignore-src -r -y

catkin_make

source devel/setup.bash
