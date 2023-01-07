#!/bin/bash

# Checks sudo perms
if [[ "$(id -u)" == 0 ]]
  then echo "Please don't run as root"
  exit
fi

cd /home/jhsrobo/Github/ROVMIND/ros_workspace/src/

source /opt/ros/noetic/setup.bash

# Remove old repos
rm -rf RPICamera copilot_interface rov_control_interface launch_files

# Clone new repos
git clone https://github.com/JHSRobo/cameras.git
git clone https://github.com/JHSRobo/copilot_interface.git -b ROVMIND-Rev-2
git clone https://github.com/JHSRobo/rov_control.git
git clone https://github.com/JHSRobo/launch_files.git -b ROVMIND-Rev-2

cd /home/jhsrobo/ROVMIND/ros_workspace

# Update dependencies
rosdep install --from-paths src --ignore-src -r -y

source /home/jhsrobo/Github/ROVMIND/ros_workspace/devel/setup.bash

catkin_make
