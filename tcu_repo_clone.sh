#!/bin/bash

# Checks sudo perms
if [[ "$(id -u)" == 0 ]]
  then echo "Please don't run as root"
  exit
fi

cd ~/ROVMIND/ros_workspace/src/

source /opt/ros/noetic/setup.bash

# Remove old repos
rm -rf RPICamera copilot_interface rov_control_interface launch_files

# Clone new repos
git clone https://github.com/JHSRobo/RPICamera.git
git clone https://github.com/JHSRobo/copilot_interface.git
git clone https://github.com/JHSRobo/rov_control_interface.git -b development
git clone https://github.com/JHSRobo/launch_files.git

cd ~/ROVMIND/ros_workspace

# Update dependencies
rosdep install --from-paths src --ignore-src -r -y

source /home/jhsrobo/ROVMIND/ros_workspace/devel/setup.bash

catkin_make