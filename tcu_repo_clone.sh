#!/bin/bash

# Checks sudo perms
if [[ "$(id -u)" == 0 ]]
  then echo "Please don't run as root"
  exit
fi

git pull --hard

cd /home/jhsrobo/ROVMIND/ros_workspace/src/

. /opt/ros/noetic/setup.bash

# Remove old repos
rm -rf cameras copilot_interface rov_control launch_files

# Clone new repos
git clone https://github.com/JHSRobo/camera_view.git
git clone https://github.com/JHSRobo/copilot_interface.git
git clone https://github.com/JHSRobo/rov_control.git
git clone https://github.com/JHSRobo/launch_files.git

cd /home/jhsrobo/ROVMIND/ros_workspace

# Update dependencies
rosdep install --from-paths src --ignore-src -r -y

. /home/jhsrobo/ROVMIND/ros_workspace/devel/setup.bash

catkin_make
