#!/bin/bash

# Checks sudo perms
if [[ "$(id -u)" == 0 ]]
  then echo "Please don't run as root"
  exit
fi

cd /home/jhsrobo/ROVMIND/ros_workspace/src/

. /opt/ros/noetic/setup.bash

# Remove old repos
rm -rf thrusters depth_sensor gpio_control launch_files

# Clone new repos
git clone https://github.com/JHSRobo/thrusters.git
git clone https://github.com/JHSRobo/depth_sensor.git
git clone https://github.com/JHSRobo/gpio_control.git
git clone https://github.com/JHSRobo/launch_files.git

cd /home/jhsrobo/ROVMIND/ros_workspace

# Update dependencies
rosdep install --from-paths src --ignore-src -r -y

. /home/jhsrobo/ROVMIND/ros_workspace/devel/setup.bash

catkin_make
