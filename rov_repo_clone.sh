#!/bin/bash

# Check sudo perms
if [[ "$(id -u)" != 0 ]]
  then echo "Please run as root"
  exit
fi

git pull --hard

cd /home/jhsrobo/ROVMIND/ros_workspace/src/

. /opt/ros/noetic/setup.bash

# Remove old repos
rm -rf thrusters depth_sensor gpio_control launch_files

# Clone new repos
sudo -u jhsrobo git clone https://github.com/JHSRobo/thrusters.git
sudo -u jhsrobo git clone https://github.com/JHSRobo/depth_sensor.git
sudo -u jhsrobo git clone https://github.com/JHSRobo/gpio_control.git
sudo -u jhsrobo git clone https://github.com/JHSRobo/launch_files.git

cd /home/jhsrobo/ROVMIND/ros_workspace

rosdep update

. /home/jhsrobo/ROVMIND/ros_workspace/devel/setup.bash

catkin_make
rosdep install --from-paths src --ignore-src -r -y

# Temporary fix below, find a better way to do this
chmod +x /home/jhsrobo/ROVMIND/ros_workspace/src/thrusters/src/thrusterInterface.py
