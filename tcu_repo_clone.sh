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
rm -rf camera_view copilot_interface rov_control launch_files

# Clone new repos
sudo -u jhsrobo git clone https://github.com/JHSRobo/camera_view.git
sudo -u jhsrobo git clone https://github.com/JHSRobo/copilot_interface.git
sudo -u jhsrobo git clone https://github.com/JHSRobo/rov_control.git
sudo -u jhsrobo git clone https://github.com/JHSRobo/launch_files.git

cd /home/jhsrobo/ROVMIND/ros_workspace

# Update dependencies
sudo -u jhsrobo rosdep install --from-paths src --ignore-src -r -y

. /home/jhsrobo/ROVMIND/ros_workspace/devel/setup.bash

catkin_make
sudo chown jhsrobo: -R /home/jhsrobo/ROVMIND
