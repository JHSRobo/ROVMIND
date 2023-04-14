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
rm -rf coral_modeling launch_files

# Clone new repos
sudo -u jhsrobo git clone https://github.com/JHSRobo/coral_modeling.git
sudo -u jhsrobo git clone https://github.com/JHSRobo/img_capture.git
sudo -u jhsrobo git clone https://github.com/JHSRobo/launch_files.git

cd /home/jhsrobo/ROVMIND/ros_workspace

. /home/jhsrobo/ROVMIND/ros_workspace/devel/setup.bash

sudo -u jhsrobo rosdep update

catkin_make
rosdep install --from-paths src --ignore-src -r -y

sudo chown jhsrobo: -R /home/jhsrobo/ROVMIND
