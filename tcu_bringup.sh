#!/bin/bash

# Check sudo perms
if [[ "$(id -u)" != 0 ]]
  then echo "Please run as root"
  exit
fi

# Edit Ubuntu Files
rm -rf /home/jhsrobo/.bashrc
rm -rf /home/jhsrobo/ROVMIND/rov_repo_clone.sh
rm -rf /home/jhsrobo/ROVMIND/rov_bringup.sh
cp /home/jhsrobo/ROVMIND/bashrc_top /home/jhsrobo/.bashrc
touch /etc/udev/rules.d/joystick.rules

echo "“KERNAL==“HyACMO” MODE==“06666””" >> /etc/udev/rules.d/joystick.rules
echo "192.168.1.100 master" >> /etc/hosts
echo "192.168.1.110 opside" >> /etc/hosts
echo "192.168.1.111 bottomside" >> /etc/hosts

. /home/jhsrobo/.bashrc

# Make nano a bit friendlier for tweaking
touch /home/jhsrobo/.nanorc
touch /root/.nanorc
echo -e "set mouse\nset autoindent\nset linenumbers" >> /home/jhsrobo/.nanorc
echo -e "set mouse\nset autoindent\nset linenumbers" >> /root/.nanorc

# Install required packages
# This is the section where we install packages that we can't install with rosdep
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
  # Above command adds key for ROS update
apt install curl ros-noetic-joy python3-pip net-tools -y
pip install simple-pid # This package will not work with rosdep for whatever reason

# Installing ROS
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
apt update && apt upgrade --allow-unauthenticated -y
apt --fix-broken-install -y
apt install ros-noetic-desktop -y
. /opt/ros/noetic/setup.bash
apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
rosdep init
sudo -u jhsrobo rosdep update

# Clone our software from Github
sudo bash /home/jhsrobo/ROVMIND/tcu_repo_clone.sh
