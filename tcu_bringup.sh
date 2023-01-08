#!/bin/bash

# Check sudo perms
if [[ "$(id -u)" != 0 ]]
  then echo "Please run as root"
  exit
fi

# Edit Ubuntu Files
rm -rf /home/jhsrobo/.bashrc
mv /home/jhsrobo/Github/ROVMIND/bashrc_top /home/jhsrobo/.bashrc
touch /etc/udev/rules.d/joystick.rules
echo "“KERNAL==“HyACMO” MODE==“06666””" >> /etc/udev/rules.d/joystick.rules
echo "192.168.1.100 master" >> /etc/hosts
echo "192.168.1.111 bottomside" >> /etc/hosts
source /home/jhsrobo/.bashrc

# Install required packages
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
  # Above command adds key for ROS update
apt install curl -y
apt install python3-pip -y
apt install net-tools -y
python3 -m pip install smbus -y
python3 -m pip install flask -y

# Installing ROS
sudo -u jhsrobo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo -u jhsrobo curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo -u jhsrobo apt update && apt upgrade --allow-unauthenticated -y
sudo -u jhsrobo apt --fix-broken-install -y
sudo -u jhsrobo apt install ros-noetic-desktop -y
sudo -u jhsrobo source /opt/ros/noetic/setup.bash
sudo -u jhsrobo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
sudo -u jhsrobo rosdep init
sudo -u jhsrobo rosdep update

# Clone our software from Github
sudo -u jhsrobo bash tcu_repo_clone.sh
