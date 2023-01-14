#!/bin/bash

# Check sudo perms
if [[ "$(id -u)" != 0 ]]
  then echo "Please run as root"
  exit
fi

# Edit Ubuntu Files
rm -rf /home/jhsrobo/.bashrc
cp /home/jhsrobo/ROVMIND/bashrc_top /home/jhsrobo/.bashrc
touch /etc/udev/rules.d/joystick.rules

if grep -Fxq "“KERNAL==“HyACMO” MODE==“06666””" /etc/udev/rules.d/joystick.rules
  then echo "“KERNAL==“HyACMO” MODE==“06666””" >> /etc/udev/rules.d/joystick.rules
fi
if grep -Fxq "192.168.1.100 master" /etc/hosts
  then echo "192.168.1.100 master" >> /etc/hosts
fi
if grep -Fxq "192.168.1.111 bottomside" /etc/hosts
  then echo "192.168.1.111 bottomside" >> /etc/hosts
fi

. /home/jhsrobo/.bashrc

# Install required packages
# This is the section where we install packages that we can't install with rosdep
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
  # Above command adds key for ROS update
apt install curl -y
apt install python3-pip -y
apt install net-tools -y
pip install simple-pid # This package will not work with rosdep for whatever reason

# Installing ROS
if grep -Fxq 'deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main' /etc/apt/sources.list.d/ros-latest.list
  sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
fi
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
apt update && apt upgrade --allow-unauthenticated -y
apt --fix-broken-install -y
apt install ros-noetic-desktop -y
. /opt/ros/noetic/setup.bash
apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
rosdep init
sudo -u jhsrobo rosdep update

# Clone our software from Github
sudo -u jhsrobo bash tcu_repo_clone.sh
