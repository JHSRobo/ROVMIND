#!/bin/bash

# Check sudo perms
if [[ "$(id -u)" != 0 ]]
  then echo "Please run as root"
  exit
fi

apt update && apt upgrade -y

# Edit Ubuntu Files
rm -rf /home/jhsrobo/.bashrc
rm -rf /home/jhsrobo/ROVMIND/tcu_repo_clone.sh
rm -rf /home/jhsrobo/ROVMIND/tcu_bringup.sh
mv /home/jhsrobo/ROVMIND/bashrc_bottom /home/jhsrobo/.bashrc
echo "192.168.1.100 master" >> /etc/cloud/templates/hosts.debian.tmpl
echo "192.168.1.111 bottomside" >> /etc/cloud/templates/hosts.debian.tmpl
touch /etc/udev/rules.d/60-extra-acl.rules
echo 'KERNEL=="ttyUSB[0-9]*", TAG+="udev-acl", TAG+="uaccess"'
. /home/jhsrobo/.bashrc

# Install required packages
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
  # Above command adds key for ROS update
apt install curl python3-pip net-tools python3-rpi.gpio -y
python3 -m pip install adafruit-circuitpython-servokit

# Installing ROS
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
apt update && apt upgrade --allow-unauthenticated -y
apt install ros-noetic-desktop -y
. /opt/ros/noetic/setup.bash
apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential i2c-tools -y
rosdep init
su - jhsrobo -c "rosdep update"

# Clone our software from Github
su - jhsrobo -c "bash /home/jhsrobo/ROVMIND/rov_repo_clone.sh"
