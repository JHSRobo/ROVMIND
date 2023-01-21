#!/bin/bash

# Check sudo perms
if [[ "$(id -u)" != 0 ]]
  then echo "Please run as root"
  exit
fi

# Edit Ubuntu Files
rm -rf /home/jhsrobo/.bashrc
rm -rf /home/jhsrobo/ROVMIND/tcu_repo_clone.sh
rm -rf /home/jhsrobo/ROVMIND/tcu_bringup.sh
cp /home/jhsrobo/ROVMIND/bashrc_bottom /home/jhsrobo/.bashrc

echo "192.168.1.100 master" >> /etc/hosts
echo "192.168.1.111 bottomside" >> /etc/hosts

python3 -m pip install adafruit-circuitpython-servokit

# Install ROS
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
apt update && apt upgrade --allow-unauthenticated -y
apt install ros-noetic-desktop -y
. /opt/ros/noetic/setup.bash
apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential i2c-tools -y
rosdep init
sudo -u jhsrobo rosdep update

# Enable i2c
echo "deb http://archive.raspberrypi.org/debian/ buster main" >> /etc/apt/sources.list
apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 7FA3303E
apt update && sudo apt-get dist-upgrade -y
apt install raspi-config -y
sudo raspi-config nonint do_i2c 0

apt install curl python3-pip net-tools
pip install RPi.GPIO # Not listed in python.yaml for ubuntu 20.04

touch /etc/udev/rules.d/60-extra-acl.rules
. /home/jhsrobo/.bashrc

# Clone our software from Github
su - jhsrobo -c "bash /home/jhsrobo/ROVMIND/rov_repo_clone.sh"
