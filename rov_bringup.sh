#!/bin/bash

# Check sudo perms
if [[ "$(id -u)" != 0 ]]
  then echo "Please run as root"
  exit
fi

adduser jhsrobo gpio

# Edit Ubuntu Files
rm -rf /home/jhsrobo/.bashrc
rm -rf /home/jhsrobo/ROVMIND/tcu_repo_clone.sh
rm -rf /home/jhsrobo/ROVMIND/tcu_bringup.sh
cp /home/jhsrobo/ROVMIND/bashrc_bottom /home/jhsrobo/.bashrc

# Template file for /etc/hosts
echo "192.168.1.100 master" >> /etc/cloud/templates/hosts.debian.tmpl
echo "192.168.1.111 bottomside" >> /etc/cloud/templates/hosts.debian.tmpl

# Make nano a bit friendlier for tweaking
touch /home/jhsrobo/.nanorc
touch /root/.nanorc
echo -e "set mouse\nset autoindent\nset linenumbers" >> /home/jhsrobo/.nanorc
echo -e "set mouse\nset autoindent\nset linenumbers" >> /root/.nanorc

# Install ROS
sudo apt install curl -y
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
apt update && apt upgrade --allow-unauthenticated -y
apt install ros-noetic-desktop -y
. /opt/ros/noetic/setup.bash
apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential i2c-tools -y
sudo rosdep init
sudo -u jhsrobo rosdep update

# Enable i2c
wget -p -O ./raspi-config_20221214_all.deb https://archive.raspberrypi.org/debian/pool/main/r/raspi-config/raspi-config_20221214_all.deb
apt update && apt-get dist-upgrade -y
apt -y install libnewt0.52 whiptail parted triggerhappy lua5.1 alsa-utils
apt install -fy
dpkg -i ./raspi-config_20221214_all.deb
rm raspi-config_20221214_all.deb
raspi-config nonint do_i2c 0

# Enable GPIO
sudo chown root.gpio /dev/gpiomem
sudo chmod g+rw /dev/gpiomem

apt install python3-pip python3-rpi.gpio net-tools -y
python3 -m  pip install RPi.GPIO # Not listed in python.yaml for ubuntu 20.04
python3 -m pip install adafruit-circuitpython-servokit

touch /etc/udev/rules.d/60-extra-acl.rules
. /home/jhsrobo/.bashrc
bot
# Clone our software from Github
bash /home/jhsrobo/ROVMIND/rov_repo_clone.sh
