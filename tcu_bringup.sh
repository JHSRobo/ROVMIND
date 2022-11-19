#!/bin/bash

# Check sudo perms
if [[ "$(id -u)" != 0 ]]
  then echo "Please run as root"
  exit
fi

# Edit Ubuntu Files
echo "alias topside=\"cd /home/jhsrobo/ROVMIND/ros_workspace && rosrun launch_files bottomside.launch\"" >> ~/.bashrc
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source /home/jhsrobo/ROVMIND/ros_workspace/devel/setup.bash"
echo "export ROS_MASTER_URI=http://master:11311" >> ~/.bashrc
echo "export ROS_HOSTNAME=master" >> ~/.bashrc
echo "export ROS_IP=192.168.1.100" >> ~/.bashrc
echo "export ROS_DISTRO=noetic" >> ~/.bashrc
echo "alias topside=\"roslaunch launch_files topside.launch\"" >> ~/.bashrc
echo "alias Topside=\"roslaunch launch_files topside.launch\"" >> ~/.bashrc
touch /etc/udev/rules.d/joystick.rules
echo "“KERNAL==“HyACMO” MODE==“06666””" >> /etc/udev/rules.d/joystick.rules
echo "192.168.1.100 master" >> /etc/hosts
echo "192.168.1.111 bottomside" >> /etc/hosts
source ~/.bashrc

# Install required packages
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
  # Above command adds key for ROS update
apt install curl -y
apt install python3-pip -y
apt install net-tools -y
python3 -m pip install smbus -y

# Installing ROS
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
apt update && apt upgrade --allow-unauthenticated -y
apt --fix-broken-install
apt install ros-noetic-desktop -y
source /opt/ros/noetic/setup.bash
apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
rosdep init
sudo -u jhsrobo rosdep update

# Clone our software from Github
sudo -u jhsrobo bash tcu_repo_clone.sh