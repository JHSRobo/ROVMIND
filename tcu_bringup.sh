#!/bin/bash
#Edit files syntax:
#echo "alias cameras=\"rosrun camera_viewer switcher.py\"" >> ~/.bashrc

#Check sudo perms
if [[ "$(id -u)" != 0 ]]
  then echo "Please run as root"
  exit
fi

apt update && apt upgrade
apt install curl -y
apt install python3-pip -y
apt install net-tools -y
python3 -m pip install smbus -y

# Edit Ubuntu Files
echo "alias topside=\"cd /home/jhsrobo/ROVMIND/ros_workspace && rosrun launch_files bottomside.launch\"" >> ~/.bashrc
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source /home/jhsrobo/ROVMIND/ros_workspace/devel/setup.bash"
echo "export ROS_MASTER_URI=http://master:11311" >> ~/.bashrc
echo "export ROS_HOSTNAME=master" >> ~/.bashrc
echo "export ROS_IP=192.168.1.100" >> ~/.bashrc
touch /etc/udev/rules.d/joystick.rules
echo "“KERNAL==“HyACMO” MODE==“06666””" >> /etc/udev/rules.d/joystick.rules
echo "192.168.1.100 master" >> /etc/hosts
echo "192.168.1.111 bottomside" >> /etc/hosts
source ~/.bashrc

#Installing ROS
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt update
apt install ros-noetic-desktop
source /opt/ros/noetic/setup.bash
apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
rosdep init
sudo -u jhsrobo rosdep update

#Clone our software from Github
sudo -u jhsrobo bash tcu_repo_clone.sh