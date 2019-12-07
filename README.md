# ROVMIND

The goal of this project is to develop preseason software technologies based on ROS. This project will explorer development paths and verify new technologies before more permanent hardware development. Key areas of testing are cameras, vector drive, controllers, and PID algortihms. Additionally new workflows/integration tactics are to be tested and documented (Travis CI, Docker, node documentation, doxygen). This project will expire at the beginning of the 2018-2019 robotics season.   

Travis:
   - Release: ![Travis](https://travis-ci.org/JHSRobo/ROVMIND.svg?branch=release) Master: ![Travis](https://travis-ci.org/JHSRobo/ROVMIND.svg?branch=master)

Code Factor: 
   - Release: [![CodeFactor](https://www.codefactor.io/repository/github/jhsrobo/rovmind/badge/release)](https://www.codefactor.io/repository/github/jhsrobo/rovmind/overview/release) Master: [![CodeFactor](https://www.codefactor.io/repository/github/jhsrobo/rovmind/badge/master)](https://www.codefactor.io/repository/github/jhsrobo/rovmind/overview/master)

## Release Page and Feature Addition Form

All latest releases will be posted on the [release page](https://jhsrobo.github.io/releasepage/). There is also a feature addition form on this page so you can submit a feature that you want to be in the next release. Please only add the feature if it is complete and in development.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on the main ROV system (Master).

FOLLOW:
*  https://docs.google.com/document/d/1tYhxP1HbuTF7Nzl1WnGgJKkUe-KNsBEhQZDjsyUwQTU/edit

*Always run IDE's from terminal if on Ubuntu (just type the name of the IDE in terminal and click enter ex. clion)*

### Launching/Running

Install ROS dependenceis: `rosdep install --from-paths src --ignore-src -r -y`

Follow Test Bench Setup Steps if Running on Test Bench
* https://docs.google.com/document/d/1srYgNUE4k3DVHkUv1TwUJawfUWw6kGkveDhAvulWMZ0/edit#heading=h.wyzbdb7zgifi

Locally:
* https://docs.google.com/document/d/16LQRhCJBEe_hL-SV67Vvk3oH7I2nqO0X7NjB1t-9Mtg/edit#heading=h.v4rl9rent2ka

### Prerequisites

What things you should to install to develop and run software and how to install them

On the RPI turn on the CSI, SPI, I2C, and UART interfaces using `sudo raspi-config`

Setup the I2C interface on Ubuntu Mate
* `sudo apt-get install python-smbus`
* `cd /boot/config.txt`
* uncomment `dtparam=i2c_arm=off` and change to `dtparam=i2c_arm=on`
* uncomment `dtparam=i2c_arm_baudrate=100000` and change to `dtparam=i2c_arm_baudrate=400000`
* restart pi

Setup the ros_lib file for arduino serial
* See sketchbook README.md documentation

RPI Camera node setup
* See raspicam_node README.md documentation

Turn RPI Safemode OFF
* Type cd /boot on the bottomside RPI
* Type sudo nano config.txt
* Type CTRL + W and then search for avoid_safe and hit enter
* Go to the bottom of the section and uncomment avoid_safemode=1
* Type CTRL + X and then Y to exit

### Network Setup

What things you need to do so that the ROS network operates properly

On ubuntu 16.04 go to Network Connections app and add a new ethernet connection (name the connection `ROVEthernetConnection`)
* On the topside computer have a static (manual) IP of `192.168.1.100`, netmask `24`, Gateway `92.168.1.1`, DNS server `27.0.1.1, 8.8.8.8, 192.168.1.1`
* On the bottomside computer have a static (manual) IP of `192.168.1.111`, netmask `24`, Gateway `192.168.1.1`, DNS server `127.0.1.1, 8.8.8.8, 192.168.1.1`
* Run the setupROSNetwork.sh script in the scripts folder

Once the network connection has been verified (on bottomside `ping master` / on topside `ping bottomside`)
* Run `sshSetup.sh` in the scripts folder
* Do not add any paraphrases
* on bottomside `ssh master` / on topside `ssh bottomside`
* Make sure both work without entering a password

#### Network Setup DEBUG
* IF you recieve `/usr/bin/ssh-copy-id: ERROR: ssh: connect to host bottomside port 22: Connection refused` go to the opposite machine from the one you recieved it on and run the following:
    * `sudo rm /etc/ssh/sshd_config`
    * `sudo apt-get purge openssh-server`
    * `sudo apt-get install openssh-server`
    * `./sshSetup.sh`

* If topside and bottomside aren't communicating do the following:
    * ping 192.168.1.100 from bottomside, and 192.168.1.111 from topside (If they don't communicate, make sure all the hardware is setup correctly. Also check your virtual machine network settings, and make sure it is briding to the network (directly connected to physical network and replicating physical connection state)

Other usefull links for common problems:
* https://superuser.com/questions/421004/how-to-fix-warning-about-ecdsa-host-key
* https://askubuntu.com/questions/762541/ubuntu-16-04-ssh-sign-and-send-pubkey-signing-failed-agent-refused-operation
* https://answers.ros.org/question/41446/a-is-not-in-your-ssh-known_hosts-file/

### Installing OS

A step by step series of examples that tell you how to get a development env running

On your Raspberry Pi 3B make sure you are running ubuntu mate 16.04 (image here https://drive.google.com/open?id=1497jupJ2dBQqy_o_x5JBPTjY3lto7-rI)
* cat /etc/os-release

### Setup Topside Peripheral Communication
* run `sudo usermod -a -G dialout $USER` to give proper permissions to USB interface programs (similar to `sudo chmod a+rw /dev/tty...`)

### Simulation Setup

* See https://uuvsimulator.github.io/installation.html#creating-and-configuring-a-workspace
* Check to see if gazebo version 9 is installed `gazebo --version`
* Run `sudo apt-get update` and `sudo apt-get upgrade`
* Run `sudo apt-get install ros-melodic-uuv-simulator`
* `roslaunch rov_description full_systems_launch.launch` or `roslaunch rov_description partial_systems_launch.launch` and `roslaunch simulate_rov.launch` on another machine
* `rosrun rov_description simulation_interface.py`

### Bottomside Build Instructions

* When building bottomside with catkin_make, type the command catkin_make -DCATKIN_WHITELIST_PACKAGES="vector_drive;hardware_interface;raspicam_node;drq1250;ms5837;sensor_readout"

### Intel Realsense Setup instructions
## Ubuntu

* Follow the driver installation instructions: https://github.com/intel-ros/realsense
* Test using `roslaunch realsense2_camera rs_camera.launch`

Note: The resolution of the realsense camera should always run at 1280x720 for the D415 and 848x480 for the D435

## RP 3B+

* Follow the driver installation instructions: https://github.com/IntelRealSense/librealsense/blob/master/doc/RaspberryPi3.md
   * https://github.com/IntelRealSense/librealsense/blob/development/doc/installation.md
* cd ros_workspace
* `git submodule init`
* `git submodule update`
* `catkin_make`
* Test using `roslaunch realsense2_camera rs_camera.launch`
* Run on the rov by launching the `rov_control` file

## Acknowledgments

* README Template source https://gist.github.com/PurpleBooth/109311bb0361f32d87a2
* Inspiration
* etc

v0.1
