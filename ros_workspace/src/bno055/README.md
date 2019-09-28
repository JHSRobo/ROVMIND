# bno055

## Description

ROS node for publishing BNO055 IMU data. Uses the adafruit BNO055 python library.

## Goal

Easy integration of BNO055 sensors into a ROS software enviroment.

## Build Instructions 
* pip install adafruit-gpio

PLEASE run on fresh system and update install instructions (not sure what was installed by default on my rpi)

* No compilation needed however msg files must be loaded through a catkin_make and source of the devel/setup.bash


## Nodes

### bno055

file: bno055_ros.py

Node name:
* bno055

Topics:

* `rov/bno055`:
  Publishes `sensor_msgs/Imu` Contains quaternion orientation, linear acceleration (m/s^2), and angular velocity (rads/s) .

* `rov/bno055_info`:
  Publishes `bno055_info` Contains the information on the IMUs calibrartion (accel, gyro, magno, sys) status and tempurature (celcius).

## Launch Information
 
Make sure that a BNO055 is plugged in to the raspberry pi's I2C interface and that the BNO055 has an address of 0x28 using `sudo i2cdetect -y 1`. Launch using `<node type="bno055_ros.py" pkg="bno055" name="bno055_node"/>` in a launch file or using `rosrun bno055 bno055_ros.py`.


## Troubleshooting

If the custom bno55_info message info can not be read make sure to source de devel/setup.bash
If a an input/output error occurs make sure the I2C bus is operational and the sensor is properly connected:
* `sudo rapi-config` to turn the bus on
* `sudo i2cdetect -y 1` to make sure that the bus is operational and that the BNO055 has an address of 0x28

## Contributors 

* Current maintaner: Michael Equi

* Contirbutors:
  * Michael Equi - initial work

## Helpful Resources

* https://github.com/adafruit/Adafruit_Python_BNO055
* https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST_BNO055_DS000_14.pdf


