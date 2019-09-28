# ms5837

## Description

Python I2C interface for publishing MS5837 external pressure, tempurature, and depth data. Based on blue robotics' ms5837 python library.

## Goal

Easy integration of MS5837 sensors into a ROS software enviroment.

## Build Instructions 

* No compilation needed however msg files must be loaded through a catkin_make and source of the devel/setup.bash
* `sudo apt-get install python-smbus`

## Nodes

### ms5837

file: ms5837_ros.py

Node name:
* ms5837

Topics:

* `rov/ms5837`:
  Publishes `ms5837_data` Custom message with tempurature (celcius), depth, and altitude (meters) data.

Parameters/Reconfigs:
*  `fluidDensity`: This is the parameter that is used to tune the depth sensor based on fluid density in kg/m^3 (default = freshwater = 997, saltwater = 1029)
 

## Launch Information
 
Make sure that a BMP280 is plugged in to the raspberry pi's I2C interface and that the BMP280 has an address of 0x77 using `sudo i2cdetect -y 1`. Launch using `<node type="ms5837_ros.py" pkg="ms5837" name="ms5837_node"> <param name="fluidDensity" value="997"/> </node>` in a launch file or using `rosrun ms5837 ms5837_ros.py`.

## Troubleshooting

If the custom ms5837_data message info can not be read make sure to source de devel/setup.bash
If a an input/output error occurs make sure the I2C bus is operational and the sensor is properly connected:
* `sudo rapi-config` to turn the bus on
* `sudo i2cdetect -y 1` to make sure that the bus is operational and that the ms5837 has an address of 0x76

## Contributors 

* Current maintaner: Michael Equi

* Contirbutors:
  * Michael Equi - initial work

## Helpful Resources

* https://github.com/bluerobotics/ms5837-python
* http://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS5837-02BA01&DocType=Data+Sheet&DocLang=English&DocFormat=pdf&PartCntxt=CAT-BLPS0059




