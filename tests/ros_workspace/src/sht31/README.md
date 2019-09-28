# sht31

## Description

Python I2C interface for publishing SHT31 humidity and tempurature data. 

## Goal

Easy integration of SHT31 sensors into a ROS software enviroment.

## Build Instructions 

* No compilation needed however msg files must be loaded through a catkin_make and source of the devel/setup.bash
* SMBUS2 is self contatined within the package: no download needed

## Nodes

### sht31

file: sht31_ros.py

Node name:
* sht31

Topics:

* `rov/sht31`:
  Publishes `sht31_data` Custom message with tempurature (celcius and fahrenheit) and humidity (%).

## Launch Information
 
Make sure that a SHT31 is plugged in to the raspberry pi's I2C interface and that the SHT31 has an address of 0x44 using `sudo i2cdetect -y 1`. Launch using `<node type="sht31_ros.py" pkg="sht31" name="sht31_node"/>` in a launch file or using `rosrun sht31 sht31_ros.py`.

## Troubleshooting

If the custom sht31 message info can not be read make sure to source de devel/setup.bash
If a an input/output error occurs make sure the I2C bus is operational and the sensor is properly connected:
* `sudo rapi-config` to turn the bus on
* `sudo i2cdetect -y 1` to make sure that the bus is operational and that the SHT31 has an address of 0x44

## Contributors 

* Current maintaner: Michael Equi

* Contirbutors:
  * Michael Equi - initial work

## Helpful Resources

* https://learn.adafruit.com/adafruit-sht31-d-temperature-and-humidity-sensor-breakout/overview
* https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/0_Datasheets/Humidity/Sensirion_Humidity_Sensors_SHT3x_Datasheet_digital.pdf

