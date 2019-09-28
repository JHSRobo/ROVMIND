# bmp280

## Description

Python I2C interface for publishing BMP280 pressure and tempurature data. 

## Goal

Easy integration of BMP280 sensors into a ROS software enviroment.

## Build Instructions 

* No compilation needed however msg files must be loaded through a catkin_make and source of the devel/setup.bash
* SMBUS2 is self contatined within the package: no download needed

## Nodes

### bmp280

file: bmp280_ros.py

Node name:
* bmp280

Topics:

* `rov/bmp280`:
  Publishes `bmp280_data` Custom message with tempurature (celcius), pressure (pascal and atm), and altitude (meters) data.
 

## Launch Information
 
Make sure that a BMP280 is plugged in to the raspberry pi's I2C interface and that the BMP280 has an address of 0x77 using `sudo i2cdetect -y 1`. Launch using `<node type="bmp280_ros.py" pkg="bmp280" name="bmp280_node"/>` in a launch file or using `rosrun bmp280 bmp280_ros.py`.

## Troubleshooting

If the custom bmp280_data message info can not be read make sure to source de devel/setup.bash
If a an input/output error occurs make sure the I2C bus is operational and the sensor is properly connected:
* `sudo rapi-config` to turn the bus on
* `sudo i2cdetect -y 1` to make sure that the bus is operational and that the BMP280 has an address of 0x77

## Contributors 

* Current maintaner: Michael Equi

* Contirbutors:
  * Michael Equi - initial work

## Helpful Resources

* https://www.adafruit.com/product/2651
* https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BMP280-DS001-19.pdf


