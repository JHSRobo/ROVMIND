# sensor_readout
 ## Description
* This package publish all of the sensor data that is useful from the raspberry pi into 6 different topics, one for each type of sensor data.
 ## Goal 
* The goal of the code is to make accessing the numbers that the sensors readout possible and eaiser. It was made into several topics because then we (software) don't have to deal with arrays and array indexing.
 ## Build Instructions 
* Already built and put into launch files
 ## Nodes
 ### sensor_readout
* outputs an int64 value at 60 hz with the readings from the all the sensors of the pi through 6 topics
* file: sensor_readout.py
### Node name:
*sensor_readout
### Topics:
####  /rov/temperature:
*  Publishes temperature data from the pi at 60 hz through an int64
####  /rov/pressure:
*  Publishes pressure data from the pi at 60 hz through an int64
#### /rov/humidity:
*  Publishes pressure data from the pi at 60 hz through an int64
#### /rov/yaw: //IF YOU DON'T KNOW THESE LOOK THEM UP
*  Publishes data from the gyroscope about the ROV's yaw at 60 hz through an int64
#### /rov/pitch:
*  Publishes data from the gyroscope about the ROV's pitch at 60 hz through an int64
#### /rov/roll:
*  Publishes data from the gyroscope about the ROV's roll at 60 hz through an int64
#### /rov/Xacceleraton:
* Publishes data from the accelerometer in g force about the ROV's X acceleration through an int64 (need to change to float)
#### /rov/Yacceleration:
* Publishes data from the accelerometer in g force about the ROV's Y acceleration through an int64 (need to change to float)
#### /rov/Zacceleration:
* Publishes data from the accelerometer in g force about the ROV's Z acceleration through an int64 (need to change to float)

## Launch Information
 
It launches in bottomside.launch and automtically launches there
Added the node to automatically launch
 ## Troubleshooting
 ## Contributors 
 * Current maintaner: 
 Andrew Grindstaff *sigh*
 * Contributors:
 Andrew Grindstaff - everything with the sensors
Jaiveer - Found the problem with the "comment" that I ignored
 ## Helpful Resources
 https://projects.raspberrypi.org/en/projects/getting-started-with-the-sense-hat/9 NEED TO ADD
https://projects.raspberrypi.org/en/projects/sense-hat-data-logger/3
