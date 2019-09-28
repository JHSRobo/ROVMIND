#!/usr/bin/env python

## @author Michael Equi
# @version 0.1
# @date 4-1-2019
# @mainpage The simulation_interface node
# @section intro_sec Introduction
# This code contains implementations for interfacing our vector drive code with the uuv simulator thruster manager using a basic model,
# multiplexing simulated cameras, and publishing mock sensor data (humidity/temperature/depth MS5837)

import rospy
from math import sqrt
import random #for noise and fake sensor error

#for interfacing vector drive with uuv sim thruster manager
from vector_drive.msg import thrusterPercents
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

#for simulating camera
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image

from ms5837.msg import ms5837_data #for simulating depth sensor

from sensor_msgs.msg import Temperature, RelativeHumidity, FluidPressure #for simulating sensor hat

thrusterVals = [0,0,0,0,0,0] ##array for holding commands (in prop angular velocity) for individual thrusters T1, T2, ... Tn
numberOfThrusters = 6

# Start the ros node
rospy.init_node("simulation_interface")
rospy.loginfo("Simulation interface starting... ")

#advertise muxed camera topic
muxedImagePub = rospy.Publisher("rov/camera/image_raw", Image, queue_size=1)
#advertise depth sensor topic
depthSensor = rospy.Publisher("rov/ms5837", ms5837_data, queue_size=1)

##Maps a variable based an input and an output range
def map(input, inMin, inMax, outMin, outMax):
    return(((input-inMin)/(inMax-inMin))*(outMax-outMin)+outMin)

##Converts the thruster percent to an rotor angular velocity that can be sent to the basic uuv thruster controller
# The function assumes a rotor constant of 0.0031 which is the default for the RexROV
# @param inputVal The input thruster percent*10 from -1000 to 1000
# @return A propellers angular velocity that can be sent to the uuv simulator thruster manager
def toRotorAngVel(inputVal):
    thrusterCmd = map(inputVal, -1000.0, 1000.0, 1100.0, 1900.0) #Convert thruster percent to microseconds signal
    if thrusterCmd >= 1475 and thrusterCmd <= 1525: #if the microseconds pulse is in the deadzone
        return 0
    # two quintic regressions - one forward, one reverse
    # https://www.desmos.com/calculator/mdhep9gttj
    elif thrusterCmd < 1500: #reverse thrust
        thrusterForce = (1.0659232851E-15*(thrusterCmd**5)-6.0041876632E-11*(thrusterCmd**4)+2.8763892765E-7*(thrusterCmd**3)-0.000541405587613*(thrusterCmd**2)+0.466418689099*thrusterCmd-156.361912754) #Thruster Force in Newtons
        #force is calculated in the uuv simulator with the equation rotorConstant*angularVel^2
        propAngVel = sqrt(thrusterForce/0.00031*(thrusterForce/abs(thrusterForce)))*(thrusterForce/abs(thrusterForce)) #We know that thrusterForce is not going to be zero here
        return propAngVel
    elif thrusterCmd > 1500: #forward thrust
        thrusterForce = (-1.235121669E-12*(thrusterCmd**5)+1.0528533279E-8*(thrusterCmd**4)-0.0000358699420294*(thrusterCmd**3)+0.0610602730083*(thrusterCmd**2)-51.9329220769*thrusterCmd+17653.3591143)
        #force is calculated in the uuv simulator with the equation rotorConstant*angularVel^2
        propAngVel = sqrt(thrusterForce/0.00031*(thrusterForce/abs(thrusterForce)))*(thrusterForce/abs(thrusterForce)) #We know that thrusterForce is not going to be zero here
        return propAngVel


#change the sign of the value in oder to compensate for the negative rotor constant
#rotor constant = -0.00031
def updateVert(data):
    global thrusterVals
    thrusterVals[0] = -toRotorAngVel(data.t1)
    thrusterVals[1] = -toRotorAngVel(data.t2)
    thrusterVals[2] = -toRotorAngVel(data.t3)
    thrusterVals[3] = -toRotorAngVel(data.t4)

def updateHoriz(data):
    global thrusterVals
    thrusterVals[4] = -toRotorAngVel(data.t1)
    thrusterVals[5] = -toRotorAngVel(data.t2)

#update the status of what camera is selected by the mux
selectedCamera = 1
def updateCameraMux(data):
    global selectedCamera
    selectedCamera = data.data
    #if selectedCamera is not a valid camera 1-4
    if not (selectedCamera > 0 and selectedCamera < 5):
        selectedCamera = 1

def updateVideoStreams(image):
    global selectedCamera, muxedImagePub
    #publish selected image
    if image.header.frame_id == "rov/camera" +  str(selectedCamera) + "_link":
        muxedImagePub.publish(image)

#update depth sensor information
def updateDepth(pressure):
    global depthSensor
    #pressure recieved in kpa
    #ms5837 data type
        # Header header
        # float64 tempC
        # float64 tempF
        # float64 depth
        # float64 altitudeM
    #https://www.grc.nasa.gov/www/k-12/WindTunnel/Activities/fluid_pressure.html
    #P=r*g*h (r=rho/density of fluid, g=gravity, h=height of liquid)
    #density = 100kg/m^3, g=9.8, fluid_pressure must be converted to pascal (*1000)
    #(pressure.fluid_pressure*1000)/(1000*9.8) -> 1000 cancels out
    depth = pressure.fluid_pressure/9.8
    temp = 27.6 + (2*random.random()-1) #degrees C +- 1
    msg = ms5837_data()
    msg.depth = depth
    msg.tempF = temp*9/5+32
    msg.tempC = temp
    msg.altitudeM = -depth
    depthSensor.publish(msg)

verticals  = rospy.Subscriber("rov/cmd_horizontal_vdrive", thrusterPercents, updateVert)
horzontals = rospy.Subscriber("rov/cmd_vertical_vdrive", thrusterPercents, updateHoriz)

#Simulated camera multiplexer
cameraMux = rospy.Subscriber("/rov/camera_select", UInt8, updateCameraMux)

#Subscribe and proccess all video streams
videoStream1 = rospy.Subscriber("rov/camera1/image_raw", Image, updateVideoStreams)
videoStream2 = rospy.Subscriber("rov/camera2/image_raw", Image, updateVideoStreams)
videoStream3 = rospy.Subscriber("rov/camera3/image_raw", Image, updateVideoStreams)
videoStream4 = rospy.Subscriber("rov/camera4/image_raw", Image, updateVideoStreams)

#Subscribe to external pressure sensor in order to get depth
extPressure = rospy.Subscriber("rov/pressure", FluidPressure, updateDepth)

#internal sensor hat publishers
sensorTempPub  = rospy.Publisher("rov/int_temperature", Temperature, queue_size=1)
sensorHumidPub = rospy.Publisher("rov/int_humidity", RelativeHumidity, queue_size=1)
sensorPresPub  = rospy.Publisher("rov/int_pressure", FluidPressure, queue_size=1)

#create a list of thruster publishers
thrusterPubs = []
for i in range(numberOfThrusters):
    thrusterName = "rov/thrusters/" + str(i+1) + "/input"
    thrusterPubs.append(rospy.Publisher(thrusterName, FloatStamped, queue_size=1))

rospy.loginfo("Created thruster publishers:  " + str(thrusterPubs))

rate = rospy.Rate(50) #50Hz update frequency
while not rospy.is_shutdown():
    #Update the thrusters by publishing the thrusterVals
    for i in range(numberOfThrusters):
        msg = FloatStamped()
        #Add noise if the thruster value is not zero
        if not thrusterVals[i] == 0:
            #thrusterVals[i] += 2*random.random()-1 #random float between -1 to 1 exclusive
            msg.data = thrusterVals[i]
        else:
            msg.data = thrusterVals[i]
        thrusterPubs[i].publish(msg)

    rospy.logdebug("Thrusters values published: " + str(thrusterVals))

    #publish sensor hat data
    tempMsg  = Temperature()
    #functions to give realistic sensor data curves over time
    timeNowMin = rospy.get_time()/60 ##Current ROS time in minutes
    tempMsg.temperature  = ((60*timeNowMin+30)/(timeNowMin+1)-2) + (0.5*random.random()-0.25)
    humidMsg = RelativeHumidity()
    humidMsg.relative_humidity = ((60*timeNowMin+90)/(timeNowMin+1)-25) + (0.5*random.random()-0.25)
    presMsg  = FluidPressure()
    presMsg.fluid_pressure  = (3000*timeNowMin/(timeNowMin+1)+101325) + (6*random.random()-3)

    sensorTempPub.publish(tempMsg)
    sensorHumidPub.publish(humidMsg)
    sensorPresPub.publish(presMsg)

    rate.sleep()
