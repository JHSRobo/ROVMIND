#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import Imu
import tf
import sys

rospy.init_node("getIMUVariance")
myargv = rospy.myargv(sys.argv)

if len(myargv) < 2:
    rospy.logerr("Usage Error: do `rosrun pkg_name file_name imu_topic_name")
    exit(1)

imuTopic = myargv[1]
rospy.loginfo("IMU topic: " + imuTopic)

rospy.loginfo("Please make sure that the IMU is reading as close to 0m/s^2 in the x and y and 9.8m/s^2 in the z as possible")
rospy.loginfo("Make sure that the IMU is level (orientation = 0, 0, 0, 1)")
rospy.loginfo("Keep the IMU level and with minimal acceleration for 30 seconds")

# helper function to turn vector3 x,y,z into an array
def toArray(vector3):
    return vector3.x, vector3.y, vector3.z #returning a tuple

#IMU Message
# Header header
#
# geometry_msgs/Quaternion orientation
# float64[9] orientation_covariance # Row major about x, y, z axes
#
# geometry_msgs/Vector3 angular_velocity
# float64[9] angular_velocity_covariance # Row major about x, y, z axes
#
# geometry_msgs/Vector3 linear_acceleration
# float64[9] linear_acceleration_covariance # Row major x, y z

numDataPoints = 0
dataPoints    = [] # store data points so that we can compute the variance once we know the mean
mean          = [0,0,0,0,0,0,0,0,0] # array of len 9
var           = [0,0,0,0,0,0,0,0,0] # variance (the average of the square of the difference from the mean)

def cb(data):
    global mean    # r,p,y ang_vel lin_accel
    global numDataPoints
    global dataPoints

    rpy = tf.transformations.euler_from_quaternion((data.orientation.x, data.orientation.y,
                                                    data.orientation.z, data.orientation.w))\

    dataPoints.append([0,0,0,0,0,0,0,0,0]) #increase number of data points stored by 1

    # First we need to calulate the mean (these should optimally be as close to 0 or 9.8 as possible)
    for i in range(3):
        mean[i] = (mean[i]+rpy[i])/2
        dataPoints[numDataPoints][i] = rpy[i]

    for i in range(3):
        mean[i+3] = (mean[i+3]+toArray(data.angular_velocity)[i])/2
        dataPoints[numDataPoints][i+3] = toArray(data.angular_velocity)[i]

    for i in range(3):
        mean[i+6] = (mean[i+6]+toArray(data.linear_acceleration)[i])/2
        dataPoints[numDataPoints][i+6] = toArray(data.linear_acceleration)[i]

    numDataPoints += 1

rospy.wait_for_message(imuTopic, Imu) #updates the ros time

start = rospy.Time.now().secs

#collect IMU data points (this line prevents the program from closing while data is colected and averaged)
while 30 > rospy.Time.now().secs - start and not rospy.is_shutdown():
    cb(rospy.wait_for_message(imuTopic, Imu))

rospy.loginfo("Data collected: " + str(mean))
rospy.loginfo("Number of points collected = " + str(numDataPoints))
rospy.loginfo("Calculating variance... ")

#Compute the IMU variance
#Sum the squares of the deviation from the mean
for data in dataPoints:
    for i in range(9):
        var[i] += (data[i]-mean[i])**2 #should we also put this against "ground truth"
#divide the sum by the number of data points
for i in range(9):
    var[i] /= numDataPoints

rospy.loginfo("IMU variance r,p,y,rv,pv,yv,x,y,z: " + str(var))

###############################################################################
#running against ground truth
for data in dataPoints:
    for i in range(9):
        if i == 8:
            var[i] += (data[i]-9.80665)**2
        else:
            var[i] += (data[i])**2
#divide the sum by the number of data points
for i in range(9):
    var[i] /= numDataPoints

rospy.loginfo("IMU variance against ground truth r,p,y,rv,pv,yv,x,y,z: " + str(var))
###############################################################################

rospy.loginfo("Program Complete!")
