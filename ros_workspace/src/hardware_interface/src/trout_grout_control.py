#!/usr/bin/env python
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Bool

# Pins 5 and 6 are used for motor control
# Counterclockwise motion is done with pin 5 high, pin 6 low
# Clockwise motion is done with both pins 5 and 6 high
# This current version is setup just for counterclockwise motion
troutGroutPin1 = 33
troutGroutPin2 = 32

# Callback function that runs every time a message is published on the topic

def callback(data):
    if data.data:
        GPIO.output(troutGroutPin1, GPIO.HIGH)
	GPIO.output(troutGroutPin2, GPIO.LOW)
    else:
        GPIO.output(troutGroutPin1, GPIO.LOW)
	GPIO.output(troutGroutPin2, GPIO.HIGH)

def hook():
    GPIO.cleanup()
    rospy.loginfo("Trout Grout  node shutdown successfully")

def listener():

    # Launches trout grout node
    rospy.init_node('trout_grout')

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(troutGroutPin1, GPIO.OUT)
    GPIO.setup(troutGroutPin2, GPIO.OUT)

    rospy.Subscriber("trout_grout", Bool, callback)

    #shutdown hook releases the GPIO when the node is killed
    rospy.on_shutdown(hook)
    rospy.spin()

if __name__=='__main__':
    listener()
