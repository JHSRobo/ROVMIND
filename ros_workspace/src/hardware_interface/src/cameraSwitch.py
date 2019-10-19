#!/usr/bin/env python
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import UInt8

# GPIO pins that are used, A0 and A1
pin1 = 35
pin2 = 36

# Callback function that runs every time a new message is
def callback(data):
    if data.data == 2:
        GPIO.output(pin1, GPIO.LOW)
        GPIO.output(pin2, GPIO.HIGH)
    elif data.data == 3:
        GPIO.output(pin1, GPIO.HIGH)
        GPIO.output(pin2, GPIO.LOW)
    elif data.data == 4:
        GPIO.output(pin1, GPIO.HIGH)
        GPIO.output(pin2, GPIO.HIGH)
    else:
        GPIO.output(pin1, GPIO.LOW)
        GPIO.output(pin2, GPIO.LOW)

def hook():
    GPIO.cleanup()
    rospy.loginfo("Camera node shutdown successfully")

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('drive_control')

    #setup GPIO that will be used for the camera Mux
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(pin1, GPIO.OUT)
    GPIO.setup(pin2, GPIO.OUT)

    rospy.Subscriber("camera_select", UInt8, callback)
    #shutdown hook releases the GPIO when the node is killed
    rospy.on_shutdown(hook)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
