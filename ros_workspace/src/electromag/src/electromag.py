#!/usr/bin/env python

import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Int32 


def toggleElectromag(pin):
  print("Callback")
  pin = pin.data
  if pin in pins:
    GPIO.output(pin, int(GPIO.input(pin) == 0))
    print("Changed pin {}".format(pin))
  else:
    print("Changed pin {} and added to pins list".format(pin))
    GPIO.setup(pin, GPIO.out)
    pins.append(pin)
    GPIO.output(pin, 1)
    
    

if __name__ == "__main__":
  GPIO.setmode(GPIO.BOARD)
  print("Setting up")
  pins = []
  rospy.init_node("electromag", anonymous=True)
  rospy.Subscriber("electromag", Int32, toggleElectromag)

  rospy.spin()
  GPIO.cleanup()
