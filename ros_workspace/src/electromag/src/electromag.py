import RPi.GPIO as GPIO
import rospy


def toggleElectromag(pin):
  if pin in pins:
    GPIO.output(pin, int(GPIO.input(pin) == 0))
  else:
    GPIO.setup(pin, GPIO.out)
    pins.append(pin)
    GPIO.output(pin, 1)
    
    

if __name__ == "__main__":
  GPIO.setmode(GPIO.BOARD)
  pins = []

  rospy.init_node("electromag")

  rospy.Subscriber("electromag", pin, toggleElectromag)

  GPIO.cleanup()
