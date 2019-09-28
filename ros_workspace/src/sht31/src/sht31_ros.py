#!/usr/bin/env python

from sht31_driver import SHT31

import rospy
from sht31.msg import sht31_data
from std_msgs.msg import Header

sensor = SHT31(0x44)


def publisher():
	pub = rospy.Publisher('rov/sht31', sht31_data, queue_size=3)
	rospy.init_node('sht31')
	rate = rospy.Rate(3) #3Hz data read
	
	while not rospy.is_shutdown():
		msg = sht31_data()
		header = Header()

		sensor.updateValues()

		msg.tempC = sensor.getTempuratureC()
		msg.tempF = sensor.getTempuratureF()
		msg.humidity = sensor.getHumidity()

		#update message headers
		header.stamp = rospy.Time.now()
		header.frame_id = 'humidity_data'
		msg.header = header

		pub.publish(msg)

		rate.sleep()

if __name__ == '__main__':
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
