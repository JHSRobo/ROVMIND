#!/usr/bin/env python

import ms5837_driver

import rospy
from ms5837.msg import ms5837_data
from std_msgs.msg import Header

# Choose seawater or freshwater depth calibration using ros param
# freshwater = 997 kg/m^3
# seawater = 1029 kg/m^3
fluidDensity = rospy.get_param('fluidDensity', '997')


def publisher():
	# set up ros stuff
	pub = rospy.Publisher('rov/ms5837', ms5837_data, queue_size=3)
	rospy.init_node('ms5837')
	rate = rospy.Rate(30)  # 30Hz data read

	sensor = ms5837_driver.MS5837_30BA()  # Default I2C bus is 1 (Raspberry Pi 3)
	# sensor = ms5837.MS5837_02BA()

	# sensor.init must run immediatly after instatation of ms5837 object
	attempts = 0
	good_init = False
	while not good_init and attempts < 10:
		try:
			sensor.init()
			good_init = True
		except Exception as e:
			rospy.logerr("ms5837 could not be initialized! %s", e)
			attempts += 1
			rospy.sleep(0.25)

	if attempts == 10:
		rospy.logerr("Sensor could not be initialized! Program end")
		exit(1)

	sensor.setFluidDensity(int(fluidDensity))

	while not rospy.is_shutdown():
		msg = ms5837_data()
		header = Header()

		good_read = False
		while not good_read:
			try:	
				sensor.read()
				good_read = True
			except Exception as e:
				rospy.logdebug("Sensor read failed! %s", e)
				rospy.sleep(0.2)

		msg.tempC = sensor.temperature(ms5837_driver.UNITS_Centigrade)
		msg.tempF = sensor.temperature(ms5837_driver.UNITS_Farenheit)
		msg.depth = sensor.depth() 
		msg.altitudeM = sensor.altitude()

		# update message headers
		header.stamp = rospy.Time.now()
		header.frame_id = 'depth_data'
		msg.header = header

		pub.publish(msg)

		rate.sleep()


if __name__ == '__main__':
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
