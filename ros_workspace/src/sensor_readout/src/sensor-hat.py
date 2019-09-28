#!/usr/bin/env python

import rospy

import time
from sense_hat import SenseHat
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Imu, Temperature, RelativeHumidity, FluidPressure

sense = SenseHat()
sense.set_imu_config(True, True, True) #compass, gyro, accele
calibration = sense.get_orientation_radians()
start_roll = calibration['yaw']
start_pitch = calibration['roll']
start_yaw = calibration['pitch']
start_time = int(time.time()) * 1000


def talker():

	temp_pub = rospy.Publisher('rov/int_temperature', Temperature, queue_size = 1) #Publisher for the different sensors: Temperature, Humidity, Pressure,
	pressure_pub = rospy.Publisher('rov/int_pressure', FluidPressure, queue_size = 1)
	humidity_pub = rospy.Publisher('rov/int_humidity', RelativeHumidity, queue_size = 1)

	imu_pub = rospy.Publisher("rov/imu", Imu, queue_size = 1) #Imu publisher

	rate = rospy.Rate(60)
	while not rospy.is_shutdown():
		global calibration, start_roll, start_pitch, start_yaw, start_time
		header = Header()
		header.stamp = rospy.Time.now()
		header.frame_id = 'sensor_data'

		temp_pub.publish(header, sense.get_temperature(), 0)#Actually pubish the things stated above
		pressure_pub.publish(header, sense.get_pressure(), 0)
		humidity_pub.publish(header, sense.get_humidity(), 0)#Temperature pressure humidity

		message = Imu() #make a new object of class IMU with name message
		message.header = header

		acceleration = sense.get_accelerometer_raw() #x y and z G force, not rounded
		message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = (acceleration['x'] * 9.80665, acceleration['z'] * 9.80665, acceleration['y'] * 9.80665) #9.80665 is gs to newtons; WITH 

		orientation = sense.get_orientation_radians() #roll pitch and yaw
		message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w = quaternion_from_euler(orientation['roll'], orientation['pitch'], orientation['yaw']) #converts the degrees returned by get_orientation() to radians then uses all 4 directions into a quaternion, then publishes the data
		elapsed_time = (int(time.time()) * 1000) - start_time
		if elapsed_time <= 0:
			elapsed_time = max(1, elapsed_time)

		angular_velocity_roll = (orientation['yaw'] - start_roll) / (elapsed_time * 1000)
		angular_velocity_pitch = (orientation['roll'] - start_pitch) / (elapsed_time * 1000)
		angular_velocity_yaw = (orientation['pitch'] - start_yaw) / (elapsed_time * 1000)

		message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = (angular_velocity_roll, angular_velocity_pitch, angular_velocity_yaw)

		imu_pub.publish(message)
		start_roll = orientation['yaw']
		start_pitch = orientation['roll']
		start_yaw = orientation['pitch']
		start_time = int(time.time()) * 1000

		rate.sleep()


if __name__ == '__main__':
	try:
		rospy.init_node("sensor_interface")
		talker()
	except rospy.ROSInterruptException:
		pass
