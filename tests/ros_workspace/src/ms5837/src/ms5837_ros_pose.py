#!/usr/bin/env python

import rospy
from ms5837.msg import ms5837_data
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped


pub = rospy.Publisher('ms5837_pose', PoseWithCovarianceStamped, queue_size=3)
global zero_value
zero_value = 0
ros_msg = PoseWithCovarianceStamped()
header = Header()
header.frame_id = 'ms5837_pose_data'
header.stamp = rospy.Time.now()
ros_msg.header = header
ros_msg.pose.pose.position.z = 0
ros_msg.pose.covariance[8] = 0.0001349092722


def callback(msg):
	header.frame_id = 'ms5837_pose_data'
	header.stamp = rospy.Time.now()
	ros_msg.header = header
	ros_msg.pose.pose.position.z = msg.depth - zero_value
	pub.publish(ros_msg)


def zeroing():
	# zeroing the message
	zero_value += ros_msg.pose.pose.position.z


def publisher():
	# set up ros stuff
	while not rospy.is_shutdown():
		rospy.init_node('ms5837_pose')
		rospy.Service('zero_depth_sensor', None, zeroing())
		rospy.Subscriber('rov/ms5837', ms5837_data.msg, callback)
		rospy.spin()


if __name__ == '__main__':
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
