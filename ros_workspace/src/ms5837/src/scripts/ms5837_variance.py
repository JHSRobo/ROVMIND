#!/usr/bin/env python

import rospy
from ms5837.msg import ms5837_data

data = []
time = 0


def calculate(array):
    mean = 0
    total = 0
    for point in array:
        mean += point
    mean /= len(array)
    for point in array:
        total += (point - mean) * (point - mean)
    total /= len(array)
    print(total)


def callback(msg):
    data.append(msg.depth)
    time += 1
    print(time)
    if time >= 2000:
        calculate(data)


def main():
    rospy.init_node('test_node')
    while not rospy.is_shutdown():
        rospy.Subscriber('rov/ms5837', ms5837_data, callback)
        rospy.spin()


if __name__ == '__main__':
    main()
