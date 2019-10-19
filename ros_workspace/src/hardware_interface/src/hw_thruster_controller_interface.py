#!/usr/bin/env python
import rospy
import serial
from vector_drive.msg import thrusterPercents

thrusters = [0, 0, 0, 0, 0, 0, 0, 0]


def horizontals(data):
    thrusters[0] = data.t1
    thrusters[1] = data.t2
    thrusters[2] = data.t3
    thrusters[3] = data.t4


def verticals(data):
    thrusters[4] = data.t1
    thrusters[5] = data.t2
    thrusters[6] = data.t3
    thrusters[7] = data.t4


def listener():
    rospy.init_node('hw_thruster_controller_interface')

    port = rospy.get_param('~thrusterControllerPort', '/dev/ttyUSB0')

    rospy.Subscriber("rov/cmd_horizontal_vdrive", thrusterPercents, horizontals)
    rospy.Subscriber("rov/cmd_vertical_vdrive", thrusterPercents, verticals)

    r = rospy.Rate(30)  # 30hz

    ser = serial.Serial(port, 115200, timeout=0)

    while not rospy.is_shutdown():
        packet = ' '.join(str(t) for t in thrusters) + '\n'  # concatenate list into proper string
        rospy.logdebug(packet)

        incommingMessage = ser.readline()
        rospy.logdebug(incommingMessage)

        ser.write(packet.encode('UTF-8'))

        r.sleep()


if __name__ == '__main__':
    listener()
