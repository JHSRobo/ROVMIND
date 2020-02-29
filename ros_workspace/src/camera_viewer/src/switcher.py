#!/usr/bin/env python
# Switcher.py
# By Andrew Grindstaff
# ROS Package to switch between cameras streaming to port 5000
# Streamer code in streamer directory at streamer.py

import cv2
import json
import time
import datetime
from cv_bridge import CvBridge
import numpy as np
import threading
import flask
import logging
import rospy
from std_msgs.msg import UInt8

# NEED TO ADD SENSOR DATA
# Overlay and timer stack
# Task specific visuals - overlay


class SwitchCameras:
    """The class for handling all the camera logic. Switches and reads the camera, adding an overlay to it"""
    def __init__(self):
        try:
            self.configed = json.load(open("config.json"))
        except IOError:
            print "Please make config.json if you want to save camera settings"
            self.configed = {}

        self.bridge = CvBridge()
        self.verified, self.failed = {}, {}
        self.num = self.cap = None
        self.change = False

    def read(self):
        """Reads a frame from the cv2 video capture and adds the overlay to it"""
        if self.change:
            self.cap.release()
            self.cap = cv2.VideoCapture('http://{}:5000'.format(self.num))
            self.change = False
        ret, frame = self.cap.read()
        if frame is None:
            self.change = True
        if ret is None or frame is None:
            return False
        cv2.putText(frame, self.num, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1,(255, 255, 255), 2, cv2.LINE_AA)
        return frame

    def screenshot(self):
        """Returns a screenshot"""
        frame = self.read()
        counter = 0
        while not frame:
            counter += 1
            frame = self.read()
            if counter == 5:
                break
        else:
            return self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")

    def wait(self):
        """Waits for a camera IP to be put into verified and then assigns numbers"""
        while not self.verified:
            if rospy.is_shutdown():
                return
            time.sleep(1)

        self.num = self.verified.keys()[0]
        print "Loading capture"
        self.cap = cv2.VideoCapture('http://{}:5000'.format(self.num))

    def change_camera(self, camera_num):
        """rospy subscriber to change cameras"""
        try:
            num = [x for x in self.verified if self.verified[x]['num'] == camera_num.data][0]
            self.change = True
            self.num = num
        except IndexError:
            pass

    def find_cameras(self):
        """Waits for a request on port 5000"""

        log = logging.getLogger("werkzeug")
        log.setLevel(logging.ERROR)
        app = flask.Flask(__name__)

        @app.route('/')
        def page():
            if flask.request.remote_addr not in self.verified:
                self.verified[flask.request.remote_addr] = {}
                try:
                    self.give_nums()
                except IndexError:
                    print 'Camera detected, but all slots are filled'
            return "go away", 200

        print 'Web server online'
        app.run(host='0.0.0.0', port=12345)

    def give_nums(self):
        for x in self.configed:
            if x in self.verified:
                self.verified[x]['num'] = self.configed[x]['num']
                print 'Camera at {}, added under {}'.format(x, self.configed[x]['num'])
            else:
                self.failed[x] = self.verified[x]

        taken = [self.verified[x]['num'] for x in self.verified if 'num' in self.verified[x]]
        available = [x for x in range(1, 8) if x not in taken]
        for x in self.verified:
            if 'num' not in self.verified[x]:
                self.verified[x]['num'] = available.pop(0)
                print 'Camera at {}, added under {}'.format(x, self.verified[x]['num'])

    def which_camera(self):
        """rospy service - not being used"""
        def ip():
            return self.verified[self.num]
        rospy.init_node("camera_ip_server")
        s = rospy.Service('current_ip', camera_viewer.srv.current_ip, ip)
        rospy.spin()

    def camera_failed(self):
        print "Camera {} has failed, switching to a different camera".format(self.verified[self.num]['num'])
        try:
            self.failed[self.num] = self.verified[self.num]
            self.verified.pop(self.num, None)
            #self.change_camera(self.verified.keys()[0])
        except IndexError:
            print 'No cameras available, quitting'
            raise RuntimeError("No cameras available")


def show_all(cameras):
    """Don't use this - incomplete"""

    def blank_frame(shape):
        """Returns a blank frame for displaying an odd number of video streams"""
        return np.zeros(shape=shape.shape, dtype=np.uint8)

    cameras = list(cameras.values())
    frame = []
    for camera in range(0, len(cameras) - 1, 1):
        ret, frame1 = cv2.VideoCapture('http://{}:5000'.format(cameras[camera])).read()
        if not ret:
            camera -= 1
            continue
        try:
            ret, frame2 = cv2.VideoCapture('http://{}:5000'.format(cameras[camera + 1])).read()
        except IndexError:
            frame2 = blank_frame(frame1)
        else:
            frame2 = blank_frame(frame1) if not ret else frame2
        frame.append(cv2.hconcat([frame1, frame2]))

    for camera in range(1, len(frame)):
        frame[0] = cv2.vconcat([frame[0], frame[camera]])

    cv2.putText(frame[0], 'All', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
    return frame[0]


def main():
    switcher = SwitchCameras()

    # ROS Setup
    rospy.init_node('pilot_page')
    rospy.Subscriber('/rov/camera_select', UInt8, switcher.change_camera)

    camera_thread = threading.Thread(target=switcher.find_cameras)
    camera_thread.setDaemon(True)
    camera_thread.start()

    #service_thread = threading.Thread(target=switcher.which_camera)
    #service_thread.start()

    screenshot_thread = threading.Thread(target=switcher.screenshot)
    screenshot_thread.start()

    print 'Waiting for cameras'
    switcher.wait()

    cv2.namedWindow("Camera Feed", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("Camera Feed", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    while not rospy.is_shutdown():
        frame = switcher.read()
        if frame is not False:
            cv2.imshow('Camera Feed', frame)
        cv2.waitKey(1)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
