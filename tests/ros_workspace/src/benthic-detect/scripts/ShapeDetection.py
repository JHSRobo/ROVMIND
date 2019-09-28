#!/usr/bin/env/
# USAGE
#needs ros support but no one wants to do it.


# import the necessary packages
from edgedetect.shapedetector import ShapeDetector
import cv_bridge
import argparse
import imutils
import cv2
import numpy as np
def processing(x):
	# load the image/video
	cap = x
	while True:
		# Capture frame-by-frame
		ret, frame = cap.read()
		cv2.rectangle(frame, (110, 400), (510, 100), (255, 255, 255), 3)
		cv2.imshow('frame', frame)
		k = cv2.waitKey(33)
		if k==97:
			mode = 1
			break
		elif k==115:
			mode = 2
			break
		elif k==255:
			continue
	# crop image to fit frame and resize it for better processing!
	y = 110
	x = 120
	h = 285
	w = 380
	crop = frame[y:y+h, x:x+w]
	resized = imutils.resize(crop, width=520)
	ratio = crop.shape[0] / float(resized.shape[0])

	# convert the resized image to grayscale, blur it slightly,
	# and threshold it
	gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
	blurred = cv2.GaussianBlur(gray, (5, 5), 0)
	##generic thresholhing
	if mode == 2:
		ret, thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY_INV)
	##adaptive threshholding
	if mode == 1:
		thresh = cv2.adaptiveThreshold(blurred ,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
	            	cv2.THRESH_BINARY_INV,11,6)
	#ret,thresh = cv2.threshold(blurred,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
	#thresh = cv2.adaptiveThreshold(blurred,255,cv2.ADAPTIVE_THRESH_MEAN_C,\
	            #cv2.THRESH_BINARY_INV,11,6)
	cv2.imshow("thresh", thresh)
	# find contours in the thresholded image and initialize the
	# shape detector
	cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	cnts = cnts[0] if imutils.is_cv2() else cnts[1]
	sd = ShapeDetector()

	# loop over the contours
	for c in cnts:

		# compute the center of the contour, then detect the name of the
		# shape using only the contour
		M = cv2.moments(c)
		cX = int((M["m10"] / (M["m00"] + 1e-7)) * ratio)
		cY = int((M["m01"] / (M["m00"] + 1e-7)) * ratio)
		shape = sd.detect(c)
		if shape == "triangle":
			s2 += 1
		if shape == "square":
			s3 += 1
		if shape == "rectangle":
			s4 += 1
		if shape == "circle":
			s5 += 1
			#circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=0,maxRadius=0)
			#if circles is not None:
				#s5 += 1

		# multiply the contour (x, y)-coordinates by the resize ratio,
		# then draw the contours and the name of the shape on the image
		c = c.astype("float")
		c *= ratio
		c = c.astype("int")
		cv2.drawContours(crop, [c], -1, (0, 255, 0), 2)
		cv2.putText(crop, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 2)
		#converts ints to strings
		c2 = str(s2)
		c3 = str(s3)
		c4 = str(s4)
		c5 = str(s5)
		#creates blackdrop for GUI
		blackdrop = np.zeros((512, 512, 3), np.uint8)
		font = cv2.FONT_HERSHEY_SIMPLEX
		#displays number of triangles
		cv2.putText(blackdrop,c2,(10,120), font, 4,(255,255,255),2,cv2.LINE_AA)
		#displays number of Squares
		cv2.putText(blackdrop,c3,(10,230), font, 4,(255,255,255),2,cv2.LINE_AA)
		p1 = (166, 40)
		p2 = (130, 120)
		p3 = (200, 120)
		# Drawing the triangle with the help of lines
		cv2.line(blackdrop, p1, p2, (0, 0, 255), 3)
		cv2.line(blackdrop, p2, p3, (0, 0, 255), 3)
		cv2.line(blackdrop, p1, p3, (0, 0, 255), 3)
		cv2.rectangle(blackdrop,(125,155),(195,225),(0,0,255),-1)
		cv2.putText(blackdrop,c4,(10,340), font, 4,(255,255,255),2,cv2.LINE_AA)
		cv2.rectangle(blackdrop,(155,265),(165,335),(0,0,255),-1)
		cv2.putText(blackdrop,c5,(10,450), font, 4,(255,255,255),2,cv2.LINE_AA)
		cv2.circle(blackdrop,(160,410), 40, (0,0,255), -1)
		cv2.imshow("blackdrop", blackdrop)

		# show the output image

	cv2.imshow("Image", crop)

	cv2.waitKey(0)

s2 = s3 = s4 = s5 = 0
class rosShit(object):
	def __init__(self):
		self.bridge_object = CvBridge()
		self.image_sub = rospy.Subscriber("/rov/image_raw", Image, self.camera_callback)

	def camera_callback(self,data):
		try:
			cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
			processing(cv_image)
		except CvBridgeError as e:
           print(e)
def main():
    benthic-detect = rosShit()
    rospy.init_node('that_benthic_node')
    rospy.spin()
if __name__ == '__main__':
    main()
