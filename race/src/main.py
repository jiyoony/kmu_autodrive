#!/usr/bin/env python
import cv2, math
import numpy as np
import roslib
import sys
import sensor_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
from xycar_motor.msg import xycar_motor
from sensor_msgs.msg import Image
from std_msgs.msg import String

import os
from obstacle_detector.msg import Obstacles
import rospy, rospkg
import genpy.message
import sensor_msgs.msg
from rosservice import ROSServiceException
from slidewindow import SlideWindow
from warper import Warper

cv_image = np.array([])
obstacles = None
ack_publisher = None
car_run_speed = 0.5
pub = None
slidewindow = SlideWindow()
warper = Warper()
bridge = CvBridge()

def img_callback(data):
	global cv_image
	try:
		cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
		print(e)

def obstacle_callback(data):
	global obstacles
	obstacles = data


def process_img(img):
	#cv2.imshow('a',img)
	gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	kernel_size = 5
	blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)
	low_threshold = 60#60
	high_threshold = 70# 70
	edges_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)
	warped = warper.warp(edges_img)
	ret, x_left,x_right, w_slide_img = slidewindow.w_slidewindow(warped)
	h_slide_img, steer = slidewindow.h_slidewindow(warped, x_left,x_right)
	return h_slide_img,0


def auto_drive(pid):
	global pub
	global car_run_speed
	msg = xycar_motor()
	msg.angle = pid
	msg.speed = car_run_speed
	pub.publish(msg)

def main():
	global cv_image
	global pub
	global cap
	global w,h
	rospy.sleep(3)
	bridge = CvBridge()
	
	pub = rospy.Publisher('/xycar_motor', xycar_motor,queue_size = 1)
	image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,img_callback)
	obstacle_sub = rospy.Subscriber("/obstacles", Obstacles, obstacle_callback, queue_size = 1)
	rospy.init_node('autodrive',anonymous=True)
	while not rospy.is_shutdown():
		print(cv_image.shape)
		img1, x_location = process_img(cv_image) 
		cv2.imshow('a',img1)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break


if __name__ == '__main__':
	main()

