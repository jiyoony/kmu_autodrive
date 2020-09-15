#!/usr/bin/env python
import cv2, math
import time
import numpy as np
import roslib
import sys
import sensor_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
from xycar_motor.msg import xycar_motor
from sensor_msgs.msg import Image
from std_msgs.msg import String
import os
import rospy, rospkg
import genpy.message
import sensor_msgs.msg
from rosservice import ROSServiceException
from slidewindow import SlideWindow
from warper import Warper
from pid import PidCal
import signal

cv_image = np.array([])
obstacles = None
ack_publisher = None
car_run_speed = 20
ar_tag_Cnt = 0
pub = None
slidewindow = SlideWindow()
warper = Warper()
bridge = CvBridge()
pid = PidCal()
inv_steer = 0



def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def img_callback(data):
	global cv_image
	try:
		tmp = bridge.imgmsg_to_cv2(data, "bgr8")
		cv_image = cv2.resize(tmp, (640,480))
	except CvBridgeError as e:
		print(e)

def process_img(img):
	global car_run_speed
	steer = 0
	old_steer = 0
	#hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
	#cv2.imshow('bf',img)
	#hsv[:,:,2] -= 60
	#cv2.imshow('after',hsv)
	#img = cv2.cvtColoi(hsv,cv2.COLOR_HSV2BGR)
	#cv2.imshow('aft',img)
	gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	#cv2.imshow('gray',gray)
	kernel_size = 5
	blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)
	low_threshold = 60#60
	high_threshold = 70# 70
	kernel = np.ones((3, 3),np.uint8)
	edges_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)
	#cv2.imshow('edges',edges_img)
	warped = warper.warp(edges_img)
	ret, x_left,x_right, w_slide_img, flag = slidewindow.w_slidewindow(warped)
	h_slide_img, steer, cneterY= slidewindow.h_slidewindow(warped, x_left, x_right, flag)
		#if abs(steer - old_steer > 10): car_run_speed /= 2

	old_steer = steer
	return h_slide_img, steer
def auto_drive(pid):
	global pub
	global car_run_speed

	msg = xycar_motor()
	msg.angle = pid
	msg.speed = 3
	#msg.speed = car_run_speed
	pub.publish(msg)



def main():
	global cv_image
	global pub
	global cap
	global w,h
	global obstacles
	global inv_steer
	rospy.sleep(3)
	bridge = CvBridge()
	x_location = 0
	steer = 0
	old_steer = 0
	old_mode = 0
	curve_cnt = 0
	angle_max = 0
	stop = None
	pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
	image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,img_callback)
	rospy.init_node('autodrive',anonymous=True)
	#cap = cv2.VideoCapture('/home/nvidia/xycar_ws/src/race/src/input_video/org2.avi')
	#cap.set(3,1280)
	#cap.set(4,720)
	auto_drive(steer)
	while not rospy.is_shutdown():
	
		#steer = pid.pid_control(steer)
		#auto_drive(steer) 
		#ret,cv_image = cap.read()
		#hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		#hsv[2] -= 30
		#cv_image = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)	
		time.sleep(0.1)
		process_Img, lane_steer = process_img(cv_image)
		cv2.imshow('cv_image', process_Img)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
		print('-------------------')
		steer = pid.pid_control(lane_steer)
		steer = np.rad2deg(steer)						
		print("Steer :", steer)
		auto_drive(steer)
	
	cap.release()
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()

