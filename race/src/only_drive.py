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
#from slidewindow import SlideWindow
from warper import Warper
from pid import PidCal
#import camtest
import signal
from slide_ver2 import SlideWindow

cv_image = np.array([])
obstacles = None
ack_publisher = None
car_run_speed = 5
ar_tag_Cnt = 0
pub = None
old_time = 0
slidewindow = SlideWindow()
warper = Warper()
bridge = CvBridge()
pid = PidCal()
inv_steer = 0
old_steer = 0



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
	global car_run_speed, old_steer
	steer = 0
	gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	kernel_size = 5
	blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)
	low_threshold = 70#60
	high_threshold = 120# 70
	kernel = np.ones((3, 3),np.uint8)
	edges_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)
	warped = warper.warp(edges_img)
	ret, thr = cv2.threshold(warped, 140,255,cv2.THRESH_BINARY)
	cv2.imshow('warped',thr)
	#ret, x_left,x_right, w_slide_img, flag = slidewindow.w_slidewindow(warped)
	#h_slide_img, steer, centerY= slidewindow.h_slidewindow(warped, x_left, x_right, flag)
	out_img, steer,line_flag = slidewindow.slidewindow(thr)
	
	#if abs(old_steer - steer) > 0.2:
	#	steer = old_steer
	centerY = 0
	#steer*=1.75
	x_left = 0
	x_right = 0
	if line_flag ==3:
		steer = old_steer
	old_steer = steer
	#cv2.imshow('w_slide',w_slide_img)
	return out_img, steer
def auto_drive(pid):
	global pub
	global car_run_speed

	msg = xycar_motor()
	msg.angle = pid
	#msg.angle = 0.34
	msg.speed = car_run_speed
	#msg.speed = 0
	pub.publish(msg)

#Find Stop Line
def findStop(roi, img):#plus : mark_Cnt
	global car_run_speed
	global old_time
	now_time = 0
	stopFlag = 0
	nonzero = roi.nonzero()

	lower_yellow = np.array([-20, 80, 80])
	upper_yellow = np.array([50, 255, 255])

	img = img[240:, :]
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
	res = cv2.bitwise_and(img, img, mask = mask_yellow)
	#cv2.imshow('res',res)
	resnon = res.nonzero()
	resx = np.array(resnon[1])
	goodres = ((resx > 200) &(resx < 440)).nonzero()[0]

	if len(nonzero[0] > 1100):
		stopFlag = 1
		if (len(goodres)):
			stopFlag = 2

	if stopFlag == 1:
		if car_run_speed - car_run_speed * 0.5 > 0: 
			if car_run_speed < 0.0005: car_run_speed = 0
			else: car_run_speed -= car_run_speed * 0.5
		else:
			if (old_time == 0): old_time = time.time()
			now_time = time.time()
			if (now_time - old_time > 3): 
				car_run_speed = 6
				old_time = 0

	#if stopFlag == 2:
		#if (mark_Cnt > 15): car_run_spped /= 0.5
	return stopFlag


def main():
	global cv_image
	global pub
	global cap
	global w,h
	global obstacles
	global inv_steer
	global car_run_speed
	global old_steer
	rospy.sleep(3)
	bridge = CvBridge()
	x_location = 0
	steer = 0
	old_steer = 0
	old_mode = 0
	curve_cnt = 0
	angle_max = 0
	stop = None
	pub = rospy.Publisher('/xycar_motor', xycar_motor, queue_size=1)
	image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,img_callback)
	rospy.init_node('autodrive',anonymous=True)
	#auto_drive(steer)
	time.sleep(2)
	while not rospy.is_shutdown():
		if cv_image is not None:
			cv2.imshow('a', cv_image)	
			time.sleep(0.1)
			process_Img, steer = process_img(cv_image)
			warp_Img = warper.warp(cv_image)
			#roi = warp_Img[centerY - 20:centerY + 20, x_left + 30:x_right- 30]
			#stop = findStop(roi, warp_Img)		
			print('old_steer - steer : ',old_steer-steer)
			cv2.imshow('cv_image', process_Img)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
			steer = -pid.pid_control(steer)
			#steer = 0
			#steer = -pid.pid_control(lane_steer)
			#if abs(steer/old_steer)>10:
				#pid.p[0]=0.45
			print('speed : ',car_run_speed)				
			print("Steer :", steer)
			auto_drive(steer)
			old_steer = steer
	
	#cap.release()
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()

