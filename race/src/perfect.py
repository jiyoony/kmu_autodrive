#!/usr/bin/env python
import cv2, math
import time
import numpy as np
import roslib
import sys
import sensor_msgs.msg
from obstacle_detector.msg import Obstacles
from cv_bridge import CvBridge, CvBridgeError
from xycar_motor.msg import xycar_motor
from sensor_msgs.msg import Image
from std_msgs.msg import String
import os
import rospy, rospkg
import genpy.message
import sensor_msgs.msg
from rosservice import ROSServiceException
from warper import Warper
from pid import PidCal
from curveDetector import CurveDetector
from sub_edit_slide_ver2 import SlideWindow

cv_image = None
obstacles = None
ack_publisher = None
car_run_speed =10
pub = None
old_time = 0
slidewindow = SlideWindow()
warper = Warper()
bridge = CvBridge()
pid = PidCal()
curve = CurveDetector()


def img_callback(data):
	global cv_image
	try:
		tmp = bridge.imgmsg_to_cv2(data, "bgr8")
		cv_image = cv2.resize(tmp,(640,480))
	except CvBridgeError as e:
		print(e)

def process_img(img):
	cols,rows,ch = img.shape
	brightness =np.sum(img)/(255*cols*rows)
	minimum_brightness = 0.5
	ratio = brightness/minimum_brightness
	bright_img = cv2.convertScaleAbs(img, alpha=1/ratio,beta =0)
	x_location = 0
	gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	kernel_size = 5
	blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)
	low_threshold = 45#60
	high_threshold = 55# 70
	kernel = np.ones((3, 3),np.uint8)
	edges_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)
	warped = warper.warp(edges_img)
	h_slide_img,x_location= slidewindow.slidewindow(warped)
	return h_slide_img, x_location#, centerY, x_left, x_right


def obstacle_callback(data):	
	global obstacles
	obstacles = data
def auto_drive(steer,speed):
	global pub
	#motor global??
	msg = xycar_motor()
	msg.angle = steer
	#msg.angle = 0.34
	msg.speed = speed
	#msg.speed = 0
	pub.publish(msg)

#Find Stop Line
def findStop(roi, img):#plus : mark_Cnt
	global car_run_speed
	global old_time
	#Using HSV
	stopFlag = 0
	nonzero = roi.nonzero()

	lower_yellow = np.array([-20, 80, 80])
	upper_yellow = np.array([50, 255, 255])

	img = img[240:, :]
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
	res = cv2.bitwise_and(img, img, mask = mask_yellow)
	#cv2.imshow('res',res)
	cv2.imshow('roi', roi)
	resnon = res.nonzero()
	resx = np.array(resnon[1])
	goodres = ((resx > 200) &(resx < 440)).nonzero()[0]
	print("NONZERO : ", len(nonzero[0]))

	if len(nonzero[0]) > 800:
		stopFlag = 1
		if (len(goodres)):
			stopFlag = 2

	if stopFlag == 1:
		if car_run_speed - car_run_speed * 0.5 > 0: 
			if car_run_speed < 0.0005: car_run_speed = 0
			else: 
				car_run_speed = 0 #car_run_speed -= car_run_speed * 0.5
				time.sleep(0.2)
		else:
			print("Stop!")
			time.sleep(4)
			car_run_speed = 6
	#if stopFlag == 2:
		#if (mark_Cnt > 15): car_run_spped /= 0.5

	'''
#Not Using HSV
	self.stopCnt = 0
	nonzero = roi.nonzero()

	cv2.imshow('roi', roi)
	goodres = ((resx > 200) &(resx < 440)).nonzero()[0]
	print("NONZERO : ", len(nonzero[0]))

	if len(nonzero[0]) > 800:
		if self.stopCnt % 3 == 2:
			if car_run_speed - car_run_speed * 0.5 > 0: 
				if car_run_speed < 0.0005: car_run_speed = 0
				else: 
					car_run_speed = 0
					time.sleep(0.2)
			else:
				print("Stop!")
				time.sleep(4)
				car_run_speed = 6
	'''

	return stopFlag


def main():
	global cv_image
	global pub
	#global cap
	#global obstacles
	global car_run_speed
	start_time=time.time()
	#rospy.sleep(3)
	x_location = 0
	steer = 0
	old_mode = 0
	curve_cnt = 0
	angle_max = 0
	stop = None
	rospy.init_node('autodrive',anonymous=True)
	pub = rospy.Publisher('/xycar_motor', xycar_motor, queue_size=1)
	image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,img_callback)
	obstacle_sub = rospy.Subscriber("/obstacles", Obstacles, obstacle_callback, queue_size = 1)
	while time.time()-start_time <3:
		auto_drive(0,0)
	speed =19
	while not rospy.is_shutdown():	
	
		if cv_image is not None:
			
			process_Img, x_location= process_img(cv_image)
			steer = pid.pid_control(x_location)
			curve.list_update(steer)
			curve.count_curve()
			print("STEER : ",steer)
			print("count_curve", curve.curve_count)
			
			#if curve.curve_count ==1:
			#	if speed < 8:
			#		speed +=0.2
			#if curve.curve_count == 2:
			#	speed = 6
				
			#print('curve`s sum',abs(sum(curve.pid_list)))
			#print("pid_steer : ", steer)
			#print(x_location)
			#warp_Img = warper.warp(cv_image)
			#roi = thr[centerY - 60:centerY + 40, x_left + 30:x_right- 30]
			#stop = findStop(roi, warp_Img)		
			#cv2.imshow('cv_image', process_Img)
			#cv2.waitKey(1)
			auto_drive(steer,speed)
			#if speed < 10:
			#	speed +=0.1
	
	#cap.release()
	cv2.destroyAllWindows()
		
		
if __name__ == '__main__':
	main()

