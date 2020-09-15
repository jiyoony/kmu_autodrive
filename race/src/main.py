#!/usr/bin/env python
import cv2, math
import numpy as np
import roslib
import sys
import os
import rospy, rospkg
from rosservice import ROSServiceException
import time
#rosMsg
import sensor_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
from xycar_motor.msg import xycar_motor
from sensor_msgs.msg import Image
from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
import genpy.message
import sensor_msgs.msg
#--src(Module)
from slidewindow import SlideWindow
from warper import Warper
from ObstacleDetector import ObstacleDetector
from Marker_of_YJ import Marker
from pid import PidCal
#from Obstacle_vec import my_vector, Obstacle
#global value
cv_image = np.array([])
obstacles = None
ack_publisher = None
car_run_speed = 0.5
pub = None
ar_tag_Cnt = 0
#class object
slidewindow = SlideWindow()
warper = Warper()
bridge = CvBridge()
obstacle = ObstacleDetector()
mark = Marker()
pid = PidCal()

#cvt rosimg2cv_img
def img_callback(data):
	global cv_image
	try:
		cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
		print(e)

#Obstacle_Callback
def obstacle_callback(data):
	global obstacles
	obstacles = data

#Process Img(Gray, Blur, Edge, Warp) -> Find Detect
def process_img(img):
	steer = 0
	old_Steer = 0

	#processing
	gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	kernel_size = 5
	blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)
	low_threshold = 60#60
	high_threshold = 70# 70
	edges_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)
	warped = warper.warp(edges_img)

	#find detect
	ret, x_left,x_right, w_slide_img, flag = slidewindow.w_slidewindow(warped)
	if ret:
		h_slide_img, steer, centerY = slidewindow.h_slidewindow(warped, x_left,x_right)
		old_Steer = steer
		return steer, centerY, x_left, x_right

#pub Motor
def auto_drive(pid):
	global pub
	global car_run_speed
	msg = xycar_motor()
	msg.angle = pid
	msg.speed = car_run_speed
	pub.publish(msg)

#Find Stop Line
def findStop(roi, img, mark_Cnt):
	global car_run_speed
	stopFlag = 0
	nonzero = roi.nonzero()

	lower_yellow = np.array([-20, 80, 80])
	upper_yellow = np.array([50, 255, 255])

	img = img[240:, :]
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
	res = cv2.bitwise_and(img, img, mask = mask_yellow)
	cv2.imshow('res',res)
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
			car_run_speed -= car_run_speed * 0.5
	else:
		car_run_speed = 20

	if stopFlag == 2:
		if (mark_Cnt > 15): car_run_spped /= 0.5
	return stopFlag

#main
def main():
	global cv_image
	global pub
	global obstacles
	rospy.sleep(3)
	bridge = CvBridge()
	#main value
	x_location = 0
	steer = 0
	old_steer = 0
	old_mode = 0
	curve_cnt = 0
	angle_max = 0
	stop = None
	true_obst = False
	obst_steer = 0
	
	#pub, sub
	pub = rospy.Publisher('/xycar_motor', xycar_motor,queue_size = 1)
	image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,img_callback)
	obstacle_sub = rospy.Subscriber("/obstacles", Obstacles, obstacle_callback, queue_size = 1)
	rospy.init_node('Flash_Fast_Very+Good',anonymous=True)
	auto_drive(steer)
	# -- 
	while not rospy.is_shutdown():
#		true_obst, obst_steer = obst.cal_path(obstacles)
		inv_steer = obst_steer * (-1)
		time.sleep(0.1)
		warp_Img = warper.warp(cv_image)
		
		#find Lane and find Stop Line
#		lane_steer, centerY, left_start, right_start = process_img(cv_image)
#		roi_Img = process_Img[centerY - 20: centerY + 20, left_start + 30 : right_start - 30]
#		stop = findStop(roi_Img, warp_Img)

#		if cv2.waitKey(1) & 0xFF == ord('q'):
#			break
		

		mark.main()	
		#Obstacle Mode
#		if true_obst == False:
#			if inv_steer <-1 :
#				print('inv_mode == 1')
#				for tmp_steer in range(int(inv_steer) , 0, 1):
#					auto_drive(tmp_steer)
#					print(tmp_steer)
#					print('-------------------')
#					time.sleep(0.001)
#					#if obst.isDetected != 0:
#					#	break
#					inv_steer +=1
#			elif inv_steer > 1:
#				print('inv_mode == 2')
#				for tmp_steer in range(int(inv_steer), 0, -1):
#					auto_drive(tmp_steer)
#					print(tmp_steer)
#					print('-------------------')
#					time.sleep(0.001)
#					#if obst.isDetected != 0:
#					#	break
#					inv_steer -=1 
			#Obstacle is not detected, Steer -> Lane Steer
#			else:
#				steer = lane_steer
		#Obstacle is detected and avoid Obstacle
#		else:
#			steer = obst_steer
		print('-------------------')
		#steer = pid.pid_control(steer)
		#steer = np.rad2deg(steer)						
		print("Steer :", -steer)
		print('-------------------')
		print("Inv : ",inv_steer)
#		auto_drive(-steer)
#		inv_steer = 3*steer
	
	cap.release()
	cv2.destroyAllWindows()


if __name__ == '__main__':
	main()

