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
#from ObstacleDetector import ObstacleDetector
from Obstacle_vec import Obstacle_path
import os
from obstacle_detector.msg import Obstacles
import rospy, rospkg
import genpy.message
import sensor_msgs.msg
from rosservice import ROSServiceException
from slidewindow import SlideWindow
from warper import Warper
from pid import PidCal
from marker import MarkerPose
import signal

cv_image = np.array([])
obstacles = None
ack_publisher = None
car_run_speed = 20
ar_tag_Cnt = 0
pub = None
slidewindow = SlideWindow()
warper = Warper()
mark = Marker()
bridge = CvBridge()
pid = PidCal()
obst = Obstacle_path()
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

def obstacle_callback(data):
	global obstacles
	obstacles = data


def process_img(img):
	global car_run_speed
	steer = 0
	old_steer = 0

	gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	kernel_size = 5
	blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)
	low_threshold = 60#60
	high_threshold = 70# 70
	kernel = np.ones((3, 3),np.uint8)
	edges_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)
	warped = warper.warp(edges_img)
	ret, x_left,x_right, w_slide_img, flag = slidewindow.w_slidewindow(warped)
	if ret:
		h_slide_img, steer, centerY = slidewindow.h_slidewindow(warped, x_left, x_right, flag)
		#if abs(steer - old_steer > 10): car_run_speed /= 2

		old_steer = steer
		return warped, steer, 240, 200, 480

def findStop(roi, img, mark_Cnt):
	global car_run_speed
	global ar_tag_Cnt
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
		if (mark_Cnt > 15) car_run_spped /= 0.5
	#print("Stop Flag : " , stopFlag, " Car_run_speed : ", car_run_speed)
	return stopFlag

def auto_drive(pid):
	global pub
	global car_run_speed

	msg = xycar_motor()
	msg.angle = pid
	msg.speed = 0
	#msg.speed = car_run_speed
	pub.publish(msg)



def main():
	global cv_image
	global pub
	global obstacles
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
	obstacle_sub = rospy.Subscriber("/obstacles", Obstacles, obstacle_callback, queue_size = 1)
	rospy.init_node('autodrive',anonymous=True)
	cap = cv2.VideoCapture(-1)
	#cap.set(3,1280)
	#cap.set(4,720)
	auto_drive(steer)
	while not rospy.is_shutdown():
		#steer = pid.pid_control(steer)
		#auto_drive(steer) 
		#ret,cv_image = cap.read()
		true_obst, obst_steer = obst.cal_path(obstacles)
		time.sleep(0.1)
		
		warp_Img = warper.warp(cv_image)
		process_Img, lane_steer, centerY, left_start, right_start = process_img(cv_image)
		#print("slide")
		roi_Img = process_Img[centerY - 20: centerY + 20, left_start + 30 : right_start - 30]
		stop = findStop(roi_Img, warp_Img)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
		

		if true_obst == False:
			if inv_steer <-1 :
				print('inv_mode == 1')
				for tmp_steer in range(int(inv_steer) , 0, 1):
					auto_drive(tmp_steer)
					print(tmp_steer)
					print('-------------------')
					time.sleep(0.001)
					#if obst.isDetected != 0:
					#	break
					inv_steer +=1
			elif inv_steer > 1:
				print('inv_mode == 2')
				for tmp_steer in range(int(inv_steer), 0, -1):
					auto_drive(tmp_steer)
					print(tmp_steer)
					print('-------------------')
					time.sleep(0.001)
					#if obst.isDetected != 0:
					#	break
					inv_steer -=1 
			else:
				steer = lane_steer
		else:
			steer = obst_steer
		print('-------------------')
		#steer = pid.pid_control(steer)
		#steer = np.rad2deg(steer)						
		print("Steer :", -steer)
		print('-------------------')
		print("Inv : ",inv_steer)
		auto_drive(-steer)
		inv_steer = 3*steer
		
	cap.release()
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()

