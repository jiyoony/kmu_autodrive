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
from obstacle_avoid import Obstacle_avoid
from parking_N import Parking

cv_image = None
obstacles = None
ack_publisher = None
car_run_speed = 10
pub = None
stop_cnt = 0
park_mode = 0
slidewindow = SlideWindow()
warper = Warper()
bridge = CvBridge()
pid = PidCal()
curve = CurveDetector()
avoid = Obstacle_avoid()
parking = Parking()
time_old = 0

def check_time():
		if time.time()-time_old < 5:
			return False

		else:
			return True
	
def img_callback(data):
	global cv_image
	try:
		tmp = bridge.imgmsg_to_cv2(data, "bgr8")
		cv_image = cv2.resize(tmp,(640,480))
	except CvBridgeError as e:
		print(e)


def findStop(roi, img):#stopFlag /0 : default /1 : StopLine /2 : StartLine Detect /3 : StartLine CK
	global stop_cnt
	global time_old
	stopFlag = 0
	#cv2.imshow('roi',roi)
	#cv2.waitKey(1)
	nonzero = roi.nonzero()
	print(len(nonzero[0]))
	if check_time():
		if len(nonzero[0]) >7600:
			stop_cnt +=1
			time_old = time.time()
	print('stop_cnt : ',stop_cnt)
	return stopFlag



def process_img(img):
	#cv2.imshow('aasdasd',img)
	#cv2.waitKey(1)
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
	warped_2 = warper.warp(img)
	roi = warped[200:300, :]
	h_slide_img,x_location= slidewindow.slidewindow(warped)
	stop_flag=findStop(roi,warped_2)
	return h_slide_img, x_location,stop_flag

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


def check_LR():
	global obstacles
	for circle in obstacles.circles:
		x = circle.center.x
		y = circle.center.y
		if 0 < x < 0.5 and -0.5 < y < 0:
			return 'right'
		elif -0.5 < x < 0 and -0.4 < y < 0:
			return 'left'
#Find Stop Line



def main():
	global cv_image
	global pub
	#global cap
	#global obstacles
	global car_run_speed
	global stop_cnt
	global obstacles
	global avoid
	start_time=time.time()
	#rospy.sleep(3)
	x_location = 0
	steer = 0
	old_mode = 0
	angle_max = 0
	stop = None
	lab_cnt =0
	park_mode = 0
	rospy.init_node('autodrive',anonymous=True)
	pub = rospy.Publisher('/xycar_motor', xycar_motor, queue_size=1)
	image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,img_callback)
	obstacle_sub = rospy.Subscriber("/obstacles", Obstacles, obstacle_callback, queue_size = 1)
	while time.time()-start_time <3:
		auto_drive(0,0)
	car_run_speed = 10
	#car_run_speed = 0
	time.sleep(3)
	while not rospy.is_shutdown():	
	
		if cv_image is not None:
			if park_mode == 0:
				
				process_Img, x_location,stop_flag= process_img(cv_image)
				steer = pid.pid_control(x_location)
			
				print("STEER : ",steer)
				print("count_curve", curve.curve_count)
				print("LabCNT : ", lab_cnt)
			
				if curve.curve_count == 0:
					car_run_speed = 8
					#if car_run_speed+0.3 < 9.5:
					#	car_run_speed+=0.15
					curve.list_update(steer)
					curve.count_curve()
				elif curve.curve_count == 1:
				#	if speed < 8:
					curve.list_update(steer)
					curve.count_curve()
				elif curve.curve_count == 2:
					car_run_speed = 6.5
					if check_LR() == 'right':
						while not avoid.Isflag or avoid.flag !=3:
							steer = avoid.avoid_RLR(obstacles)
							auto_drive(steer,car_run_speed)
					elif check_LR() == 'left':
						while not avoid.Isflag or avoid.flag !=3:
							steer = avoid.avoid_LRL(obstacles)
							auto_drive(steer,car_run_speed)
					car_run_speed = 5
					if stop_cnt == 1:
						time.sleep(5)
						stop_cnt += 1
						curve.curve_count +=1
				if stop_cnt == 3:
					stop_cnt = 0
					curve.curve_count = 0
					avoid.mode = 0
					avoid.flag = 0
					avoid.steer = 0
					avoid.old = 0
					avoid.now = 0
					avoid.Isflag = False
					lab_cnt +=1
				if lab_cnt == 3:
					park_mode = 1
				#print('curve`s sum',abs(sum(curve.pid_list)))
				#print("pid_steer : ", steer)
				#print(x_location)
				#warp_Img = warper.warp(cv_image)
				#roi = thr[centerY - 60:centerY + 40, x_left + 30:x_right- 30]
				#stop = findStop(roi, warp_Img)		
				#cv2.imshow('cv_image', process_Img)
				#cv2.waitKey(1)
				auto_drive(steer,car_run_speed)
				#if speed < 10:
				#	speed +=0.1
			else:
				steer, car_run_speed = parking.execute(obstacles)
				auto_drive(steer,car_run_speed)
	
	#cap.release()
	cv2.destroyAllWindows()
		
		
if __name__ == '__main__':
	main()

