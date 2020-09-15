#!/usr/bin/env python
import cv2, math
import time
import numpy as np
import roslib
import sys
import sensor_msgs.msg
from xycar_motor.msg import xycar_motor
from std_msgs.msg import String
#from ObstacleDetector import ObstacleDetector
from Obstacle_vec import Obstacle_path
import os
from obstacle_detector.msg import Obstacles
import rospy, rospkg
import genpy.message
import sensor_msgs.msg
from rosservice import ROSServiceException
from pid import PidCal
import signal

cv_image = np.array([])
obstacles = None
ack_publisher = None
car_run_speed = 20
ar_tag_Cnt = 0
pub = None
pid = PidCal()
obst = Obstacle_path()
inv_steer = 0



def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def obstacle_callback(data):
	global obstacles
	obstacles = data



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
	global cap
	global w,h
	global obstacles
	global inv_steer
	rospy.sleep(3)
	x_location = 0
	steer = 0
	old_steer = 0
	old_mode = 0
	curve_cnt = 0
	angle_max = 0
	stop = None
	pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
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

