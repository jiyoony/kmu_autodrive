#!/usr/bin/env python
# -*- coding: utf-8 -*-


#수평 주차

import numpy as np
import rospy
from obstacle_detector.msg import Obstacles
from xycar_motor.msg import xycar_motor
import math
import time

pub = None
car_run_speed = 3
yaw_rate = 0
ndetect = 0
steer = 0

class Parking:
	def __init__(self):
		self.mode = 0
		self.throttle = 0
		self.flag = 0
		self.Isflag3 = False
		self.now_time = 0
		self.old_time = 0
	def execute(self):
		global car_run_speed
		steer = 0
		
		for circle in obstacles.circles:
			x = circle.center.x
			y = circle.center.y
			if self.flag == 0 and self.mode == 0:				# 만약 주차모드가 아니고 flag가 0이면								
				# print("1 :", "x:",x,"y:",y)
				if 0 < x < 0.6 and -1 < y < -0.2:			# 만약 lidar가 object를 (0 ~ 0.6, -1 ~ -0.2)(기둥)에서 감지하면
					print("2 :", "x:",x,"y:",y)	
					car_run_speed = 0				# 차 속도 0, 주차모드 ON, flag 1
					self.mode = 1
					self.flag = 1
			elif self.mode == 1:						# 만약 주차모드 상태라면
				if self.flag == 1: 
					car_run_speed = -2
					#print("3:","x:", x,"y:", y)
					if 0.85 < x < 1.3: #and -0.3 < y < -0.03:	# 만약 lidar가 object를 (0.85 ~ 1.3, -0.3 ~ -0.03)(모서리 기둥)에서 감지하면
						#print("4:","x:", x,"y:", y)
						self.flag = 2
					# else -> 차 각도 -7 -> 우향 후진
					else:
						#print(steer)
						steer = -7		
				elif self.flag == 2:
					if 0.1 < x < 0.3:				# 주차선맞추기(오른쪽 기둥과의 거리) -> 만약 lidar가 object를 (-0.18 ~ -0.23, 0)에서 감지하면
						steer = 0				# 차 각도 0, car speed 0
						self.flag = 3
					else:
						steer = 5
				elif self.flag == 3:
					#print("5:","x:", x,"y:", y)
					#if -0.1 < y < -0.5:				# 주차선맞추기(앞쪽 기둥과의 거리) -> 만약 lidar가 object를 (0, -0.1 ~ -0.5)에서 감지하면
						#car_run_speed = 0
						#self.mode = 0
					self.now_time = time.time()			# 주차선맞추기(앞쪽전진 1초후 정지)
					if (self.old_time == 0): 
						self.old_time = time.time()
					if (self.now_time - self.old_time > 1): 	# 1초후 정지
						car_run_speed = 0
						self.mode = 0
					# else -> 앞으로 이동
					else:
						car_run_speed = 2
		return steer
	
def auto_drive(steer):
	global pub
	global car_run_speed		

	x_m = xycar_motor()
	x_m.angle = steer
	x_m.speed = car_run_speed
	pub.publish(x_m)
						

obstacles = None

def obst_callback(data):
	global obstacles
	obstacles = data

def main():
	global obstacles
	global pub
	

	prk = Parking()

	pub = rospy.Publisher('/xycar_motor', xycar_motor, queue_size=1)
	rospy.init_node('parking',anonymous=True)
	rospy.Subscriber("/obstacles", Obstacles, obst_callback, queue_size = 1)
	rospy.sleep(3)
	
	
	print("parking mode")
	while not rospy.is_shutdown():	
		if obstacles:
			steer = prk.execute()
			auto_drive(steer)
			rospy.sleep(0.1)

if __name__ == '__main__':
	main()
