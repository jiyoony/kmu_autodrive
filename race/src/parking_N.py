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
	def execute(self, obstacles):
		steer = 0
		car_run_speed = 0
		#print(obstacles)
		for circle in obstacles.circles:
			x = circle.center.x
			y = circle.center.y
			#print("obstacle")
			if self.flag == 0 and self.mode == 0:
				car_run_speed = 0				# 차 속도 0, 주차모드 ON, flag 1
				self.mode = 1
				self.flag = 1
				self.old_time = 0
			elif self.mode == 1:						# 만약 주차모드 상태라면
				if self.flag == 1: 
					self.now_time = time.time()
					if (self.old_time == 0): 
						self.old_time = time.time()
					if (self.now_time - self.old_time > 4.3):	# operate to left steer after 4 second
						if 0.5 < x < 1.3: #and -0.2 < y < -0.1:	# 만약 lidar가 object를 (0.5 ~ 1.3, -0.2 ~ -0.1)(모서리 기둥)에서 감지하면
							#print("4:","x:", x,"y:", y)
							self.flag = 2
							self.old_time = 0
					# else -> 차 각도 -11 -> 우향 후진
					else:
						#print(steer)
						car_run_speed = -2
						steer = -5
				elif self.flag == 2:
					self.now_time = time.time()
					if (self.old_time == 0): 
						self.old_time = time.time()
					if (self.now_time - self.old_time > 3.51):
						steer = 0				# 차 각도 0, car speed 0
						self.flag = 3
						self.old_time = 0
					else:
						car_run_speed = -2
						steer = 5
				elif self.flag == 3:
					#print("5:","x:", x,"y:", y)
					#if -0.1 < y < -0.5:				# 주차선맞추기(앞쪽 기둥과의 거리) -> 만약 lidar가 object를 (0, -0.1 ~ -0.5)에서 감지하면
						#car_run_speed = 0
						#self.mode = 0
					self.now_time = time.time()			# 주차선맞추기(앞쪽전진 1.6초후 정지)
					if (self.old_time == 0): 
						self.old_time = time.time()
					if (self.now_time - self.old_time > 1.4	): 	# 1.6초후 정지
						car_run_speed = 0
						self.mode = 0
					# else -> 앞으로 이동
					else:
						car_run_speed = 2
		return steer, car_run_speed
