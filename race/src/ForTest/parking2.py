#!/usr/bin/env python
# -*- coding: utf-8 -*-


#수평 주차
#Dc구하기 -> 각도 a, path의 길이 구하기 -> 구해놓은 길이를 따라 움직여서 주차

import numpy as np
import rospy
import sys
from sensor_msgs.msg import LaserScan
import math

class Parking:
	def __init__(self):
		self.mode = 0
		# mode = 0 : 주차전    mode 1 : 주차시작
		self.steer = 0
		self.throttle = 0
		self.D_new = 0 # 기둥을 지나 D_old를 만들어 줄 거리 
		self.D_old = 0 # 기둥을 지나간 일정거리 # D_new - 차 길이
		self.D_obj = 0 # 차와 기둥사이의 거리
		self.D_park = 0 # 플래그와 기둥사이의 거리
		self.R = 0 # 벽과 차폭 중간까지의 거리 
		self.Wc = 0 # 차폭
		self.a = 0
		self.b = 0
		self.M = 0 # 플래그와 기둥사이의 거리 + D_car
		self.L = 0

	def execute(self):
		D = self.D_obj + self.Wc/2
		a = np.arcsin(2*self.R - self.Wc/2 - D/2*self.R)
		self.M = 2* self.R * np.cos(a)
		self.D_old = self.M - self.D_park
		self.b = 90 -self.a
		L = 2*math.pi*self.R*self.b/360
		
		#if # 라이다가 벽을 감지하다가 기둥을 감지하면
			# D_new만큼 이동
			# D_new를 간 후 조향 왼쪽으로 self.steer = ? 을 줘서 후진으로 L 이동
		#	if # L만큼 갔으면 조향 왼쪽으로 self.steer = ? 줘서 후진으로 L 이동하면
		#		if # 앞에 있는 기둥 플래그 인식하면
					# 앞에 있는 기둥 플래그 - 290mm 까지 이동

obstacles = None

def obst_callback(data):
	global obstacles
	obstacles = data

def main():
	global obstacles
	rospy.init_node('parking',anonymous=True)
	rospy.Subscriber("/scan", LaserScan, obst_callback, queue_size = 1)
	rospy.sleep(3)

	while not rospy.is_shutdown():	
		if obstacles:
			print(type(obstacles))
			#print(len(obstacles))	

if __name__ == '__main__':
        #rospy.sleep(3)
	#rospy.Subscriber("/scan", LaserScan, obst_callback, queue_size = 1)
	main()
