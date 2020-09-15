
#수평 주차
#Dc구하기 -> 각도 a, path의 길이 구하기 -> 구해놓은 길이를 따라 움직여서 주차

import numpy as np
import rospy, rospkg
import sys
import sensor_msgs.msg import LaserScan
import math
from pixel_stop import Pix_Stop

px = Pix_stop()

class Parking:
	def __init__(self):
		self.mode = 0
		self.steer = 0
		self.throttle = 0
		self.D_car = 0 # 기둥을 지나간 일정거리
		self.D_obj = 0 # 차와 기둥사이의 거리
		self.D_park = 0 # 플래그와 기둥사이의 거리
		self.R = 0 # 벽과 차폭 중간까지의 거리
		self.Wc = 0 # 차폭
		self.a = 0
		self.b = 0
		self.M = 0 # 플래그와 기둥사이의 거리 + D_car
		self.L = 0

	def execute(self):
		a = np.arcsin(2R - Wc/2 - D/2R)
		M = 2R * np.cos(a)
		D_car = M - D_park	
		b = 90 -a
		L = 2*math.pi*R*b/360
		
		if # 라이다가 벽을 감지하다가 기둥을 감지하면
			# D_car만큼 이동
			# D_car를 간 후 조향 왼쪽으로 self.steer = ? 을 줘서 후진으로 L 이동
			if # L만큼 갔으면 조향 왼쪽으로 self.steer = ? 줘서 후진으로 L 이동하면
				if # 앞에 있는 기둥 플래그 인식하면
					# 앞에 있는 기둥 플래그 - 290mm 까지 이동

obstacles = None

def callback(data):
	global obstacles
	obstacels = data

def main():
	global obstacles

	rospy.sleep(3)
	rospy.Subscriber("/scan", LaserScan,obstacles, queue_size = 1)
	while not rospy.is_shutdown():
		print(len(obstacles))	

if __name__ == '__main__':
	main()
