from obstacle_detector.msg import Obstacles
import rospy
import numpy as np
import math
class ObstacleDetector:
	def __init__(self):
		self.mode = 0
		self.cnt = 0
		self.steer = 0
		self.tmp = 0.1
		self.steer_old = 0
		self.left_list = []
		self.right_list = []
		self.p = 0
	def check(self,obstacles):
		for circle in obstacles.circles:
			self.p = circle.center
			if len(self.left_list) > 10:
				del self.left_list[0]
			
			if len(self.right_list) > 10:
				del self.right_list[0]
	
			if -0.4 <p.y < 0:
				if -0.4<abs(p.x) <0.6:
					if p.x > 0.15:
						#self.steer =-1 *  math.cos(p.y / math.sqrt(math.pow(p.x,2)+math.pow(p.y,2)))
						self.left_list.append(p.x)
						self.mode = 1	
						self.steer = -(1 - np.mean(self.left_list) + circle.radius)
						
					elif p.x < 0.15:
						#self.steer = math.cos(p.y/math.sqrt(math.pow(p.x,2)+math.pow(p.y,2)))
						self.right_list.append(p.x)
						self.mode =2 
						self.steer = -(-1 - np.mean(self.right_list) - circle.radius)
					break
			else:
				self.mode = 0
		self.steer = math.degrees(self.steer)
		if -70<self.steer_old<70:
			self.steer_old = self.steer
		if self.mode == 0:
			self.left_list = []
			self.right_list = []
		if self.steer >100:
			self.steer = self.steer_old
		elif self.steer <-100:
			self.steer = self.steer_old
		print("Lidar Mode :", self.mode)

		
		return self.mode, self.steer
