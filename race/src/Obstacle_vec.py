from obstacle_detector.msg import Obstacles
import numpy as np
import math

class Obstacle_path:
	def __init__(self):
		self.isDetected = 0
	def cal_path(self, obstacles):	
		R_obs_x = 0
		R_obs_y = 0
		L_obs_x = 0
		L_obs_y = 0
		L_detec = False
		R_detec = False
		
		L_vec = 0
		R_vec = 0
		totalPointX = 0 
		totalPointY = 0 
		for circle in obstacles.circles:
			if -0.8 < circle.center.y < 0:
				if -0.4 < circle.center.x < 0.6:
					if circle.center.x >0.15 and not R_detec: 
						R_obs_x = circle.center.x + circle.radius
						R_obs_y = circle.center.y
						R_detec = True
					elif circle.center.x <0.15 and not L_detec:
						L_obs_x = circle.center.x - circle.radius
						L_obs_y = circle.center.y
						L_detec = True
			if R_detec and L_detec:
				break

		if R_detec and L_detec:
			self.isDetected = 3
			totalPointX = R_obs_x + L_obs_x
			totalPointY = R_obs_y + L_obs_y
			self.steer = math.degrees(math.atan(totalPointX / totalPointY))
			return True,self.steer

		elif R_detec:
			self.isDetected = 2
			totalPointX = -R_obs_x
			totalPointY = R_obs_y
			self.steer = math.degrees(math.atan(totalPointX / totalPointY))
			return True,self.steer

		elif L_detec:
			self.isDetected = 1
			totalPointX = -L_obs_x
			totalPointY = L_obs_y
			self.steer = math.degrees(math.atan(totalPointX / totalPointY))
			return True,self.steer

		else:
			self.idDetected = 0
			return False, 123.123123123

						
