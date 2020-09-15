from obstacle_detector.msg import Obstacles
import numpy as np
import math

class Obstacle_path:
	def __init__(self):
		self.L_Detected = Fasle
		self.R_Detected = False
		self.L_Avoid_1 = False
		self..L_Avoid_2 = False
		self.R_Avoid = False
		self.mode = 0
		self.steer = 0
		self.throttle = 0
	def check_obstacle(self,msg):
		for circle in obstacles.circles:
			self.p = circle.center
			if -0.4 < p.y < 0:
				if -0.4 < p.x <0.6:
					if p.x > 0.15:
						self.L_Detected = True
					
					else:
						self.R_Detected = True
			if <p.y
	
	def avoid_obstacle(self):
		while True:
			if self.L_Avoid_1 and self.L_Avoid_2 and self.R_Avoid:
				#steer for car's yaw to straight
				self.steer = -15
				self.throttle = -5
				break
			#for minus car_run_spped
			elif self.L_Avoid_1 and self.R_Avoid:
				#steer to right
				if self.L_Detected:
					self.steer = 30
					self.throttle = 5
				elif not self.L_Detected:
					self.steer = 0
					self.throttle = 0
					self.L_Avoid_2 = True
			
			elif self.L_Avoid_1:
				#steer to left
				if self.L_Detected:
					self.steer = -30 
					self.throttle = 5
				elif  not self.R_Detected and self.L_Detected:
					self.steer = 0
					self.throttle = 0
					self.R_Avoid = True
			else:
				#steer to right
				if self.L_Detected:
					self.steer = 30
					self.throttle = 5
				elif not self.L_Detected and self.R_Detected:
					self.self.steer = 0
					self.throttle = 0
					self.L_Avoid_2 = True
		
				
