#!/usr/bin/env python
import roslib
import sys
import os
import rospy, rospkg
import time
#rosMsg
import sensor_msgs.msg
from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
import genpy.message
import sensor_msgs.msg
#global value
obstacles = None
old_time = 0 #find StopLine ~> Stop Time Ck
#class object

car_run_speed = 4
steer = 0

class Obstacle_avoid:
	def __init__(self):
		self.mode = 0
		self.flag = 0
		self.steer = 0
		self.old = 0
		self.now = 0
		self.Isflag = False

	def avoid_RLR(self,obstacles):
		steer = 0
		speed = 0
		self.now = time.time()
		for circle in obstacles.circles:
			x = circle.center.x
			y = circle.center.y

			# Right Left Right
			
			if self.flag == 0 and self.mode == 0:
				if 0 < x < 0.5 and -0.5 < y < 0:
					print("R_Detected")
					self.flag = 1
					self.mode = 1
					self.old = 0
			if self.mode == 1:
				if self.flag == 1:
					if self.old == 0:
						self.old = time.time()
					if self.Isflag:
						if (self.now - self.old) > 2.2: #3
							if 0 < x < 0.5 and -0.5 < y < 0:
								print("R_Detected")
								self.old = 0
								self.flag = 2
						else:
							steer = 0.2
					else:
						if (self.now - self.old) > 0.6: #1
					
							self.old = 0
							self.flag = 2
						else:
							steer = 0.2
				elif self.flag == 2:
					if self.old == 0:
						self.old = time.time()
					if self.Isflag:
						if (self.now - self.old) > 2: #4
							self.old = 0
							self.flag = 3
						else:
							steer = -0.2
					else:
						if (self.now - self.old) > 2: #2
							if -0.5 < x < 0 and -0.4 < y < 0:
								print("L_Detected")
								self.old = 0
								self.flag = 1
								self.Isflag = True
						else:
							steer = -0.2
				elif self.flag == 3 and self.Isflag:
					if self.old == 0:
						self.old = time.time()
					if (self.now - self.old) > 0.5: #5
						self.old = 0
						#end obstacle
						self.flag = 100
					else:
						steer = 0
					
				
		return steer



	def avoid_LRL(self,obstacles):
		steer = 0
		speed = 0
		self.now = time.time()
		for circle in obstacles.circles:
			x = circle.center.x
			y = circle.center.y

			# Left Right Left
			
			if self.flag == 0 and self.mode == 0:
				if -0.5 < x < 0 and -0.525 < y < 0:
					print("L_Detected")
					self.flag = 1
					self.mode = 1
					self.old = 0
			if self.mode == 1:
				if self.flag == 1:
					if self.old == 0:
						self.old = time.time()
					if self.Isflag:
						if (self.now - self.old) > 2.4: #3
							if -0.5 < x < 0 and -0.5 < y < 0:
								print("L_Detected")
								self.old = 0
								self.flag = 2
						else:
							steer = -0.2
					else:
						if (self.now - self.old) > 0.8: #1
					
							self.old = 0
							self.flag = 2
						else:
							steer = -0.2
				elif self.flag == 2:
					if self.old == 0:
						self.old = time.time()
					if self.Isflag:
						if (self.now - self.old) > 1.6: #4
							self.old = 0
							self.flag = 3
						else:
							steer = 0.2
					else:
						if (self.now - self.old) > 1.7: #2
							if 0 < x < 0.5 and -0.4 < y < 0:
								print("R_Detected")
								self.old = 0
								self.flag = 1
								self.Isflag = True
						else:
							steer = 0.2
				elif self.flag == 3 and self.Isflag:
					if self.old == 0:
						self.old = time.time()
					if (self.now - self.old) > 0.5: #5
						self.old = 0
						#end obstacle
						self.flag = 100
					else:
						steer = 0
					
				
		return steer
def auto_drive(steer):
	global pub
	global car_run_speed		

	x_m = xycar_motor()
	x_m.angle = steer
	x_m.speed = car_run_speed
	pub.publish(x_m)

def obstacle_callback(data):	
	global obstacles
	obstacles = data

#main
def main():
	global cv_image
	global pub
	global obstacles
	#rospy.sleep(3)
	#main value
	x_location = 0
	#steer = 0
	old_steer = 0
	old_mode = 0
	curve_cnt = 0
	angle_max = 0
	stop = None
	true_obst = False
	obst_steer = 0
	obst = Obstacle_path()
	#pub, sub
	pub = rospy.Publisher('/xycar_motor', xycar_motor,queue_size = 1)
	obstacle_sub = rospy.Subscriber("/obstacles", Obstacles, obstacle_callback, queue_size = 1)
	rospy.init_node('Flash_Fast_Very+Good',anonymous=True)
	#auto_drive(steer)
	# -- 
	time.sleep(3)
	while not rospy.is_shutdown():
		steer = obst.avoid_RLR(obstacles)
		auto_drive(steer)
		#print(obst.mode)
		

if __name__ == '__main__':
	main()

