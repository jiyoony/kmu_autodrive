#!/usr/bin/env python
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion

TARGET_ID = -1

class Marker:

	def __init__ (self):

		self.target_Id = -1
		self.dist = 0
		self.flag = 0

	def get_Marker(self, msg):
		find_Tag = False
		n = len(msg.markers)

		if n:
			print("Find!")
			find_Tag = True
			print("Tag : ", msg.markers)

			for tag in msg.markers:
				print(tag.id)

	def main(self):
		rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.get_Marker)


'''
      x: 3.78158000793e-312
      y: 3.2959465535e-312
      z: 3.65522180363e-312
'''
'''
      x: 6.69558678202e-312
      y: 5.12909639841e-312
      z: 6.1087391121e-312
'''
