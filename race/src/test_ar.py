import cv2
import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMArkers

class check_ar:
	def __init__(self):
		rospy.init_node('recog_marker',annonymous = True)
		self.sub = rospy.Subscriber('/ar_pose_marker',AlvarMarkers, self.get_marker)
		self.rate = rospy.Rate(10)
		self.p = PoseStamped()
		self.id = -1		


	def get_marker(self, msg):
		n = len(msg.markers)
		if (n != 0):
			self.id = msg.markers[0].id
			self.p = msg.markers[0].pose

			self.print_marker()


	def print(_marker(self):
		print("marker id     = %d" %(self.id))
	    print("position.x    = %f" %(self.p.pose.position.x))
	    print("position.y    = %f" %(self.p.pose.position.y))
	    print("position.z    = %f" %(self.p.pose.position.z))
	    print("orientation.x = %f" %(self.p.pose.orientation.x))
	    print("orientation.y = %f" %(self.p.pose.orientation.y))
	    print("orientation.z = %f" %(self.p.pose.orientation.z))
	    print("orientation.w = %f" %(self.p.pose.orientation.w))

if __name__ == '__main__':
	try:
		check_ar()
		rospy.spin()

	except rospy.ROSInterruptException: pass
