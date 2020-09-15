#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from math import pow, sqrt, atan, pi, degrees, cos, sin, atan
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion #, quaternion_from_euler

TARGET_ID = -1
	
'''
	          y                        z      
	          ^  y                     ^       
	  marker  | /                      | robot 
	(on wall) |/                       |       
	          +------> z      x <------+       
	                                  /         
	                                 /          
	                                y      
    dist   = position.z
    
    q(orientation.x, orientation.y, orientation.z, orientation.w)
    0 = euler_from_quaternion(q)[1]  ( z <--> y )
    
    dist_x = dist * cos0
    dist_y = dist * sin0
	       
      | marker  |  (0 < O)                             | marker  |  (0 > O)          
      -----+---------+                            ----------+-----
	   |\R-0    R|                            |     R-0/|
	   |0\       |                            |       /0|
	   |  \      |                            |      /  |
	   |   \     | <-------- dist_x --------> |     /   |
	   |  dist   |                            |  dist   |
	   |     \   |                            |   /     |
	   |      \  |                            |  /      |
	   |       \0|                            | /       |
	   |R    R-0\|          location          |/        |
	   +---------x <<<<<<<     of     >>>>>>> x---------+
	      dist_y              robot              dist_y
	                                                                    
'''           
class MarkerPose:

    def __init__(self):
    
	rospy.init_node('pub_marker_pose', anonymous = True)        
	rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.get_marker )
	
	self.pub = rospy.Publisher('/marker_pose', Pose, queue_size = 10)
	
	self.ar_pose   = Pose()
	
	self.target_id = -1
	self.dist      =  0
        self.flag      =  0
	
    def get_marker(self, msg):
    	find_Tag = False
	n = len(msg.markers)
	
	if( n != 0 ):
	    print("marker found!")
	    find_Tag = True
	    
	    for tag in msg.markers:
	    
	        if(tag.id == TARGET_ID):
	            
	            print("target marker ID checked!")
	            

                    self.flag = 1
                    self.parking_car()

	            theta = self.get_ar_pose(tag)
	            
	            if  (theta >  5.):
	                self.ar_pose.theta = theta - 2 * pi            
	            elif(theta < -5.):
	                self.ar_pose.theta = theta + 2 * pi
	            else:
	                self.ar_pose.theta = theta
	            
	            self.dist = tag.pose.pose.position.z
	            
	            self.ar_pose.x = self.dist * cos(self.ar_pose.theta)
                    print("ar_x = ", self.ar_pose.x)
	            self.ar_pose.y = self.dist * sin(self.ar_pose.theta)
                    print("ar_y = ", self.ar_pose.y)
	            self.pub.publish(self.ar_pose)    
	            self.print_info()
	
	else:
	    pass #print("Marker not found.")
	
	return find_Tag
    
    def get_ar_pose(self, msg):
	"""
	  orientation x,y,z,w --+
	                        +--> 4   +-------------------------+
	input orientaion of marker ----->|                         |
	                                 | euler_from_quaternion() |
	returnned rpy of marker <--------|                         |
	                         +-- 3   +-------------------------+
	         r,p,y angle <---+
	                                 +-------------------------+ 
	  r: euler_from_quaternion(q)[0] | roll  (x) - (y) pitch   | 
	* p: euler_from_quaternion(q)[1] | pitch (y) - (z) yaw  ** | <--
	  y: euler_from_quaternion(q)[2] | yaw   (z) - (x) roll    | 
	                                 +-------------------------+ 
	"""
	q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
	     msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
	     
	theta = euler_from_quaternion(q)[1]
	
	if theta < 0:
	    theta = theta + pi * 2
	if theta > pi * 2:
	    theta = theta - pi * 2

	return theta
	
	
    def print_info(self):
	print("d = %s, x = %s, y = %s, th = %s"
	       %(self.dist, self.ar_pose.x, self.ar_pose.y,
	       degrees(self.ar_pose.theta)))
	       
	print("d = %s, \t\t\t\t  th = %s" 
	       %(sqrt(pow(self.ar_pose.x,2)+pow(self.ar_pose.y,2)),
	         degrees(atan(self.ar_pose.y/self.ar_pose.x))))

    def parking_car(self):
        print("change car mode to parking mode")
	       

if __name__ == '__main__':
    try:
	TARGET_ID = int(input("input marker ID: "))
	MarkerPose()
	rospy.spin()
	
    except rospy.ROSInterruptException:  pass
