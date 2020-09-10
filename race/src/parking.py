import numpy as np
import rospy, rospkg
import sys
import sensor_msgs.msg import LaserScan

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
