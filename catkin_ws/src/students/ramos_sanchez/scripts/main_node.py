#!/usr/bin/env python 

import rospy
from geometry_msgs.msg import PoseStamped 
from geometry_msgs.msg import Twist  
from nav_msgs.msg import Path
from std_msgs.msg import String
import tf 

def start_callback(msg):
	global start_x, start_y, start_a
	message = msg.data
	start_position = message.split(",")
	start_x = start_position[0]
	start_y = start_position[1]
	start_a = start_position[2]

def main():
	rospy.init_node('final_proyect')
	#rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_callback)
	rospy.Subscriber("/navigation/localization", String, start_callback)
	while not rospy.is_shutdown():
		rospy.spin()


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass