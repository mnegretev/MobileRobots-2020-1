#!/usr/bin/env python 

import rospy
from geometry_msgs.msg import PoseStamped 
from visualization_msgs.msg import Marker 
from geometry_msgs.msg import Twist  
from nav_msgs.msg import Path
import tf 

def goal_callback(msg):
	x = msg.pose.position.x
	y = msg.pose.position.y

	rospy.loginfo("x: {}, y: {}".format(x,y))
	if x or y:
		print("Hello I am not empty")
	pose = PoseStamped()
	pose.header.stamp = rospy.Time.now()
	pose.header.frame_id = "map"
	pose.position.x = x
	pose.position.y = y
	pose.pose.position.z = 0

	quaternion = tf.transformations.quaternion_from_euler(0,0,0)
	pose.pose.orientation.x = quaternion[0]
	pose.pose.orientation.y = quaternion[1]
	pose.pose.orientation.z = quaternion[2]
	pose.pose.orientation.w = quaternion[3]

	


def Posicion(msg):
	#marker = []
	#marker = Marker()
	#x=marker.pose.position
	#print(x)
	#x=[]
	#x[1]=len(msg.pose.position)
	y=msg.pose.position[1]
	print(y)




def main():
	print "PRACTICE F - " 
	rospy.init_node('final_proyect')
	rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_callback)
	rospy.Subscriber("/hri/visualization_marker",Marker, Posicion)

	while not rospy.is_shutdown():
		rospy.spin()


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass