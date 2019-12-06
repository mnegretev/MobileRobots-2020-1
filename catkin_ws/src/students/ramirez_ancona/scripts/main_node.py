#!/usr/bin/env python 

import rospy
from geometry_msgs.msg import PoseStamped 
from geometry_msgs.msg import Twist  
from nav_msgs.msg import Path
from std_msgs.msg import String
import tf
from nav_msgs.srv import GetMap
import math
from navig_msgs.srv import CalculatePath
from navig_msgs.srv import SmoothPath

def goal_callback(goal):
	listener = tf.TransformListener()
	listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(5.0))
	robot_x, robot_y, robot_a = get_robot_pose(listener)
	start = PoseStamped()
	start.pose.position.x = robot_x
	start.pose.position.y = robot_y
	map = get_map()
	path = calc_path(start, goal, map)
	smooth = smooth_path(path)
	control(smooth)

def get_robot_pose(listener):
    try:
        (trans, rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
        robot_x = trans[0]
        robot_y = trans[1]
        robot_a = 2*math.atan2(rot[2], rot[3])
        if robot_a > math.pi:
            robot_a -= 2*math.pi
        return robot_x, robot_y, robot_a
    except:
        pass
    return None

def get_map():
	rospy.wait_for_service('/navigation/localization/static_map')
	try:
		get_map = rospy.ServiceProxy('/navigation/localization/static_map', GetMap)
		map = get_map()
		return map.map
	except rospy.ServiceException, e:
		print "Service call failed: %s" % e

def calc_path(start, goal, map):
	rospy.wait_for_service('/navigation/localization/static_map')
	try:
		calculated_path = rospy.ServiceProxy('/navigation/path_planning/a_star_search', CalculatePath)
		path = calculated_path(start, goal, map, 0)
		return path.path
	except rospy.ServiceException, e:
		print "Service call failed: %s" % e

def smooth_path(path):
	rospy.wait_for_service('/navigation/path_planning/smooth_path')
	try:
		calculated_path = rospy.ServiceProxy('/navigation/path_planning/smooth_path', SmoothPath)
		smooth_path = calculated_path(path)
		return smooth_path.smooth_path
	except rospy.ServiceException, e:
		print "Service call failed: %s" % e

def control(smooth_path):
	pub_smooth_path = rospy.Publisher('/navigation/simple_move/follow_path', Path, queue_size=1)
	pub_smooth_path.publish(smooth_path)

def main():
	rospy.init_node('final_proyect')
	rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_callback)

	while not rospy.is_shutdown():
		rospy.spin()

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass