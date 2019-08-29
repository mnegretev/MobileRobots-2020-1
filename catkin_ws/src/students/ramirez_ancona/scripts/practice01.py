#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2020-1
# PRACTICE 1 - PATH PLANNING BY BREADTH FIRST SEARCH AND DEPTH FIRST SEARCH
#
# Instructions:
# Write the code necessary to plan a path using two search algorithms:
# Breadth first search and Depth first search
#

import sys
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from navig_msgs.srv import CalculatePath
from navig_msgs.srv import CalculatePathResponse
from collections import deque

NAME = "APELLIDO_PATERNO_APELLIDO_MATERNO"

def callback_bfs(req):
    print "Calculating path by breadth first search"
    steps = 0
    msg_path = Path()
    #
    # TODO:
    # Write a breadth first search algorithm to find a path between the start position given by
    # [req.start.pose.position.x, req.start.pose.position.y]
    # and the goal position, given by
    # [req.goal.pose.position.x, req.goal.pose.position.y]
    # Use the 'steps' variable to store the total steps needed for calculations
    #
    
    print "Path calculated after " + str(steps) + " steps."
    msg_path.header.frame_id = "map"
    #
    # TODO:
    # Store the resulting path in the 'msg_path' variable (see online documentation for nav_msgs/Path)
    # Return the appropiate response.
    #
    pub_path = rospy.Publisher('/navigation/path_planning/calculated_path', Path, queue_size=10)
    pub_path.publish(msg_path)
    return CalculatePathResponse(msg_path)

def callback_dfs(req):
    print "Calculating path by depth first search" ####
    steps = 0
    msg_path = Path()
    #
    # TODO:
    # Write a depth first search algorithm to find a path between the start position
    # [req.start.pose.position.x, req.start.pose.position.y]
    # and the goal position
    # [req.goal.pose.position.x, req.goal.pose.position.y]
    # Use the 'steps' variable to store the total steps needed for calculations
    #
    
    print "Path calculated after " + str(steps) + " steps."
    msg_path.header.frame_id = "map"
    #
    # TODO:
    # Store the resulting path in the 'msg_path' variable
    # Return the appropiate response.
    #
    pub_path = rospy.Publisher('/navigation/path_planning/calculated_path', Path, queue_size=10)
    pub_path.publish(msg_path)
    return CalculatePathResponse(msg_path)

def main():
    print "PRACTICE 01 - " + NAME
    rospy.init_node("practice01")
    rospy.Service('/navigation/path_planning/breadth_first_search', CalculatePath, callback_bfs)
    rospy.Service('/navigation/path_planning/depth_first_search'  , CalculatePath, callback_dfs)
    rospy.wait_for_service('/navigation/localization/static_map')
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
