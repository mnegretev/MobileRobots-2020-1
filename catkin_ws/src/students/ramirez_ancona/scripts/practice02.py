#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2020-1
# PRACTICE 2 - PATH PLANNING BY DIJKSTRA AND A-STAR WITH MAP INFLATION AND NEARNESS
#
# Instructions:
# Write the code necessary to plan a path using 
# Dijkstra and A star.
# Before planning, map obstacles must be inflated and nearness should be taken into account
# as part of the cost function.
#

import sys
import copy
import rospy
import heapq
from collections import deque
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from navig_msgs.srv import CalculatePath
from navig_msgs.srv import CalculatePathResponse

NAME = "ramirez_ancona"

def inflate_map(map):
    if rospy.has_param("/navigation/path_planning/inflation_radius"):
        radius = float(rospy.get_param("/navigation/path_planning/inflation_radius"))
    else:
        radius = 1.0        
    print "Inflating map by " + str(radius) + " meters"
    inflated = OccupancyGrid()
    #
    # TODO:
    # Inflate all obstacles in 'map' by 'radius'
    # Store the resulting inflated map in 'inflated'
    #
    
    ####
    return inflated

def get_nearness(map):
    if rospy.has_param("/navigation/path_planning/nearness_radius"):
        radius = float(rospy.get_param("/navigation/path_planning/nearness_radius"))
    else:
        radius = 1.0
    print "Calculating nearness for a radius of " + str(radius) + " m"
    nearness = OccupancyGrid()
    #
    # TODO:
    # Get a map where all free cells within a 'radius' distance from an obstacle
    # contain a number indicating the distance to such obstacles
    # Store the resulting map in 'nearness'
    #
    return nearness

def callback_dijkstra(req):
    print "Calculating path by Dijkstra search"###
    map = inflate_map(req.map)
    map = get_nearness(map)
    steps = 0
    #
    # TODO:
    # Write a Dijkstra algorithm to find a path between the start position
    # [req.start.pose.position.x, req.start.pose.position.y]
    # and the goal position
    # [req.goal.pose.position.x, req.goal.pose.position.y]
    # Consider nearness and distance traveled as cost function.
    # Use the 'steps' variable to store the total steps needed for calculations
    # HINT: Use a heap structure to keep track of the node with the smallest cost function
    #
    
    ####
    print "Path calculated after " + str(steps) + " steps."
    msg_path = Path()
    msg_path.header.frame_id = "map"
    #
    # TODO:
    # Store the resulting path in the 'msg_path' variable
    # Return the appropiate response
    # 
    
    ####
    pub_path = rospy.Publisher('/navigation/path_planning/calculated_path', Path, queue_size=10)
    pub_path.publish(msg_path)
    return CalculatePathResponse(msg_path)

def callback_a_star(req):
    print "Calculating path A-Star" ####
    map = inflate_map(req.map)
    map = get_nearness(map)
    steps = 0
    #
    # TODO:
    # Write a A-star algorithm to find a path between the start position
    # [req.start.pose.position.x, req.start.pose.position.y]
    # and the goal position
    # [req.goal.pose.position.x, req.goal.pose.position.y]
    # Consider nearness and distance traveled as cost function and Manhattan distance to goal as h-value
    # Use the 'steps' variable to store the total steps needed for calculations
    # HINT: Use a heap structure to keep track of the node with the smallest f-value
    #
    
    ####
    print "Path calculated after " + str(steps) + " steps."
    msg_path = Path()
    msg_path.header.frame_id = "map"
    #
    # TODO:
    # Store the resulting path in the 'msg_path' variable
    # Return the appropiate response
    #
    
    ####
    pub_path = rospy.Publisher('/navigation/path_planning/calculated_path', Path, queue_size=10)
    pub_path.publish(msg_path)
    return CalculatePathResponse(msg_path)

def main():
    print "PRACTICE 02 - " + NAME
    rospy.init_node("practice02")
    rospy.Service('/navigation/path_planning/dijkstra_search', CalculatePath, callback_dijkstra)
    rospy.Service('/navigation/path_planning/a_star_search'  , CalculatePath, callback_a_star)
    rospy.wait_for_service('/navigation/localization/static_map')
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
