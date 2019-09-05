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

NAME = "RAMOS_SANCHEZ_SAMUEL"

def callback_bfs(req):
    #este request esta relacionado a CalculatePath, esta ubicada la biblioteca en
    #src/navigation
    print "Calculating path by breadth first search"
    steps = 0
    n = -1
    msg_path = Path()
    open_list = deque()
    map_width = req.map.info.width

    is_in_open_list = [False]*(map_width*req.map.info.height)
    is_in_close_list = [False]*(map_width*req.map.info.height)
    n_values = [sys.maxint]*(map_width*req.map.info.height)
    previous = [-1]*(map_width*req.map.info.height)
    start_x = req.start.pose.position.x
    start_y = req.start.pose.position.y
    goal_x = req.goal.pose.position.x
    goal_y = req.goal.pose.position.y
    start_idx = int((start_y - req.map.info.origin.position.y)/req.map.info.resolution)*map_width
    start_idx += int((start_x - req.map.info.origin.position.x)/req.map.info.resolution)
    goal_idx = int((goal_y - req.map.info.origin.position.y)/req.map.info.resolution)*map_width
    goal_idx += int((goal_x - req.map.info.origin.position.x)/req.map.info.resolution)

    open_list.append(start_idx)
    while(len(open_list) != 0 and n != goal_idx):
        n = open_list.popleft()
        neighbors = [n + map_width, n + 1, n - map_width, n - 1]
        for x in neighbors:
            if not is_in_open_list[x] and not is_in_closed_list[x]:
                open_list.append(x)
                is_in_open_list[x] = True
            if n_values[x] > n_values[n] + 1:
                n_values[x] = n_values[n] + 1
                previous[x] = n
                  
        is_in_closed_list[n] = True

    if n == goal_idx:
        print 'Path found'
        while previous[n] != -1:
            print n
            n = previous[n]
    else: 
        print 'Path not found'
    #Traducir las celdas a valores X,Y     
    # TODO:
    # Write a breadth first search algorithm to find a path between the start position given by
    # [req.start.pose.position.x, req.start.pose.position.y]
    # and the goal position, given by
    # [req.goal.pose.position.x, req.goal.pose.position.y]
    # Use the 'steps' variable to store the total steps needed for calculations
    
    print "Path calculated after " + str(steps) + " steps."
    msg_path.header.frame_id = "map"
    # TODO:
    # Store the resulting path in the 'msg_path' variable (see online documentation for nav_msgs/Path)
    # Return the appropiate response.
    pub_path = rospy.Publisher('/navigation/path_planning/calculated_path', Path, queue_size=10)
    pub_path.publish(msg_path)
    return CalculatePathResponse(msg_path)

def callback_dfs(req):
    print "Calculating path by depth first search" ####
    steps = 0
    msg_path = Path()
    # TODO:
    # Write a depth first search algorithm to find a path between the start position
    # [req.start.pose.position.x, req.start.pose.position.y]
    # and the goal position
    # [req.goal.pose.position.x, req.goal.pose.position.y]
    # Use the 'steps' variable to store the total steps needed for calculations
    
    print "Path calculated after " + str(steps) + " steps."
    msg_path.header.frame_id = "map"
    # TODO:
    # Store the resulting path in the 'msg_path' variable
    # Return the appropiate response.
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
    
