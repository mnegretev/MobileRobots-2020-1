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

NAME = "AVALOS_GARCIA"

def callback_bfs(req):
    print "Calculating path by breadth first search"
    steps = 0
    #
    # TODO:
    # Write a breadth first search algorithm to find a path between the start position
    # [req.start.pose.position.x, req.start.pose.position.y]
    # and the goal position
    # [req.goal.pose.position.x, req.goal.pose.position.y]
    # Use the 'steps' variable to store the total steps needed for calculations
    #
    start_idx  = int((req.start.pose.position.x - req.map.info.origin.position.x)/req.map.info.resolution)
    start_idx += int((req.start.pose.position.y - req.map.info.origin.position.y)/req.map.info.resolution)*req.map.info.width
    goal_idx   = int((req.goal.pose.position.x  - req.map.info.origin.position.x)/req.map.info.resolution)
    goal_idx  += int((req.goal.pose.position.y  - req.map.info.origin.position.y)/req.map.info.resolution)*req.map.info.width
    
    open_list      = deque()
    in_open_list   = [False]*len(req.map.data)
    in_closed_list = [False]*len(req.map.data)
    distances      = [sys.maxint]*len(req.map.data)
    parent_nodes   = [-1]*len(req.map.data)
    
    current = start_idx
    open_list.append(start_idx)
    in_open_list[start_idx] = True
    distances[start_idx]    = 0

    while len(open_list) > 0 and current != goal_idx:
        current = open_list.popleft()
        in_closed_list[current] = True
        neighbors = [current + req.map.info.width, current - req.map.info.width, current + 1, current - 1]
        dist = distances[current] + 1
        for n in neighbors:
            if req.map.data[n] > 40 or req.map.data[n] < 0 or in_closed_list[n]:
                continue
            if dist < distances[n]:
                distances[n]    = dist
                parent_nodes[n] = current
            if not in_open_list[n]:
                in_open_list[n] = True
                open_list.append(n)
            steps += 1

    if current != goal_idx:
        print "Cannot calculate path :'("
        return None
    ####
    print "Path calculated after " + str(steps) + " steps."
    msg_path = Path()
    msg_path.header.frame_id = "map"
    #
    # TODO:
    # Store the resulting path in the 'msg_path' variable
    # Return the appropiate response.
    #
    while parent_nodes[current] != -1:
        p = PoseStamped()
        p.pose.position.x = (current%req.map.info.width)*req.map.info.resolution + req.map.info.origin.position.x
        p.pose.position.y = (current/req.map.info.width)*req.map.info.resolution + req.map.info.origin.position.y
        msg_path.poses.insert(0,p)
        current = parent_nodes[current]
    ####
    pub_path = rospy.Publisher('/navigation/path_planning/calculated_path', Path, queue_size=10)
    pub_path.publish(msg_path)
    return CalculatePathResponse(msg_path)

def callback_dfs(req):
    print "Calculating path by depth first search" ####
    steps = 0
    #
    # TODO:
    # Write a depth first search algorithm to find a path between the start position
    # [req.start.pose.position.x, req.start.pose.position.y]
    # and the goal position
    # [req.goal.pose.position.x, req.goal.pose.position.y]
    # Use the 'steps' variable to store the total steps needed for calculations
    #
    
    ####
    print "Path calculated after " + str(steps) + " steps."
    msg_path = Path()
    msg_path.header.frame_id = "map"
    #
    # TODO:
    # Store the resulting path in the 'msg_path' variable
    # Return the appropiate response.
    #
    
    ####
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
    
