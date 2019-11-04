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

NAME = "Moreno Madrid Maria Guadalupe"

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
    n = int(radius/map.info.resolution)
    inflated.info = copy.deepcopy(map.info)
    for i in range(len(map.data)):
        inflated.data.append(0)

    for i in range(len(map.data)):
        if map.data[i] == -1:
            inflated.data[i] = -1
        if map.data[i] == 100:
            for j in range(-n, n+1):
                for m in range(-n, n+1):
                    inflated.data[(j * inflated.info.width) + m +i] = 100
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
    n = int(radius/map.info.resolution)
    nearness.info = copy.deepcopy(map.info)
    width = nearness.info.width
    counter = 1 - n

    for i in range(len(map.data)):
        nearness.data.append(1)
        if map.data[i] == -1:
            nearness.data[i] = -1
        elif map.data[i] == 100:
            nearness.data[i] = 100

    for i in range(len(map.data)):
        if nearness.data[i] == 100:
            for j in range(-n, n + 1):
                counter = +1
                for m in range(-n, n + 1):
                    if nearness.data[(j * width) + m + i] == 0:
                        nearness.data[(j * width) + m + i] = n - abs(counter)
            counter = 1 - n
    return nearness

def callback_dijkstra(req):
    print "Calculating path by Dijkstra search"
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
    
    start_idx  = int((req.start.pose.position.x - map.info.origin.position.x)/map.info.resolution)
    start_idx += int((req.start.pose.position.y - map.info.origin.position.y)/map.info.resolution)*map.info.width
    goal_idx   = int((req.goal.pose.position.x  - map.info.origin.position.x)/map.info.resolution)
    goal_idx  += int((req.goal.pose.position.y  - map.info.origin.position.y)/map.info.resolution)*map.info.width

    open_list = []
    in_open_list   = [False]*len(map.data)
    in_closed_list = [False]*len(map.data)
    distances      = [sys.maxint]*len(map.data)
    parent_nodes   = [-1]*len(map.data)

    current_index = start_idx
    in_open_list[current_index] = True
    distances[current_index] = 0
    heapq.heappush(open_list, (distances[current_index], current_index))

    while len(open_list) != 0 and current_index != goal_idx:
        current_index = heapq.heappop(open_list)[1]
        in_closed_list[current_index] = True
        in_open_list[current_index] = False
        neighbors = [current_index + map.info.width, current_index - map.info.width, current_index + 1, current_index - 1]

        for n in neighbors:
            if map.data[n] > 40 or map.data[n] < 0 or in_closed_list[n]:
                continue
            dist = distances[current_index] + 1 + map.data[n]
            if dist < distances[n]:
                distances[n]    = dist
                parent_nodes[n] = current_index
            if not in_open_list[n]:
                in_open_list[n] = True
                heapq.heappush(open_list, (dist, n))
        steps += 1

    if current_index != goal_idx:
        print "Cannot calculate path :'("
        return None

    print "Path calculated after " + str(steps) + " steps."
    msg_path = Path()
    msg_path.header.frame_id = "map"
    #
    # TODO:
    # Store the resulting path in the 'msg_path' variable
    # Return the appropiate response
    # 
    while parent_nodes[current_index] != -1:
        p = PoseStamped()
        p.pose.position.x = (current_index%map.info.width)*map.info.resolution + map.info.origin.position.x
        p.pose.position.y = (current_index/map.info.width)*map.info.resolution + map.info.origin.position.y
        msg_path.poses.insert(0,p)
        current_index = parent_nodes[current_index]
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
    start_idx  = int((req.start.pose.position.x - map.info.origin.position.x)/map.info.resolution)
    start_idx += int((req.start.pose.position.y - map.info.origin.position.y)/map.info.resolution)*map.info.width
    goal_idx   = int((req.goal.pose.position.x  - map.info.origin.position.x)/map.info.resolution)
    goal_idx  += int((req.goal.pose.position.y  - map.info.origin.position.y)/map.info.resolution)*map.info.width

    open_list = []
    in_open_list   = [False]*len(map.data)
    in_closed_list = [False]*len(map.data)
    g              = [sys.maxint] * len(map.data)
    parent_nodes   = [-1]*len(map.data)

    current = start_idx
    in_open_list[current] = True
    g[current]    = 0
    heapq.heappush(open_list, (g[current], current))

    while len(open_list) != 0 and current != goal_idx:
        current = heapq.heappop(open_list)[1]
        in_closed_list[current] = True
        in_open_list[current] = False
        neighbors = [current + map.info.width, current - map.info.width, current + 1, current - 1]
        for n in neighbors:
            if map.data[n] > 40 or map.data[n] < 0 or in_closed_list[n]:
                continue
            dist = g[current] + 1 + int(map.data[n])
            f_value = sys.maxint
            if dist < g[n]:
                h  = abs(n%map.info.width - goal_idx%map.info.width);
		h += abs(n/map.info.width - goal_idx/map.info.width);
                g[n]    = dist
                parent_nodes[n] = current
                f_value = dist + h
            if not in_open_list[n]:
                in_open_list[n] = True
                heapq.heappush(open_list, (f_value, n))
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
    # Return the appropiate response
    #
    while parent_nodes[current] != -1:
        p = PoseStamped()
        p.pose.position.x = (current%map.info.width)*map.info.resolution + map.info.origin.position.x
        p.pose.position.y = (current/map.info.width)*map.info.resolution + map.info.origin.position.y
        msg_path.poses.insert(0,p)
        current = parent_nodes[current]
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
    
    
