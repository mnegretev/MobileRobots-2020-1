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

NAME = "Alvarado_Paz"

def inflate_map(map):
    if rospy.has_param("/navigation/path_planning/inflation_radius"):
        radius = float(rospy.get_param("/navigation/path_planning/inflation_radius"))
    else:
        radius = 1.0        
    print "Inflating map by " + str(radius) + " meters"
    inflated = OccupancyGrid()
    inflation_cells = (radius / map.info.resolution);
    # Me falto implementar los 2 fors para iflar los obstaculos
    # y otro para devolver el mapa nuevo con los obstaculos inflados

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
    steps = (radius/map.info.resolution)
    boxSize = (steps*2 + 1)*(steps*2 + 1)
    distances = boxSize
    neighbors = boxSize
    counter = 0
    i = -steps
    j = -steps
    for i in steps:
        for j in steps:
            neighbors[counter] = i*map.info.width + j
            distances[counter] = (steps - max(i, abs(j)) + 1)
            counter += 1

    a = 0
    for a in [sys.maxint]*len(map.data[a]):
        if map.data[i] > 40:
            b = 0
            for b in boxSize:
                if (nearness.data[i + neighbors[j]] < distances[j]):
                    nearness.data[i + neighbors[j]] = distances[j]
    
    #
    # TODO:
    # Get a map where all free cells within a 'radius' distance from an obstacle
    # contain a number indicating the distance to such obstacles
    # Store the resulting map in 'nearness'
    #
    # Obtenga un mapa donde todas las celdas libres dentro de una distancia de
    # 'radio' de un obstaculo
    # contenga un numero que indique la distancia a tales obstaculos
    # Almacene el mapa resultante en 'proximidad'
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
    start_idx  = int((req.start.pose.position.x - map.info.origin.position.x)/map.info.resolution)
    start_idx += int((req.start.pose.position.y - map.info.origin.position.y)/map.info.resolution)*map.info.width
    goal_idx   = int((req.goal.pose.position.x  - map.info.origin.position.x)/map.info.resolution)
    goal_idx  += int((req.goal.pose.position.y  - map.info.origin.position.y)/map.info.resolution)*map.info.width
    
    open_list      = []###
    in_open_list   = [False]*len(map.data)
    in_closed_list = [False]*len(map.data)
    distances      = [sys.maxint]*len(map.data)
    parent_nodes   = [-1]*len(map.data)
    
    current = start_idx
    heapq.heappush(open_list, (0, start_idx))##
    in_open_list[start_idx] = True
    distances[start_idx]    = 0

    while len(open_list) > 0 and current != goal_idx:
        current = heapq.heappop(open_list)[1]##
        in_closed_list[current] = True
        neighbors = [current + map.info.width, current - map.info.width, current + 1, current - 1]
        for n in neighbors:
            if map.data[n] > 40 or map.data[n] < 0 or in_closed_list[n]:
                continue
            dist = distances[current] + 1 + map.data[n]##
            if dist < distances[n]:
                distances[n]    = dist
                parent_nodes[n] = current
            if not in_open_list[n]:
                in_open_list[n] = True
                heapq.heappush(open_list, (dist, n))###
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
    
    open_list      = []#####
    in_open_list   = [False]*len(map.data)
    in_closed_list = [False]*len(map.data)
    distances      = [sys.maxint]*len(map.data)
    parent_nodes   = [-1]*len(map.data)
    
    current = start_idx
    heapq.heappush(open_list, (0, start_idx))##
    in_open_list[start_idx] = True
    distances[start_idx]    = 0

    while len(open_list) > 0 and current != goal_idx:
        current = heapq.heappop(open_list)[1]##
        in_closed_list[current] = True
        neighbors = [current + map.info.width, current - map.info.width, current + 1, current - 1]
        for n in neighbors:
            if map.data[n] > 40 or map.data[n] < 0 or in_closed_list[n]:
                continue
            dist = distances[current] + 1 + int(map.data[n]/5.0)##
            f_value = sys.maxint
            if dist < distances[n]:
                h  = abs(n%map.info.width - goal_idx%map.info.width);
		h += abs(n/map.info.width - goal_idx/map.info.width);
                distances[n]    = dist
                parent_nodes[n] = current
                f_value = dist + h
            if not in_open_list[n]:
                in_open_list[n] = True
                heapq.heappush(open_list, (f_value, n))###
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
    
