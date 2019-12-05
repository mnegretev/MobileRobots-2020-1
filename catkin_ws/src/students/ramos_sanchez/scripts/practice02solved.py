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
# Consider the map as a bidimensional array with free and occupied cells:
# [[ 0 0 0 0 0 0]
#  [ 0 X 0 0 0 0]
#  [ 0 X X 0 0 0]
#  [ 0 X X 0 0 0]
#  [ 0 X 0 0 0 0]
#  [ 0 0 0 X 0 0]]
# Where occupied cells 'X' have a value of 100 and free cells have a value of 0.
# In this example map[1][1] has a value of 100 and map[1][2] has a value of 0.
#
# Consider the nearness map as a bidimensional array where free cells have a value indicating
# how near they are to the nearest occupied cell:
# [[ 3 3 3 2 2 1]
#  [ 3 X 3 3 2 1]
#  [ 3 X X 3 2 1]
#  [ 3 X X 3 2 2]
#  [ 3 X 3 3 3 2]
#  [ 3 3 3 X 3 2]]
    

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
from std_msgs.msg import String

NAME = "APELLIDO_PATERNO_APELLIDO_MATERNO"

def Dijkstra(map, nearness_map, start, goal):
    print "Calculating Dijkstra from " + str(start) + " to " + str(goal)
    #
    # TODO:
    # Implement the Dijktra algorithm given a map, nearness map, start and goal positions.
    # The Dijkstra algorithm can be performed with the following steps:
    #

    # Variables:
    # steps          = These variable is just to get an approximate of the computational cost.
    # open_list      = The open list will be used later as a priority queue.                  
    # g_values       = We need a set of g_values for all cells, initialy in infinity          
    # in_open_list   = A set of booleans indicating whether a node is in open list, or not.   
    # in_closed_list = A set of booleans indicating whether a node is in closed list, or not.
    # parents        = A set of parent nodes for all cells, all being None at the beginning
    steps         = 0  
    open_list     = []
    g_values      = [[sys.maxint for j in range(len(map[0]))] for i in range(len(map))]
    in_open_list  = [[False      for j in range(len(map[0]))] for i in range(len(map))]
    in_closed_list= [[False      for j in range(len(map[0]))] for i in range(len(map))]
    parents       = [[None       for j in range(len(map[0]))] for i in range(len(map))]
            
    [current_r, current_c]             = start #Current node is a pair of coordinates [row,col]
    heapq.heappush(open_list, (0, start))      #Open list is a priority queue, with g_value, the weighting value
    in_open_list[current_r][current_c] = True  #We start by adding the start node into the open list
    g_values    [current_r][current_c] = 0     #The g_value of the start node is set to zero. 

    
    #While the open list is not empty and we haven't reached the goal position:
    while len(open_list) > 0 and [current_r, current_c] != goal:
        #We choose the next node to expand from the open lits (the node with the smallest g-value),
        #we mark that node as part of the closed list
        #and we expand the node (pick the neighbors) using 4-connectivity
        [current_r, current_c]               = heapq.heappop(open_list)[1]
        in_closed_list[current_r][current_c] = True
        neighbors = [[current_r+1, current_c], [current_r-1, current_c], [current_r, current_c+1], [current_r, current_c-1]]
        for [r, c] in neighbors:
            if map[r][c] != 0 or in_closed_list[r][c]: #If the node is not free or is already in the close list, we skip it.
                continue
            #The new g-value for the neighbor [r,c] is the sum of the g-value of the current node
            #(accumulated distance) and the cost of going from current node to neighbor [r,c] (1 + nearness)
            g = g_values[current_r][current_c] + 1 + nearness_map[r][c] 
            if g < g_values[r][c]:                          #If the new g value is less than the already set
                g_values[r][c] = g                          #Then we change it and set the current node
                parents [r][c] = [current_r, current_c]     #as the parent of the neighbor being checked
            if not in_open_list[r][c]:                      
                in_open_list[r][c] = True                   #If it is not already in the open list, we add the neighbor
                heapq.heappush(open_list, (g, [r,c]))       #to the open list. Remember we are using a heap structure 
                                                            #to keep nodes sorted by g-values.
        steps += 1

    if [current_r, current_c] != goal:
        print "Cannot calculate path :'("
        return None
    path = []
    while parents[current_r][current_c] != None:                #Once we reached the goal point, we 
        path.insert(0, [current_r, current_c])                  #We build the path by finding the previous nodes 
        [current_r, current_c] = parents[current_r][current_c]  #until we reach the start node (with parent = None)
    path.insert(0, [current_r, current_c])
    print "Path calculated with " + str(len(path)) + " points after " +  str(steps) + " steps"
    return path
    

def a_star(map, nearness_map, start, goal):
    print "Calculating A* from " + str(start) + " to " + str(goal)
    #
    # TODO:
    # Implement the A* algorithm given a map, nearness map, start and goal positions.
    # The A* algorithm can be implement with the following steps. 

    # Variables:
    # steps          = These variable is just to get an approximate of the computational cost.
    # open_list      = The open list will be used later as a priority queue.                  
    # g_values       = We need a set of g_values for all cells, initialy in infinity
    # f_values       = Also we need a set of f-values for all cells, initially in infinity
    # in_open_list   = A set of booleans indicating whether a node is in open list, or not.   
    # in_closed_list = A set of booleans indicating whether a node is in closed list, or not.
    # parents        = A set of parent nodes for all cells, all being None at the beginning
    steps         = 0  
    open_list     = []
    g_values      = [[sys.maxint for j in range(len(map[0]))] for i in range(len(map))]
    f_values      = [[sys.maxint for j in range(len(map[0]))] for i in range(len(map))] ####
    in_open_list  = [[False      for j in range(len(map[0]))] for i in range(len(map))]
    in_closed_list= [[False      for j in range(len(map[0]))] for i in range(len(map))]
    parents       = [[None       for j in range(len(map[0]))] for i in range(len(map))]
            
    [current_r, current_c]             = start #Current node is a pair of coordinates [row,col]
    [goal_r   , goal_c   ]             = goal  #Goal node is also a pair of coordinates [row, col]   ###
    heapq.heappush(open_list, (0, start))      #Open list is a priority queue, with g_value, the weighting value
    in_open_list[current_r][current_c] = True  #We start by adding the start node into the open list
    g_values    [current_r][current_c] = 0     #The g_value of the start node is set to zero. 

    
    #While the open list is not empty and we haven't reached the goal position:
    while len(open_list) > 0 and [current_r, current_c] != goal:
        #We choose the next node to expand from the open lits (the node with the smallest g-value),
        #we mark that node as part of the closed list
        #and we expand the node (pick the neighbors) using 4-connectivity
        [current_r, current_c]               = heapq.heappop(open_list)[1]
        in_closed_list[current_r][current_c] = True
        neighbors = [[current_r+1, current_c], [current_r-1, current_c], [current_r, current_c+1], [current_r, current_c-1]]
        for [r, c] in neighbors:
            if map[r][c] != 0 or in_closed_list[r][c]: #If the node is not free or is already in the close list, we skip it.
                continue
            #The new g-value for the neighbor [r,c] is the sum of the g-value of the current node
            #(accumulated distance) and the cost of going from current node to neighbor [r,c] (1 + nearness)
            g = g_values[current_r][current_c] + 1 + nearness_map[r][c]
            h = abs(r - goal_r) + abs(c - goal_c)           #Heuristic is the Manhattan distance to goal       ###
            f = g + h                                       #f-value for neighbor [r,c] is f+h                 ###
            if g < g_values[r][c]:                          #If the new g value is less than the already set
                g_values[r][c] = g                          #Then we change the g-value and 
                f_values[r][c] = f                          #the f-value and set the current node              ###
                parents [r][c] = [current_r, current_c]     #as the parent of the neighbor being checked
            if not in_open_list[r][c]:                      
                in_open_list[r][c] = True                   #If it is not already in the open list, we add the neighbor
                heapq.heappush(open_list, (f, [r,c]))       #to the open list. Remember we are using a heap structure 
                                                            #to keep nodes sorted by g-values.                 ###
        steps += 1

    if [current_r, current_c] != goal:
        print "Cannot calculate path :'("
        return None
    path = []
    while parents[current_r][current_c] != None:                #Once we reached the goal point, we 
        path.insert(0, [current_r, current_c])                  #We build the path by finding the previous nodes 
        [current_r, current_c] = parents[current_r][current_c]  #until we reach the start node (with parent = None)
    path.insert(0, [current_r, current_c])
    print "Path calculated with " + str(len(path)) + " points after " +  str(steps) + " steps"
    return path

def inflate_map(map, size):
    print "Inflating map by " + str(size) + " cells"
    inflated = copy.deepcopy(map)
    for i in range(len(map)):
        for j in range(len(map[0])):
            inflated[i][j] = map[i][j]
            if map[i][j] == 100:
                for k1 in range(-size, size+1):
                    for k2 in range(-size, size+1):
                        inflated[i+k1][j+k2] = 100
    return inflated
                        
def get_nearness(map, size):
    print "Calculating nearness map with " +str(size) + " cells"
    nearness_map = copy.deepcopy(map)
    if size > 10:
        size = 10
    for i in range(len(map)):
        for j in range(len(map[0])):
            if map[i][j] == 100:
                for k1 in range(-size, size+1):
                    for k2 in range(-size, size+1):
                        nearness = size - max(abs(k1),abs(k2)) + 1
                        nearness_map[i+k1][j+k2] = max(nearness, nearness_map[i+k1][j+k2])
    return nearness_map

def callback_generic(req, algorithm):
    global strx, stry, stra
    print "Calculating path by " + algorithm + " search"
    if rospy.has_param("/navigation/path_planning/inflation_radius"):
        inflation_radius = float(rospy.get_param("/navigation/path_planning/inflation_radius"))
    else:
        inflation_radius = 1.0
    if rospy.has_param("/navigation/path_planning/nearness_radius"):
        nearness_radius = float(rospy.get_param("/navigation/path_planning/nearness_radius"))
    else:
        nearness_radius = 1.0

    map = [[0 for j in range(req.map.info.width)] for i in range(req.map.info.height)]
    for i in range(req.map.info.height):
        for j in range(req.map.info.width):
            map[i][j] = req.map.data[i*req.map.info.width + j]
            
    inflated_map = inflate_map(map, int(inflation_radius/req.map.info.resolution))
    nearness_map = get_nearness(inflated_map, int(nearness_radius/req.map.info.resolution))
    
    start_x = int((strx - req.map.info.origin.position.x)/req.map.info.resolution)
    start_y = int((stry - req.map.info.origin.position.y)/req.map.info.resolution)
    goal_x  = int((req.goal.pose.position.x  - req.map.info.origin.position.x)/req.map.info.resolution)
    goal_y  = int((req.goal.pose.position.y  - req.map.info.origin.position.y)/req.map.info.resolution)

    if algorithm == "Dijkstra":
        path = Dijkstra(inflated_map, nearness_map, [start_y, start_x], [goal_y, goal_x])
    elif algorithm == "A*":
        path = a_star(inflated_map, nearness_map, [start_y, start_x], [goal_y, goal_x])
    else:
        print "Invalid algorithm name"
        return None
    
    if path == None:
        return None

    msg_path = Path()
    msg_path.header.frame_id = "map"
    for [r,c] in path:
        p = PoseStamped()
        p.pose.position.x = c*req.map.info.resolution + req.map.info.origin.position.x
        p.pose.position.y = r*req.map.info.resolution + req.map.info.origin.position.y
        msg_path.poses.append(p)
    pub_path = rospy.Publisher('/navigation/path_planning/calculated_path', Path, queue_size=10)
    pub_path.publish(msg_path)
    return CalculatePathResponse(msg_path)

def callback_a_star(req):
    return callback_generic(req, "A*")

def callback_dijkstra(req):
    return callback_generic(req, "Dijkstra")

def start_callback(msg):
	global strx, stry, stra
	message = msg.data
	start_position = message.split(",")
	strx = float(start_position[0])
	stry = float(start_position[1])
	stra = float(start_position[2])

def main():
    print "PRACTICE 02 - " + NAME
    rospy.init_node("practice02")
    rospy.Service('/navigation/path_planning/dijkstra_search', CalculatePath, callback_dijkstra)
    rospy.Service('/navigation/path_planning/a_star_search'  , CalculatePath, callback_a_star)
    rospy.Subscriber("/navigation/localization", String, start_callback)
    rospy.wait_for_service('/navigation/localization/static_map')
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
