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
import heapq    #Arbol para cola dinamica
from collections import deque
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from navig_msgs.srv import CalculatePath
from navig_msgs.srv import CalculatePathResponse

NAME = "Altamirano_Leal"


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

    #declaramos las variables Ni
    start_idx  = int((req.start.pose.position.x - req.map.info.origin.position.x)/req.map.info.resolution)
    start_idx += int((req.start.pose.position.y - req.map.info.origin.position.y)/req.map.info.resolution)*req.map.info.width
    goal_idx   = int((req.goal.pose.position.x  - req.map.info.origin.position.x)/req.map.info.resolution)
    goal_idx  += int((req.goal.pose.position.y  - req.map.info.origin.position.y)/req.map.info.resolution)*req.map.info.width
    
    
    #declaramos un areglo open_list
    open_list = []
    #Combertimos el areglo en un arbol
    heapq.heapify(open_list)    

    #Declaramos areglos para ir almacenando los puntos del mapa que ya an salido de la cola 
    in_open_list   = [False]*len(req.map.data)
    in_closed_list = [False]*len(req.map.data)
    distances      = [sys.maxint]*len(req.map.data)
    parent_nodes   = [-1]*len(req.map.data)
    
    #Agregamos una celda del mapa a la cola
    current = start_idx
    heapq.heappush(open_list,start_idx)
    in_open_list[start_idx] = True
    distances[start_idx]    = 0

    #Algoritmo de busqueda 

    #Mientdas la lenta sea mayor a 0 y nuestro nodo actual sea distinto de destino (goal)
    while len(open_list) > 0 and current != goal_idx:


        current = heapq.heappop(open_list)  
        in_closed_list[current] = True       # colocamos el elemento minimo en la lista cerrada
        # declaramos los vecinos (neighbors)
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
                heapq.heappush(open_list,n) 
            steps += 1

    if current != goal_idx:
        print "Cannot caaalculate path :'("
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

#declaramos las variables Ni
    start_idx  = int((req.start.pose.position.x - req.map.info.origin.position.x)/req.map.info.resolution)
    start_idx += int((req.start.pose.position.y - req.map.info.origin.position.y)/req.map.info.resolution)*req.map.info.width
    goal_idx   = int((req.goal.pose.position.x  - req.map.info.origin.position.x)/req.map.info.resolution)
    goal_idx  += int((req.goal.pose.position.y  - req.map.info.origin.position.y)/req.map.info.resolution)*req.map.info.width
    
    
    #declaramos un areglo open_list
    open_list = []
    #Combertimos el areglo en un arbol
    heapq.heapify(open_list)    

    #Declaramos areglos para ir almacenando los puntos del mapa que ya an salido de la cola 
    in_open_list   = [False]*len(req.map.data)
    in_closed_list = [False]*len(req.map.data)
    distances      = [sys.maxint]*len(req.map.data)
    parent_nodes   = [-1]*len(req.map.data)
    
    #Agregamos una celda del mapa a la cola
    current = start_idx
    heapq.heappush(open_list,start_idx)
    in_open_list[start_idx] = True
    distances[start_idx]    = 0

    #Algoritmo de busqueda 

    #Mientdas la lenta sea mayor a 0 y nuestro nodo actual sea distinto de destino (goal)
    while len(open_list) > 0 and current != goal_idx:


        current = heapq.heappop(open_list)  
        in_closed_list[current] = True       # colocamos el elemento minimo en la lista cerrada
        # declaramos los vecinos (neighbors)
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
                heapq.heappush(open_list,n) 
            steps += 1

    if current != goal_idx:
        print "Cannot caaalculate path :'("
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
    
