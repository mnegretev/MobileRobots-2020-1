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
import math

NAME = "CRUZ_NAVARRETE"

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
    
    start_idx  = int((req.start.pose.position.x - req.map.info.origin.position.x)/req.map.info.resolution)
    start_idx += int((req.start.pose.position.y - req.map.info.origin.position.y)/req.map.info.resolution)*req.map.info.width
    goal_idx   = int((req.goal.pose.position.x  - req.map.info.origin.position.x)/req.map.info.resolution)
    goal_idx  += int((req.goal.pose.position.y  - req.map.info.origin.position.y)/req.map.info.resolution)*req.map.info.width

    open_list      = []
    #open_list = deque()
    in_open_list   = [False]*len(req.map.data)
    in_closed_list = [False]*len(req.map.data)
    distances      = [sys.maxint]*len(req.map.data)
    parent_nodes   = [-1]*len(req.map.data)
    
    current = start_idx
    heapq.heappush(open_list,start_idx) #aqui agregamos el nodo inicial a la lista abierta, se ordema de modo que le menor número queda la inicio de la cola
    #open_list.append(start_idx)
    in_open_list[start_idx] = True
    distances[start_idx]    = 0
    
    while len(open_list) > 0 and current != goal_idx:
        current = heapq.heappop(open_list) #quitamos al elemento de menor n
        #current = open_list.popleft() 
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
                heapq.heappush(open_list,n)
                #open_list.append(n)
            steps += 1

    if current != goal_idx:
        print ("Cannot calculate path :'(")
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
    
    ####
    pub_path = rospy.Publisher('/navigation/path_planning/calculated_path', Path, queue_size=10)
    pub_path.publish(msg_path)
    return CalculatePathResponse(msg_path)




'''
ALGORITMO A*

Se quiere calcular la mejor ruta, esta es la de menor costo, es decir
la de la distancia más corta entre el inicio y el punto meta.

Cada nodo tiene una distancia con respecto a otro, este es el costo.

HEURISTICA --> Distancia que hay entre uno de los nodos al destino.

g --> es el costo del nodo anterior 
h --> heuristica
'''


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
    
    
    start_idx  = int((req.start.pose.position.x - req.map.info.origin.position.x)/req.map.info.resolution)
    start_idx += int((req.start.pose.position.y - req.map.info.origin.position.y)/req.map.info.resolution)*req.map.info.width
    goal_idx   = int((req.goal.pose.position.x  - req.map.info.origin.position.x)/req.map.info.resolution)
    goal_idx  += int((req.goal.pose.position.y  - req.map.info.origin.position.y)/req.map.info.resolution)*req.map.info.width

    open_list      = []
    #open_list = deque()
    in_open_list   = [False]*len(req.map.data)
    in_closed_list = [False]*len(req.map.data)
    distances      = [sys.maxint]*len(req.map.data)
    parent_nodes   = [-1]*len(req.map.data)
    
    current = start_idx
    heapq.heappush(open_list,start_idx) #aqui agragamos el nodo inicial a la lista abierta
    #open_list.append(start_idx)
    in_open_list[start_idx] = True
    distances[start_idx]    = 0
    
    while len(open_list) > 0 and current != goal_idx:
        current = heapq.heappop(open_list) #quitamos al elemento de menor f
        #current = open_list.popleft() 
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
                heuristic = abs(start_idx - goal_idx)
                heapq.heappush(open_list,n + heuristic)
                #open_list.append(n)
            steps += 1

    if current != goal_idx:
        print ("Cannot calculate path :'(")
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
    


'''
El algiritmo de la cola heap también se conoce como el algoritmo de 
la cola con prioridad.

heaps son arboles binarios en el cual cada nodo padre tiene un valor 
menor o igual que cualquiera de sus hijos.

Una propiedad interesante de una cola heap es que su elemento mas 
pequeño es siempre la raiz, heap[0].

El método POP regresa el elemento más pequeño, se le llama "min heap"

heap.sort() --> mantiene a la cola heap invariante.

Para crear una cola heap debemos usar una lista inicializada en []
o podemos transformar una lista en una cola heap con la función 
heapify()

heapq.heappush(list,item) --> con este método agregamos elementos a la
a la cola heap, esta cola siempre pondrá el elemento más pequeño al 
principio, esto lo hace por default.
'''
