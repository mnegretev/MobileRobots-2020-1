#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2020-1
# PRACTICE 3 - PATH SMOOTHING BY GRADIENT DESCEND
#
# Instructions:
# Write the code necessary to smooth a path given by a sequence of points
# using a gradient descend algorithm.
#

import sys
import rospy
import copy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from navig_msgs.srv import SmoothPath
from navig_msgs.srv import SmoothPathResponse
from collections import deque

NAME = "APELLIDO_PATERNO_APELLIDO_MATERNO"

def get_smooth_path(original_path, alpha, beta):
    #
    # TODO:
    # Write an algorithm to smooth the 'original_path' and return a new path
    # Path is composed of a set of points [x,y]. The following line
    # [xo_i,yo_i] = original_path[i]
    # stores the x,y coordinates of the i-th point of the original path
    # in the variables xo_i and yo_i respectively. 
    # The algorithm for smoothing the path is as follows:
    #
    #
    smooth_path  = copy.deepcopy(original_path)            # At the beginnig, the smooth path is the same than the original path.
    tolerance    = 0.00001                                 # If gradient magnitude is less than a tolerance, we consider.
    gradient_mag = tolerance + 1                           # we have reached the local minimum.
    gradient     = [[0,0] for i in range(len(smooth_path))]# Gradient has N components of the form [x,y]. 
    epsilon      = 0.5                                     # This variable will weight the calculated gradient.

    while gradient_mag > tolerance:
        #First we calculate all components of gradient (except first and last,
        #since first and last points (start and goal) remain the same).
        for i in range(1, len(original_path)-1):
            [xo_i, yo_i]   = original_path[i] # i-th point of the original path
            [xn_i, yn_i]   = smooth_path  [i] # i-th point of the smooth path
            [xn_in, yn_in] = smooth_path[i+1] # (i+1)-th point of the smooth path
            [xn_ip, yn_ip] = smooth_path[i-1] # (i-1)-th point of the smooth path
            grad_x = beta*(xn_i - xo_i) + alpha*(2*xn_i - xn_ip - xn_in) #X-value of the i-th component of the gradient
	    grad_y = beta*(yn_i - yo_i) + alpha*(2*yn_i - yn_ip - yn_in) #Y-value of the i-th component of the gradient
            gradient[i] = [grad_x, grad_y]
        #Once we have calculated the gradient, we move variables
        #in the opposite direction of the gradient:
        for i in range(len(smooth_path)):
            smooth_path[i][0] -= epsilon*gradient[i][0]
            smooth_path[i][1] -= epsilon*gradient[i][1]
        #Finaly, we calculate the magnitude of the gradient to check if we have already reached the local minimum:
        gradient_mag = 0
        for i in range(len(gradient)):
            gradient_mag += abs(gradient[i][0]) + abs(gradient[i][1]) #We are using the norm-1
    return smooth_path

def callback_smooth_path(req):
    if rospy.has_param("/navigation/path_planning/smoothing_alpha"):
        alpha = rospy.get_param("/navigation/path_planning/smoothing_alpha")
    else:
        alpha = 0.5
    if rospy.has_param("/navigation/path_planning/smoothing_beta"):
        beta = rospy.get_param("/navigation/path_planning/smoothing_beta")
    else:
        beta = 0.5

    path = []
    for i in range(len(req.path.poses)):
        x = req.path.poses[i].pose.position.x
        y = req.path.poses[i].pose.position.y
        path.append([x,y])

    print "Smoothing a " + str(len(req.path.poses)) + " points path with alpha=" + str(alpha) + " and beta=" + str(beta)
    smooth_path = get_smooth_path(path, alpha, beta)
    print "Path smoothed"
    msg_smooth_path = Path()
    msg_smooth_path.header.frame_id = "map"
    for i in range(len(path)):
        p = PoseStamped()
        p.pose.position.x = smooth_path[i][0]
        p.pose.position.y = smooth_path[i][1]
        msg_smooth_path.poses.append(p)
        
    msg_smooth_path.header.stamp    = rospy.Time.now()
    pub_path = rospy.Publisher('/navigation/path_planning/smooth_path', Path, queue_size=10)
    pub_path.publish(msg_smooth_path)
    return SmoothPathResponse(msg_smooth_path)

def main():
    print "PRACTICE 03 - " + NAME
    rospy.init_node("practice03")
    rospy.Service('/navigation/path_planning/smooth_path', SmoothPath, callback_smooth_path)
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
