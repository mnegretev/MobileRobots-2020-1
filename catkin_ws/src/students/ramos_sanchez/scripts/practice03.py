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
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from navig_msgs.srv import SmoothPath
from navig_msgs.srv import SmoothPathResponse
from collections import deque

NAME = "RAMOS_SANCHEZ"

def callback_smooth_path(req):
    if rospy.has_param("/navigation/path_planning/smoothing_alpha"):
        alpha = rospy.get_param("/navigation/path_planning/smoothing_alpha")
    else:
        alpha = 0.5
    if rospy.has_param("/navigation/path_planning/smoothing_beta"):
        beta = rospy.get_param("/navigation/path_planning/smoothing_beta")
    else:
        beta = 0.5
    print "Smoothing a " + str(len(req.path.poses)) + " points path with alpha=" + str(alpha) + " and beta=" + str(beta)

    old_path = req.path
    x = old_path.poses[0].pose.position.x
    N = len(old_path.poses)
    smooth_path = Path()
    smooth_path.header.frame_id = "map"
    smooth_path.header.stamp    = rospy.Time.now()
    smooth_path = old_path

    #
    # TODO:
    # Write the algorithm to smooth the path by gradient descend algorith
    # using as cost function the sum of the distance between point in the new path
    # plus the distance bewteen points in the new and old paths.
    # Store the resulting path in the 'smooth_path' variable.
    #

    tol = 0.000001
    gradient_mag = tol + 1
    delta = 0.05
    cycles = 1000

    while gradient_mag > tol and cycles != 0:
        gradx_0 = alpha * (smooth_path.poses[0].pose.position.x - smooth_path.poses[1].pose.position.x)
        gradx_0 += beta * (smooth_path.poses[0].pose.position.x - old_path.poses[0].pose.position.x)
        new_x_0 = delta*gradx_0
        smooth_path.poses[0].pose.position.x -= new_x_0

        grady_0 = alpha * (smooth_path.poses[0].pose.position.y - smooth_path.poses[1].pose.position.y)
        grady_0 += beta * (smooth_path.poses[0].pose.position.y - old_path.poses[0].pose.position.y)
        new_y_0 = delta*grady_0
        smooth_path.poses[0].pose.position.y -= new_y_0

        for i in range(1, N-1):
            gradx_i = alpha * (2*smooth_path.poses[i].pose.position.x - smooth_path.poses[i-1].pose.position.x - smooth_path.poses[i+1].pose.position.x)
            gradx_i += beta * (smooth_path.poses[i].pose.position.x - old_path.poses[i].pose.position.x)
            new_x_i = delta * gradx_i
            smooth_path.poses[i].pose.position.x -= new_x_i

            grady_i = alpha * (2*smooth_path.poses[i].pose.position.y - smooth_path.poses[i-1].pose.position.y - smooth_path.poses[i+1].pose.position.y)
            grady_i += beta * (smooth_path.poses[i].pose.position.y - old_path.poses[i].pose.position.y)
            new_y_i = delta * grady_i
            smooth_path.poses[i].pose.position.y -= new_y_i

            gradient_mag += pow(gradx_i, 2) + pow(grady_i, 2)

        gradx_N = alpha * (smooth_path.poses[N-1].pose.position.x - smooth_path.poses[N-1-1].pose.position.x)
        gradx_N += beta * (smooth_path.poses[N-1].pose.position.x - old_path.poses[N-1].pose.position.x)
        new_x_N = delta * gradx_N
        smooth_path.poses[N-1].pose.position.x -= new_x_N

        grady_N = alpha * (smooth_path.poses[N-1].pose.position.y - smooth_path.poses[N-1-1].pose.position.y)
        grady_N += beta * (smooth_path.poses[N-1].pose.position.y - old_path.poses[N-1].pose.position.y)
        new_y_N = delta * grady_N
        smooth_path.poses[N-1].pose.position.y -= new_y_N

        gradient_mag += pow(gradx_0, 2) + pow(gradx_N, 2) + pow(grady_0, 2) + pow(grady_N, 2)
        gradient_mag = math.sqrt(gradient_mag)
        cycles -= 1

####
    #print "Path smoothed after " + str(attempts) + " steps"
    pub_path = rospy.Publisher('/navigation/path_planning/smooth_path', Path, queue_size=10)
    pub_path.publish(smooth_path)
    return SmoothPathResponse(smooth_path)

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
    
