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
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from navig_msgs.srv import SmoothPath
from navig_msgs.srv import SmoothPathResponse
from collections import deque

NAME = "ramirez_ancona"

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

    smooth_path = Path()
    smooth_path.header.frame_id = "map"
    smooth_path.header.stamp    = rospy.Time.now()
    
    #
    # TODO:
    # Write the algorithm to smooth the path by gradient descend algorith
    # using as cost function the sum of the distance between point in the new path
    # plus the distance bewteen points in the new and old paths.
    # Store the resulting path in the 'smooth_path' variable.
    # 
    for i in range(len(req.path.poses)):
        smooth_path.poses.append(req.path.poses[i])

    tol = 0.00001 * len(req.path.poses)
    gradient_mag = tol + 1
    delta = 0.5
    attempts = 10000

    while gradient_mag >= tol and attempts > 0:

	"""grad_0 = aplpha*(smooth_path.poses[0].pose.position.x - smooth_path.poses[1].pose.position.x)
	grad_0 += beta*(smooth_path.poses[0].pose.position.x - old_path.poses[0].pose.position.x)
	new_x_0 -= delta*grad_0

	grad_0 = aplpha*(smooth_path.poses[0].pose.position.y - smooth_path.poses[1].pose.position.y)
	grad_0 += beta*(smooth_path.poses[0].pose.position.y - old_path.poses[0].pose.position.y)
	new_y_0 -= delta*grad_0"""
	gradient_mag = 0
	
	for i in range(1,len(req.path.poses)-2):

		x_old = req.path.poses[i].pose.position.x
		y_old = req.path.poses[i].pose.position.y

		x_new_prev = smooth_path.poses[i-1].pose.position.x
		y_new_prev = smooth_path.poses[i-1].pose.position.y

		x_new_i = smooth_path.poses[i].pose.position.x
		y_new_i = smooth_path.poses[i].pose.position.y

		x_new_next = smooth_path.poses[i+1].pose.position.x
		y_new_next = smooth_path.poses[i+1].pose.position.y

		grad_x = beta*(x_new_i - x_old) + alpha*(2*x_new_i - x_new_prev - x_new_next)
		grad_y = beta*(y_new_i - y_old) + alpha*(2*y_new_i - y_new_prev - y_new_next)

		x_new_i -= delta*grad_x
		y_new_i -= delta*grad_y

		gradient_mag += abs(grad_x) + abs(grad_y)
	print "Gradient_mag " + str(gradient_mag)
        print tol
	attempts -= 1	    

    ####
    print "Path smoothed after " + str(attempts) + " steps"
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
    











