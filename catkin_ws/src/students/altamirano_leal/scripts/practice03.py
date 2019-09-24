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

NAME = "APELLIDO_PATERNO_APELLIDO_MATERNO"

def callback_smooth_path(req):
    #definimos alpha y beta 
    if rospy.has_param("/navigation/path_planning/smoothing_alpha"):
        alpha = rospy.get_param("/navigation/path_planning/smoothing_alpha")
    else:
        alpha = 0.5
    if rospy.has_param("/navigation/path_planning/smoothing_beta"):
        beta = rospy.get_param("/navigation/path_planning/smoothing_beta")
    else:
        beta = 0.5
    print "Smoothing a " + str(len(req.path.poses)) + " points path with alpha=" + str(alpha) + " and beta=" + str(beta)
    #
    

    #para acedera una cordenada x
    smooth_path = Path()
    smooth_path.header.frame_id = "map"
    smooth_path.header.stamp    = rospy.Time.now()
    attempts = 0

    #
    # TODO:
    # Write the algorithm to smooth the path by gradient descend algorith
    # using as cost function the sum of the distance between point in the new path
    # plus the distance bewteen points in the new and old paths.
    # Store the resulting path in the 'smooth_path' variable.
    # 
    smooth_path = req.path
    tol = 0.0001
    gradient_mag = tol+800 #gradiente
    delta = 0.05 
    

    #algoritmos de gradiente 
    while gradient_mag > tol:
        
        grad_0_x = alpha*(smooth_path.poses[0].pose.position.x - smooth_path.poses[1].pose.position.x )  
        grad_0_x+= beta*(smooth_path.poses[0].pose.position.x - req.path.poses[0].pose.position.x)
        
        

        ## #primer valor de el gradiente libreta
        grad_0_y = alpha*(smooth_path.poses[0].pose.position.y - smooth_path.poses[1].pose.position.y )  
        grad_0_y += beta*(smooth_path.poses[0].pose.position.y - req.path.poses[0].pose.position.y)
        

        smooth_path.poses[0].pose.position.x -=delta*grad_0_x
        smooth_path.poses[0].pose.position.y -=delta*grad_0_y



        for i in range(1,len(req.path.poses)-1):
            grad_i_x = alpha*(2*(smooth_path.poses[i].pose.position.x )-smooth_path.poses[i-1].pose.position.x-smooth_path.poses[i+1].pose.position.x)
            grad_i_x +=beta*(smooth_path.poses[i].pose.position.x-req.path.poses[i].pose.position.x)
            


            grad_i_y = alpha*(2*(smooth_path.poses[i].pose.position.y )-smooth_path.poses[i-1].pose.position.y-smooth_path.poses[i+1].pose.position.y)
            grad_i_y +=beta*(smooth_path.poses[i].pose.position.y-req.path.poses[i].pose.position.y)
            
           

            smooth_path.poses[i].pose.position.x -= delta*grad_i_x
            smooth_path.poses[i].pose.position.y -= delta*grad_i_y
            gradient_mag -= abs(grad_i_x) + abs(grad_i_y)

        

        grad_fx = alpha*(smooth_path.poses[i+1].pose.position.x - smooth_path.poses[i].pose.position.x )
        grad_fx += beta*(smooth_path.poses[i+1].pose.position.x + req.path.poses[i+1].pose.position.x)
         


        grad_fy = alpha*(smooth_path.poses[i+1].pose.position.y - smooth_path.poses[i].pose.position.y )
        grad_fy += beta*(smooth_path.poses[i+1].pose.position.y + req.path.poses[i+1].pose.position.y)
        
        smooth_path.poses[i+1].pose.position.x -= delta*grad_i_x
        smooth_path.poses[i+1].pose.position.y -= delta*grad_i_y
        #print "resta " + str(delta*grad_fx)
        #print "valor " + str(smooth_path.poses[i+1].pose.position.x)

 

        
       # print "Gra " + str(gradient_mag)
        attempts=attempts+1


       # hacer lo mismo para  y 
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
    
