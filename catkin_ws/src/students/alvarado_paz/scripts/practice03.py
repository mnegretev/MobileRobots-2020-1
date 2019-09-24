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

NAME = "ALVARADO_PAZ"

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
    new_path = Path()
    smooth_path.header.frame_id = "map1"
    smooth_path.header.stamp    = rospy.Time.now()


    delta      = 0.5 # supongo un valor pequeno
    tolerancia = 0.0001 
    grad_m  = tolerancia + 1

    ################################
    # Me quede corrigiendo lo errores que salen en la ejecucion de el Calc Path 
    # referentes a mi sintaxis en mi codigo, la forma en que accedo a las cordenadas
    # es donde me arroja errores, sigo resolviendolo
    ################################
    while grad_m > tolerancia:
        # Puntos a ocupar en el gradiente
        # Coord de la ruta original
        xo_0 = req.smooth_path.poses[0].pose.position.x
        yo_0 = req.smooth_path.poses[0].pose.position.y
        # Coord de la runa n
        xn_0 = new_path.poses[0].pose.position.x
        yn_0 = new_path.poses[0].pose.position.y
        # Coord de la ruta n + 1
        xn_1 = new_path.poses[1].pose.position.x
        yn_1 = new_path.poses[1].pose.position.y

        ##
        grad_x = delta*(alpha*(xn_0 - xn_1) + beta*(xn_0 - xo_0))
        grad_y = delta*(alpha*(xn_0 - xn_1) + beta*(xn_0 - xo_0))

        # Coord de nueva ruta
        xn_0 = xn_0 - grad_x
        yn_0 = yn_1 - grad_y

        grad_m += abs(grad_x) + abs(grad_y)


        # Calculo de k 
        for i in xrange(1,len(path.poses)):
            xo_i   = req.smooth_path.poses[i].pose.position.x
            yo_i   = req.smooth_path.poses[i].pose.position.y

            xn_i   = new_path.poses[i].pose.position.x
            yn_i   = new_path.poses[i].pose.position.y

            xn_i_s = new_path.poses[i].pose.position.x
            yn_i_s = new_path.poses[i].pose.position.y

            xn_i_a = new_path.poses[i].pose.position.x
            yn_i_a = new_path.poses[i].pose.position.y

            grad_x = delta*(alpha*(2*xn_i - xn_i_s - xn_i_a) + beta*(xn_i - xo_i))
            grad_y = delta*(alpha*(2*yn_i - yn_i_s - yn_i_a) + beta*(yn_i - yo_i))

            # i-esimo termino
            new_path.poses[i].pose.position.x = new_path.poses[i].pose.position.x - grad_x
            new_path.poses[i].pose.position.y = new_path.poses[i].pose.position.x - grad_y

            grad_m += abs(grad_x) + abs(grad_y)

        # ultimos terminos
        al_x = alpha*(new_path.poses[len(path.poses)].pose.position.x - new_path.poses[len(path.poses) - 1].pose.position.x)
        be_x = beta*(new_path.poses[len(path.poses)].pose.position.x - req.smooth_path.poses[len(path.poses)].pose.position.x)
        al_y = alpha*(new_path.poses[len(path.poses)].pose.position.y - new_path.poses[len(path.poses) - 1].pose.position.y)
        be_y = beta*(new_path.poses[len(path.poses)].pose.position.y - req.smooth_path.poses[len(path.poses)].pose.position.y)
        grad_x = delta*(al_x - be_x)
        grad_y = delta*(al_y - be_y)

        #
        new_path.poses[len(path.poses)].pose.position.x = new_path.poses[len(path.poses) - 1].pose.position.x - grad_x
        new_path.poses[len(path.poses)].pose.position.y = new_path.poses[len(path.poses) - 1].pose.position.x - grad_y

        grad_m += abs(grad_x) + abs(grad_y)  

    #smooth_path = new_path
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
    
