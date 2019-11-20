#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2020-1
# PRACTICE 0 - THE PLATFORM ROS 
#
# Instructions:
# Write a program to move the robot forward until the laser
# detects an obstacle in front of it.
# Required publishers and subscribers are already declared and initialized.
#

import rospy
from sensor_msgs.msg   import LaserScan
from geometry_msgs.msg import Twist

NAME = "Alfonso Altamirano"

def callback_laser_scan(msg):

	global objeto_detectado #variable global 
	
	if msg.ranges[len(msg.ranges)/2]<1.0: # ranges da la longuitud de un areglo en este el areglo de los angulos
	
		print "Obstaculo detectado"
		objeto_detectado=True
	else:
		objeto_detectado=False
    #
    # TODO:
    # Do something to detect if there is an obstacle in front of the robot.
    #

def main():
    print "PRACTICE 00 - " + NAME
    rospy.init_node("practice00")
    #subscriptor p
    rospy.Subscriber("/hardware/scan", LaserScan, callback_laser_scan)
    #publicador publica una x y z 
    pub_cmd_vel = rospy.Publisher("/hardware/mobile_base/cmd_vel", Twist, queue_size=10)
    loop = rospy.Rate(10)
    
    cmd_vel=Twist()
    global objeto_detectado 
    objeto_detectado=False

    while not rospy.is_shutdown():

    	if objeto_detectado:
    		cmd_vel.linear.x=0

    	else:
    		cmd_vel.linear.x=0.5 #velocidad lineal = .5
    	pub_cmd_vel.publish(cmd_vel) #publicar los valores de la velocidad para la coneccion a la ros
        #
        # TODO:
        # Declare a Twist message and assign the appropiate speeds:
        # Move forward if there is no obstacle in front and stop otherwise.
        # Publish the message.
        #
        loop.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
