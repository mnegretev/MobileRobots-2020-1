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

NAME = "AVALOS_GARCIA"

def callback_laser_scan(msg): #It is a parallel event which determines if there is an obstacle, at each laser measure.
    #
    # TODO:
    # Do something to detect if there is an obstacle in front of the robot.
    #
    global obstacle_detected #Boolean variable to determine if obstacle is being detected
    if msg.ranges[len(msg.ranges)/2]<1.0: #It retrieves the data from the laser, since it is the trayectory from the robot to the wall and the return from the wall to the robot, it must be divided by two. 1.0 represents a distance in meters.
	print "Obstacle detected"
	obstacle_detected=True
    else:
	obstacle_detected=False
    return
    #     

def main():
    print "PRACTICE 00 - " + NAME
    rospy.init_node("practice00")
    rospy.Subscriber("/hardware/scan", LaserScan, callback_laser_scan)
    pub_cmd_vel = rospy.Publisher("/hardware/mobile_base/cmd_vel", Twist, queue_size=10)
    loop = rospy.Rate(10)
    #
    cmd_vel=Twist() #cmd_vel is subscribed to Twist that expresses velocity in free space broken into its linear and angular parts. 
    global obstacle_detected #Declaration of variable in main.
    obstacle_detected=False #As an initial condition, it is going to be uploaded with the sensor information.
    #
    while not rospy.is_shutdown():
        #
        # TODO:
        # Declare a Twist message and assign the appropiate speeds:
        # Move forward if there is no obstacle in front and stop otherwise.
        # Publish the message.
        #
	if obstacle_detected:
		cmd_vel.linear.x=0.0 #If the laser detects an obstacle stops.
	else:
		cmd_vel.linear.x=0.5 #If the laser does not detect an obstacle keeps moving.
	pub_cmd_vel.publish(cmd_vel) #pub_cmd_vel publishes cmd_vel value.
        loop.sleep() #It is used to keep rate during loop.  
        #

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
