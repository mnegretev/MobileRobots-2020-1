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

NAME = "ramirez_ancona"


def callback_laser_scan(msg): #msg variable tipo LaserScan
    #
    # TODO:
    # Do something to detect if there is an obstacle in front of the robot.
    global obstacleDetected
    if msg.ranges[len(msg.ranges)/2] < 1.0:
        print "Obstacle detected"
        obstacleDetected = True
    else:
        obstacleDetected = False
    return

def main():
    print "PRACTICE 00 - " + NAME
    rospy.init_node("practice00")
    rospy.Subscriber("/hardware/scan", LaserScan, callback_laser_scan)
    pub_cmd_vel = rospy.Publisher("/hardware/mobile_base/cmd_vel", Twist, queue_size=10)
    loop = rospy.Rate(10)
    cmd_vel = Twist()
    global obstacleDetected
    obstacleDetected = False
    attemps = 0
    while not rospy.is_shutdown() and attemps < 25:
        #
        # TODO:
        # Declare a Twist message and assign the appropiate speeds:
        # Move forward if there is no obstacle in front and stop otherwise.
        # Publish the message.
        #
        if obstacleDetected:
            cmd_vel.linear.x = 0
        else:
            cmd_vel.linear.x = 0.3
        pub_cmd_vel.publish(cmd_vel)
        loop.sleep()
        #print loop.sleep()
        attemps += 1


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

