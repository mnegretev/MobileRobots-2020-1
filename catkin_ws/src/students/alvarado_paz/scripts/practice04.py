#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2020-1
# PRACTICE 4 - POSITION CONTROL
#
# Instructions:
# Write the code necessary to move the robot along a given path.
# Consider a differential base. Max linear and angular speeds
# must be 0.8 and 1.0 respectively.
#
import sys
import rospy
import tf
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist

NAME = "APELLIDO_PATERNO_APELLIDO_MATERNO"

def callback_follow_path(path):
    print "Following path with " + str(len(path.poses)) + " points..."
    pub_cmd_vel = rospy.Publisher('/hardware/mobile_base/cmd_vel', Twist, queue_size=10)
    listener = tf.TransformListener()
    listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(5.0))
    loop = rospy.Rate(20)

    #
    # TODO:
    # Use the calculate_control function to move the robot along
    # each point of the path given by 'path'
    #
    # You can get the i-th point with
    # path.poses[i].pose.position.x
    # path.poses[i].pose.position.y
    #
    # And you can get the robot position with
    # robot_x, robot_y, robot_a = get_robot_pose(listener)
    #
    # Use the publisher 'pub_cmd_vel' to publish the calculated linear and angular speeds:
    # pub_cmd_vel.publish(cmd_vel)
    # where cmd_vel is the Twist message returned by the calculate_control function.
    #

    #
    # Use a while loop like this one to keep the robot moving while the
    # goal point is not reached.
    # Add as many conditions as you need.
    #
    # while not rospy.is_shutdown():
    #     loop.sleep()
    #
    
    print "Global goal point reached"

def get_robot_pose(listener):
    try:
        (trans, rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
        robot_x = trans[0]
        robot_y = trans[1]
        robot_a = 2*math.atan2(rot[2], rot[3])
        if robot_a > math.pi:
            robot_a -= 2*math.pi
        return robot_x, robot_y, robot_a
    except:
        pass
    return None

def calculate_control(robot_x, robot_y, robot_a, goal_x, goal_y):
    #
    # TODO:
    # Implement the control law given by:
    #
    # v = v_max*math.exp(-error_a*error_a/alpha)
    # w = w_max*(2/(1 + math.exp(-error_a/beta)) - 1)
    #
    # where error_a is the angle error and
    # v and w are the linear and angular speeds taken as input signals
    # and v_max, w_max, alpha and beta, are tunning constants.
    # Store the resulting v and w in the Twist message cmd_vel
    # and return it (check online documentation for the Twist message).
    # Remember to keep error angle in the interval (-pi,pi]
    #
    
    cmd_vel = Twist()
    # cmd_vel.linear.x  =  #Assign the calculated linear speed v 
    # cmd_vel.angular.z =  #Assign the calculated angular speed w
    return cmd_vel

def main():
    print "PRACTICE 04 - " + NAME
    rospy.init_node("practice04")
    rospy.Subscriber('/navigation/simple_move/follow_path', Path, callback_follow_path)
    
    loop = rospy.Rate(20)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
