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
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

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
    robot_x, robot_y, robot_a = get_robot_pose(listener)
    global_goal_x = path.poses[len(path.poses)-1].pose.position.x
    global_goal_y = path.poses[len(path.poses)-1].pose.position.y
    error_global = math.sqrt((global_goal_x - robot_x)*(global_goal_x-robot_x) + (global_goal_y-robot_y)*(global_goal_y-robot_y))
    counter=0
    goal_x = path.poses[counter].pose.position.x
    goal_y = path.poses[counter].pose.position.y
    error_local = math.sqrt((goal_x - robot_x)*(goal_x-robot_x) + (goal_y-robot_y)*(goal_y-robot_y))
    while not rospy.is_shutdown() and abs(error_global) > 0.05 and counter < len(path.poses):
        goal_x = path.poses[counter].pose.position.x
        goal_y = path.poses[counter].pose.position.y
        if error_local < 0.3:
            counter += 3
        pub_cmd_vel.publish(calculate_control(robot_x, robot_y, robot_a, goal_x, goal_y))
        robot_x, robot_y, robot_a = get_robot_pose(listener)
        error_global = math.sqrt((global_goal_x-robot_x)*(global_goal_x-robot_x) + (global_goal_y-robot_y)*(global_goal_y-robot_y))
        error_local  = math.sqrt((goal_x - robot_x)*(goal_x-robot_x) + (goal_y-robot_y)*(goal_y-robot_y))
        loop.sleep()
    print "Global goal point reached"

def get_robot_pose(listener):
    try:
        (trans, rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
        robot_x = trans[0]
        robot_y = trans[1]
        robot_a = 2*math.atan2(rot[2], rot[3])
        if robot_a > math.pi:
            robot_a -= 2*math.pi
        if robot_a < -math.pi:
            robot_a += 2*math.pi
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
    global laser_readings
    v_max = 0.8
    w_max = 1.0
    alpha = 0.2
    beta  = 0.5
    error_x = goal_x - robot_x
    error_y = goal_y - robot_y
    error_a = math.atan2(error_y, error_x) - robot_a
    error_d = math.sqrt(error_x*error_x + error_y*error_y)
    if error_a >  math.pi:
        error_a -= 2*math.pi
    if error_a < -math.pi:
        error_a += 2*math.pi
    v_max = min(v_max, error_d)
    cmd_vel = Twist()
    cmd_vel.linear.x  = v_max*math.exp(-error_a*error_a/alpha)
    cmd_vel.angular.z = w_max*(2/(1 + math.exp(-error_a/beta)) - 1)
    rfx, rfy = rejection_force(robot_x, robot_y, robot_a, laser_readings)
    rfx_robot = rfx * math.cos(robot_a) - rfy * math.sin(robot_a)
    rfy_robot = rfx * math.sin(robot_a) + rfy * math.cos(robot_a)
    #pub_pot_fields = rospy.Publisher("/campos", String, queue_size=1)
    #message = str(rfy_robot) + " " + str(rfx_robot)
    #pub_pot_fields.publish(message)
    cmd_vel.linear.y = rfy_robot * 0.4
    return cmd_vel

def rejection_force(robot_x, robot_y, robot_a, laser_readings):
    beta = 6.0 #Rejection constant
    d0   = 1.15 #Distance of influence
    force_x = 0
    force_y = 0
    for [distance, angle] in laser_readings:
        if distance < d0 and distance > 0:
            mag = beta*math.sqrt(1/distance - 1/d0)
        else:
            mag = 0
        force_x += mag*math.cos(angle + robot_a)
        force_y += mag*math.sin(angle + robot_a)
    if len(laser_readings) == 0:
        return [force_x, force_y]
    [force_x, force_y] = [force_x/len(laser_readings), force_y/len(laser_readings)]
    return [force_x, -force_y]

def callback_scan(msg):
    global laser_readings
    laser_readings = [[0,0] for i in range(len(msg.ranges))]
    for i in range(len(msg.ranges)):
        laser_readings[i] = [msg.ranges[i], msg.angle_min + i*msg.angle_increment]

def main():
    print "PRACTICE 04 - " + NAME
    rospy.init_node("practice04")
    rospy.Subscriber("/hardware/scan", LaserScan, callback_scan)
    rospy.Subscriber('/navigation/simple_move/follow_path', Path, callback_follow_path)
    
    loop = rospy.Rate(20)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
