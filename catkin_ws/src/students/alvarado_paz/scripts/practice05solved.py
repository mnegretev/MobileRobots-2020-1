#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2020-1
# PRACTICE 5 - OBSTACLE AVOIDANCE BY POTENTIAL FIELDS
#
# Instructions:
# Complete the code to implement obstacle avoidance by potential fields
# using the attractive and repulsive fields technique.
# Tune the constants alpha and beta to get a good movement. 
#

import rospy
import tf
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan

NAME = "ALVARADO_PAZ"

def attraction_force(robot_x, robot_y, goal_x, goal_y):
    #
    # TODO:
    # Calculate the attraction force, given by
    # a vector whose direction points from goal position
    # towards robot position and whose magnitude is
    # a constant alpha. 
    # Return a tuple of the form [force_x, force_y]
    # where force_x and force_y are the X and Y components
    # of the resulting attraction force. 
    #
    alpha = 2.0  #Attraction constant
    force_x  = robot_x - goal_x
    force_y  = robot_y - goal_y
    mag = math.sqrt(force_x**2 + force_y**2)
    if mag == 0:
        return [0, 0]
    [force_x, force_y] = [alpha*force_x/mag, alpha*force_y/mag]
    return [force_x, force_y]

def rejection_force(robot_x, robot_y, robot_a, laser_readings):
    #
    # TODO:
    # Calculate the toal rejection force given by the average
    # of the rejection forces caused by each laser reading.
    # laser_readings is an array where each element is a tuple [distance, angle]
    # both measured w.r.t. robot's frame.
    # See lecture notes for equations to calculate rejection forces.
    # Return a tuple of the form [force_x, force_y]
    # where force_x and force_y are the X and Y components
    # of the resulting rejection force. 
    #
    beta = 6.0 #Rejection constant
    d0   = 1.0 #Distance of influence
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
    return [force_x, force_y]

def callback_pot_fields_goal(msg):
    goal_x = msg.pose.position.x
    goal_y = msg.pose.position.y
    print "Moving to goal point " + str([goal_x, goal_y]) + " by potential fields"
    pub_cmd_vel = rospy.Publisher('/hardware/mobile_base/cmd_vel', Twist,  queue_size=10)
    pub_markers = rospy.Publisher('/navigation/pot_field_markers', Marker, queue_size=10)
    listener = tf.TransformListener()
    listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(5.0))
    loop = rospy.Rate(20)
    global laser_readings

    #
    # TODO:
    # Move the robot towards goal point using potential fields.
    # Remember goal point is a local minimun in the potential field, thus,
    # it can be reached by the gradient descend algorithm.
    # Sum of attraction and rejection forces is the gradient of the potential field,
    # then, you can reach the goal point with the following pseudocode:
    #
    # Set constant epsilon (0.5 is a good start)
    # Set tolerance  (0.1 is a good start)
    # Get robot position by calling robot_x, robot_y, robot_a = get_robot_pose(listener)
    # Calculate distance to goal as math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
    # WHILE distance_to_goal_point > tolerance:
    #     Calculate attraction force Fa by calling [fax, fay] = attraction_force(robot_x, robot_y, goal_x, goal_y)
    #     Calculate rejection  force Fr by calling [frx, fry] = rejection_force (robot_x, robot_y, robot_a, laser_readings)
    #     Calculate resulting  force F = Fa + Fr
    #     Calculate next local goal point P = [px, py] = Pr - epsilon*F
    #
    #     Calculate control signals by calling msg_cmd_vel = calculate_control(robot_x, robot_y, robot_a, px, py)
    #     Send the control signals to mobile base by calling pub_cmd_vel.publish(msg_cmd_vel)
    #
    #     To draw the attraction force, call pub_markers.publish(get_force_marker(robot_x,robot_y,afx,afy,[0,0,1,1]  , 0))
    #     To draw the rejection  force, call pub_markers.publish(get_force_marker(robot_x,robot_y,rfx,rfy,[1,0,0,1]  , 1))
    #     To draw the resulting  force, call pub_markers.publish(get_force_marker(robot_x,robot_y,fx ,fy ,[0,0.6,0,1], 2))
    #
    #     Wait a little bit of time by calling loop.sleep()
    #     Update robot position by calling robot_x, robot_y, robot_a = get_robot_pose(listener)
    #     Update distance to goal position

    robot_x, robot_y, robot_a = get_robot_pose(listener)
    dist_to_goal = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
    tolerance = 0.1
    epsilon = 0.5
    while dist_to_goal > tolerance:
        afx, afy = attraction_force(robot_x, robot_y, goal_x, goal_y)
        rfx, rfy = rejection_force (robot_x, robot_y, robot_a, laser_readings)
        [fx, fy] = [afx + rfx, afy + rfy]
        [px, py] = [robot_x - epsilon*fx, robot_y - epsilon*fy]
        
        msg_cmd_vel = calculate_control(robot_x, robot_y, robot_a, px, py)
        pub_cmd_vel.publish(msg_cmd_vel)
        
        pub_markers.publish(get_force_marker(robot_x, robot_y, afx, afy, [0,0,1,1]  , 0))
        pub_markers.publish(get_force_marker(robot_x, robot_y, rfx, rfy, [1,0,0,1]  , 1))
        pub_markers.publish(get_force_marker(robot_x, robot_y,  fx,  fy, [0,0.6,0,1], 2))

        loop.sleep()
        robot_x, robot_y, robot_a = get_robot_pose(listener)
        dist_to_goal = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
    print "Goal point reached"

def calculate_control(robot_x, robot_y, robot_a, goal_x, goal_y):
    v_max = 0.5
    w_max = 0.8
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
    #
    #rfx, rfy = rejection_force (robot_x, robot_y, robot_a, laser_readings)
    #m_rej_f = math.sqrt(rfx*rfx + rfy*rfy)
    #cte_p = 0.3
    #cmd_vel.linear.y  = cte_p*m_rej_f
    #
    cmd_vel.angular.z = w_max*(2/(1 + math.exp(-error_a/beta)) - 1)
    return cmd_vel

def get_robot_pose(listener):
    listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(5.0))
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

def callback_scan(msg):
    global laser_readings
    laser_readings = [[0,0] for i in range(len(msg.ranges))]
    for i in range(len(msg.ranges)):
        laser_readings[i] = [msg.ranges[i], msg.angle_min + i*msg.angle_increment]

def get_force_marker(robot_x, robot_y, force_x, force_y, color, id):
    mrk = Marker()
    mrk.header.frame_id = "map"
    mrk.header.stamp    = rospy.Time.now()
    mrk.ns = "pot_fields"
    mrk.id = id
    mrk.type   = Marker.ARROW
    mrk.action = Marker.ADD
    mrk.color.r, mrk.color.g, mrk.color.b, mrk.color.a = color
    mrk.scale.x, mrk.scale.y, mrk.scale.z = [0.07, 0.1, 0.15]
    mrk.points.append(Point())
    mrk.points.append(Point())
    mrk.points[0].x, mrk.points[0].y = robot_x,  robot_y
    mrk.points[1].x, mrk.points[1].y = robot_x - force_x, robot_y - force_y
    return mrk

def main():
    print "PRACTICE 05 - " + NAME
    rospy.init_node("practice05")
    rospy.Subscriber("/hardware/scan", LaserScan, callback_scan)
    rospy.Subscriber("/navigation/obs_avoidance/pot_fields_goal", PoseStamped, callback_pot_fields_goal)
    
    loop = rospy.Rate(20)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
