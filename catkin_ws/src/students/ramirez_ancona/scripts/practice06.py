#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2020-1
# PRACTICE 6 - LINE EXTRACTION FROM LASER READINGS
#
# Instructions:
# Complete the code to extract lines from laser readings.
#
#

import sys
import math
import rospy
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

NAME = "ramirez_ancona"

#
# TODO:
# Tune this constants to get a good performance.
#
P2P_DIST_THRESHOLD  = 0.3
MIN_POINTS_PER_LINE = 20
MAX_DIST_P_TO_LINE  = 0.15

def get_line_equation(x1, y1, x2, y2):
    #
    # TODO:
    # Given two points, (x1, y1) and (x2, y2), calculate the equation of the form
    # Ax + By + C = 0
    # and return the equation coefficients [A,B,C]
    #
    a = y1 - y2
    b = x2 - x1
    c = -(a*x1+b*y1)
    [A,B,C] = a, b, c
    return [A,B,C]

def find_farthests_point(line_equation, points):
    #
    # TODO:
    # Given a line with equation L: Ax + By + C = 0, with coefficients [A,B,C], and a set of points [x,y]
    # Find the farthest point from line L and its corresponding index.
    # You can get the line coefficients with [A,B,C] = line_equation.
    # You can get the i-th point with [x,y] = points[i]
    # Return the farthest distance and the index
    [A,B,C] = line_equation
    max_distance = 0
    index = 0
    for i in range(len(points)):
        [x,y] = points[i]
        d = abs(A*x+B*y+C)
        if d > max_distance:
            max_distance = d
            farthest_point_index = i
    return max_distance, farthest_point_index

def cluster_by_distance(point_cloud):
    #
    # TODO:
    # Cluster the points in 'point_cloud' by distance, i.e., all clusters
    # should have points that are close to each other. You can do this with
    # the following algorithm:
    #
    # Initialize 'clusters' as an empty list. This variable will contain all clusters
    # Initialize 'cluster' with the first point of the point cloud. This variable will contain the current points being clustered.
    # FOR all points in point cloud:
    #     Get the i-th     point: [xi, yi] = point_cloud[i]
    #     Get the (i-1)-th point: [xp, yp] = point_cloud[i-1]
    #     Calculate distance between this two points
    #     IF distance < P2P_DIST_THRESHOLD:
    #         Add point (xi,yi) to the current cluster: cluster.append([xi, yi])
    #     ELSE:
    #         Append the current cluster to clusters: clusters.append(cluster)
    #         Reinitialize cluster with point (xi, yi): cluster = [[xi, yi]]
    # return clusters
    #
    clusters = []
    cluster_a = [point_cloud[0]]
    for i in range(1,len(point_cloud)):
        [xi , yi] = point_cloud[i]
        [xp , yp] = point_cloud[i-1]
        d = math.sqrt((xi-xp)*(xi-xp)+(yi-yp)*(yi-yp)) 
        if d < P2P_DIST_THRESHOLD:
            cluster_a.append([xi , yi])
        else:
            clusters.append(cluster_a)
            cluster_a = [xi , yi]
    return clusters

def split_clusters(clusters):
    #
    # TODO:
    # Given a set of clusters (previously calcualted with the cluster_by_distance function), split them
    # to get new clusters such that, each new cluster can fit in a straight line.
    # You can do this with the following steps:
    #
    # Initialize 'line_clusters' as an empty list. This variable will contain all clusters.
    # FOR each cluster in clusters:
    #     IF size of i-th clustes is less than MIN_POINTS_PER_LINE:
    #         Ignore it and continue
    #     Get the first point of the i-th cluster: [x1, y1] = clusters[i][0]
    #     Get the last  point of the i-th cluster: [x2, y2] = clusters[i][len(clusters[i]) - 1]
    #     Calculate the line equation L given by (x1,y1) and (x2,y2)
    #     Find the farthets point of cluster[i] from line L and its corresponding index.
    #     If distance > MAX_DIST_P_TO_LINE:
    #         Add to line_clusters a new cluster: line_clusters.append(clusters[i][0:farthest_point_index])
    #         Add to line clusters a new cluster: line_clusters.append(clusters[i][farthest_point_index:len(clusters[i])])
    #     ELSE:
    #         Add to line_clusters the cluster[i]
    # return line_clusters
    #
    line_clusters = []
    for cluster in clusters:
        if len(cluster) < MIN_POINTS_PER_LINE:
            continue
        print cluster[0]
        [x1 , y1] = cluster[0]
        [x2 , y2] = cluster[len(cluster) - 1]
        eq = get_line_equation(x1, x2, y1, y2)
        dist, farthest_point_index = find_farthests_point(eq, cluster)
        if dist < MAX_DIST_P_TO_LINE:
            line_clusters.append(cluster[0:farthest_point_index])
            line_clusters.append(cluster[farthest_point_index:len(cluster)])
        else:
            line_clusters.append(cluster)
    return line_clusters

def extract_lines(point_cloud):
    #
    # TODO:
    # Cluster the point cloud by distance (call the corresponding function)
    # Call the split_clusters function until length of resulting clusters is
    # equal to the previous one.
    # Get a set of lines where each line is give by the first and last points of the correponding cluster:
    # lines = [..., [cluster[i][0], cluster[i][ni]], ...], where ni is the lenght of the i-th cluster.
    # return lines
    clusters = cluster_by_distance(point_cloud)
    prev_counter = 0
    line_clusters = split_clusters(clusters)
    while len(line_clusters) != prev_counter:
        prev_counter = len(line_clusters)
        line_clusters = split_clusters(line_clusters)
    for i in range((len(clusters))):
        lines.append([clusters[i][0],clusters[i][len(clusters[i]-1)]])
    return lines

def get_line_markers(lines):
    mrk = Marker()
    mrk.header.frame_id = "map"
    mrk.header.stamp    = rospy.Time.now()
    mrk.ns = "line_detection"
    mrk.id = 0
    mrk.type    = Marker.LINE_LIST
    mrk.action  = Marker.ADD
    mrk.scale.x = 0.07
    mrk.color.r, mrk.color.g, mrk.color.b, mrk.color.a = [0,0.5,1,0.8]
    for [[x1, y1],[x2,y2]] in lines:
        p1 = Point()
        p2 = Point()
        p1.x, p1.y = x1, y1
        p2.x, p2.y = x2, y2
        mrk.points.append(p1)
        mrk.points.append(p2)
    return mrk

def callback_scan(msg):
    global robot_x, robot_y, robot_a
    global pub_markers
    point_cloud = []
    for i in range(len(msg.ranges)):
        if msg.ranges[i] >= msg.range_max:
            continue
        [r, a] = [msg.ranges[i], msg.angle_min + i*msg.angle_increment]
        point_cloud.append([robot_x +r* math.cos(robot_a + a), robot_y + r*math.sin(robot_a + a)])
    lines = extract_lines(point_cloud)
    pub_markers.publish(get_line_markers(lines))
    
def main():
    global robot_x, robot_y, robot_a
    global pub_markers
    print "PRACTICE 06 - " + NAME
    rospy.init_node("practice06")
    rospy.Subscriber("/hardware/scan", LaserScan, callback_scan)
    pub_markers = rospy.Publisher('/navigation/extracted_lines', Marker, queue_size=10)
    listener = tf.TransformListener()
    loop = rospy.Rate(10)
    [robot_x, robot_y, robot_a] = [0,0,0]
    while not rospy.is_shutdown():
        listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(5.0))
        try:
            (trans, rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
            robot_x = trans[0]
            robot_y = trans[1]
            robot_a = 2*math.atan2(rot[2], rot[3])
            if robot_a > math.pi:
                robot_a -= 2*math.pi
        except:
            pass
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
