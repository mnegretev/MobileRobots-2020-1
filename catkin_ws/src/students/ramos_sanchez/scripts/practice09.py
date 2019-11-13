#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2020-1
# PRACTICE 9 - COLOR SEGMENTATION
#
# Instructions:
# Complete the code to estimate the position of a small ball
# using color segmentation.
#
#

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker

NAME = "Ramos_Sanchez"

def callback_image(msg):
    bridge = CvBridge()
    img_bgr = bridge.imgmsg_to_cv2(msg, "bgr8")
    #
    # TODO:
    # - Change color space from RGB to HSV.
    #   Check online documentation for cv2.cvtColor function
    # - Determine the pixels whose color is in the color range of the ball.
    #   Check online documentation for cv2.inRange
    # - Calculate the centroid of all pixels in the given color range (ball position).
    #   Check online documentation for cv2.findNonZero and cv2.mean
    # - Draw a circle in the original image to indicate the ball position.
    #   Check online documentation for cv2.circle
    # - Display the image. Check online documentation for cv2.imshow
    # - Call function cv.waitKey. Check online documentation. 
    #
    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    lower = np.array([2, 200, 100])
    upper = np.array([7, 255, 255])
    img_bin = cv2.inRange(img_hsv, lower, upper)
    idx = cv2.findNonZero(img_bin)
    if idx is not None:
        [centroid_r, centroid_c, a, b] = cv2.mean(idx)
        circle_image = cv2.circle(img_bgr, (int(centroid_r), int(centroid_c)), 20, [0, 0, 255], 3)
        cv2.imshow("Circle image", circle_image)
    cv2.waitKey(1)

def main():
    print "PRACTICE 09 - " + NAME
    rospy.init_node("practice09")
    rospy.Subscriber("/hardware/camera_rgb/image", Image, callback_image)
    loop = rospy.Rate(30)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

