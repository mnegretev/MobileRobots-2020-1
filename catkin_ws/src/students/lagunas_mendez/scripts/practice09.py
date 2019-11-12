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

NAME = "lagunas_mendez_oscar_jair"

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

    #operadores morfologicos cv2.dilate cv2.erode ROI vision activa 
    
    kernel = np.ones((7,7), np.uint8) 
    kernel2 = np.ones((3,3), np.uint8) 

    hsv = cv2.cvtColor(img_bgr,cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (2, 180,100), (6, 255,255))

    mask = cv2.erode(mask, kernel2, iterations=1) 


    mask = cv2.dilate(mask, kernel, iterations=4) 


    idx = cv2.findNonZero(mask)
    if idx is not None and len(idx)>10:
        [x,y,_,_]= cv2.mean(idx)
        img_bgr = cv2.circle(img_bgr,(int(x),int(y)),20,(255, 0, 0),3)

    #cv2.imshow("hsv", hsv)    
    cv2.imshow("imagen",img_bgr)
    cv2.imshow("mask",mask)
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

