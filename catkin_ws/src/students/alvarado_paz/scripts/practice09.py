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

import numpy
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker

import numpy as np
#import imutils


NAME = "ALVARADO_PAZ"

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
    
    orangeMin = (0, 200, 95)  # 4, 140, 200     # 2, 169, 166
    orangeMax = (10, 255, 255)# 30, 255, 255    # 179, 255, 255
    #Kernels a usar en la dilatacion y erosion en los operadores morfologicos
    kernel0 = np.ones((4,4), np.uint8)
    kernel1 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(4,4))
    #BGR a HSV
    hsv  = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    #Deteccion de la pelota naranja
    mask = cv2.inRange(hsv, orangeMin, orangeMax)
    #Operadores morfologicos
    mask = cv2.erode( mask, kernel0, iterations=1)
    mask = cv2.dilate(mask, kernel0, iterations=3)

    idx = cv2.findNonZero( mask )

    # Contorno
    #(_, cnts, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    # Centroide de la pelota
    retval = cv2.mean( idx )
    center = ( int(retval[0]), int(retval[1]) )
    cv2.circle(img_bgr, center, 4, (255, 255, 0), -1)

    # Contorno    
    #for i in range(0, len(cnts) -1):
    #    cv2.drawContours(img_bgr, cnts, i, (240, 0, 159), 3)

    # Mostrando imagenes
    cv2.imshow("Frame", img_bgr)
    #cv2.imshow("HSV",   hsv)
    cv2.imshow("MASK",  mask)

    cv2.waitKey (1)

    
    
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

