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
import cv2       #open cv
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker

NAME = "APELLIDO_PATERNO_APELLIDO_MATERNO"

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

    #imagen en hsv
    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    #imagen binaria
    img_bin = cv2 . inRange ( img_hsv , numpy.array([2,200,100]), numpy.array([7,255,255] ))

    #lista de picxeles en blanco (cordenadas )

    ######
    ## buscar el numero de pixeles 
    ## buscar operadores morfologuicos open cv cv2 dilatacion y erocion 
    ## cv2.dilate
    ## cv2.erode
    ##


    idx = cv2.findNonZero(img_bin)
    #media de pociciones 
    [centroide_r , centrode_c, a,b] = cv2.mean(idx)

    #circulo
    if idx is not None and len(idx)>400:
        cv2.circle(img_bgr, (int(centroide_r) , int(centrode_c)), 20, [100,255,0], 3) # 20 radio de circulo 3 ancho de la linea
        print len(idx) ;
    #desplegamos la imagen
    cv2.imshow("Bin Image" , img_bin) 
    cv2.imshow("HSV Image" , img_hsv)
    cv2.imshow("BGR Image" , img_bgr)
    cv2.waitKey(1)
    print img_bin.size ; 
   
    
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

