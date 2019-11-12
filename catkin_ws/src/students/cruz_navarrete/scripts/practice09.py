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
import math 

NAME = "CRUZ_NAVARRETE"

def callback_image(msg):
    bridge = CvBridge()
    img_bgr = bridge.imgmsg_to_cv2(msg, "bgr8") #convert the message to a new image
    #bgr8 -- color image with blue-green-red color order
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
    # cv2.cvtColor  --- cambiar de un espaci de color aotro
    #    cv2.imshow("BGR Image",img_bgr) -- rango de un color
    #cv2.findNonZero

    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    
    lower_orange = numpy.array([2,200,100])
    upper_orange = numpy.array([7,250,250])
    img_bin = cv2.inRange(img_hsv, lower_orange, upper_orange)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(4,4))
    img_opening = cv2.morphologyEx(img_bin, cv2.MORPH_OPEN, kernel)

    kernel2 = cv2.getStructuringElement(cv2.MORPH_RECT,(10,10))
    img_dilation = cv2.dilate(img_opening,kernel2, iterations = 1)

    img_cropped_color = cv2.bitwise_and(img_bgr, img_bgr, mask = img_dilation)

    img_hsv2 = cv2.cvtColor(img_cropped_color, cv2.COLOR_BGR2HSV)

    img_1 = cv2.inRange(img_hsv2,lower_orange, upper_orange)



    cv2MajorVersion = cv2.__version__.split(".")[0]
    
    if int(cv2MajorVersion) >= 4:
        ctrs, hier = cv2.findContours(img_dilation.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    else:
        im2, ctrs, hier = cv2.findContours(img_dilation.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # sort contours
    sorted_ctrs = sorted(ctrs, key=lambda ctr: cv2.boundingRect(ctr)[0])

    for i, ctr in enumerate(sorted_ctrs):
        # Get bounding box
        x, y, w, h = cv2.boundingRect(ctr)

        # Getting ROI
        roi = img_bgr[y:y + h, x:x + w]
        #print("x: " + str(type(x)) + "y: " + str(type(y)) + "w: " + str(type(w)) + "h: "+ str(type(h)))
        # show ROI
        # cv2.imshow('segment no:'+str(i),roi)
        cv2.rectangle(img_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(img_bgr, (x+w/2,y+h/2),20, (0,255,0),3)

        #if w > 15 and h > 15:
            #cv2.imwrite('C:\\Users\\PC\\Desktop\\output\\{}.png'.format(i), roi)
   

    cv2.imshow("Dilated image", img_dilation)
    cv2.imshow("CROPED IMAGE COLOR", img_cropped_color)
    #cv2.imshow("BINARY IMAGE", img_1)
    cv2.imshow("DILATED",img_bgr)
    cv2.waitKey(1)


    
def main():
    print "PRACTICE 09 - " + NAME
    rospy.init_node("practice09")
    rospy.Subscriber("/hardware/camera_rgb/image", Image, callback_image)
    #The ROS timer allows to set a callback that will triggered at a given rate
    #in this case te given rate is 30 Hz
    loop = rospy.Rate(30) #keep the node suscribing at a fixed frequency
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

#boundary -- perimetro, frontera