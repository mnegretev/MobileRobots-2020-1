#!/usr/bin/env python

import numpy
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

def main():
    rospy.init_node("camera_rgb")
    pub_rgb = rospy.Publisher("/hardware/camera_rgb/image", Image, queue_size=10)
    loop = rospy.Rate(30)
    video_source = 0
    if rospy.has_param("~video_file"):
        video_source = rospy.get_param("~video_file")
    print "Initializing camera_rgb node using " + ("camera" if video_source == 0 else str(video_source))
    
    bridge = CvBridge()
    cap    = cv2.VideoCapture(video_source)
    if not cap.isOpened():
        print "Cannot open video source"
        return
    
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue
        pub_rgb.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
        loop.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
