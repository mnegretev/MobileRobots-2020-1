#!/usr/bin/env python
import math
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty


def callbackJoy(msg):
    global speedX
    global speedY
    global yaw
    global stop
    
    ### Control of mobile-base with right Stick
    stop = msg.buttons[1]
    rightStickX = msg.axes[3]
    rightStickY = msg.axes[4]
    magnitudRight = math.sqrt(rightStickX*rightStickX + rightStickY*rightStickY)
    if magnitudRight > 0.05 and stop == 0:
        speedX = rightStickY
        yaw = rightStickX
    else:
        speedX = 0
        yaw = 0

def main():
    global speedX
    global speedY
    global yaw
    global stop
    
    speedY = 0
    speedX = 0
    yaw = 0
    msgTwist = Twist()
    msgStop = Empty()
    #msgHeadTorque = Float32MultiArray()
    
    print "INITIALIZING JOYSTICK TELEOP BY MARCOSOFT... :)"
    rospy.init_node("joystick_teleop")
       
    # rospy.Subscriber("/hardware/joy", Joy, callbackJoy)
    rospy.Subscriber("/hardware/joy", Joy, callbackJoy)
    pubStop = rospy.Publisher("/hardware/robot_state/stop", Empty, queue_size = 1)
    pubTwist = rospy.Publisher("/hardware/mobile_base/cmd_vel", Twist, queue_size =1)

    loop = rospy.Rate(10)
    stop_published = True
    while not rospy.is_shutdown():
        if math.fabs(speedX) > 0 or math.fabs(speedY) > 0 or math.fabs(yaw) > 0:
            stop_published = False
            msgTwist.linear.x = speedX
            msgTwist.linear.y = speedY/2.0
            msgTwist.linear.z = 0
            msgTwist.angular.z = yaw
            #print "x: " + str(msgTwist.linear.x) + "  y: " + str(msgTwist.linear.y) + " yaw: " + str(msgTwist.angular.z)
            pubTwist.publish(msgTwist)
        elif not stop_published:
            stop_published=True
            msgTwist.linear.x = 0
            msgTwist.linear.y = 0
            msgTwist.linear.z = 0
            msgTwist.angular.z = 0
            #print "x: " + str(msgTwist.linear.x) + "  y: " + str(msgTwist.linear.y) + " yaw: " + str(msgTwist.angular.z)
            pubTwist.publish(msgTwist)
            
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
