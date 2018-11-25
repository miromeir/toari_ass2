#!/usr/bin/env python
import rospy
import numpy
import math
import cv2
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Twist

def listener():
    rospy.init_node('turnAround', anonymous=True)
    rospy.Subscriber('turnAround',Int32,onTurnAround)
    rospy.spin()

def onTurnAround(data):
    print("got TurnAround")
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    angleInRad = math.radians(data.data)
    msg = Twist()
    msg.angular.z = 1
    pub.publish(msg)
    print("sleeping for 1 seconds",angleInRad)
    rospy.sleep(angleInRad) #sleep for 'angleInRad' seconds
    print("done sleeping 1 sec stop.")
    msg.angular.z = 0
    pub.publish(msg)

def stopTurtleBot():
    print("STOP")
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    msg = Twist()
    msg.linear.x = 0
    pub.publish(msg)

if __name__ == '__main__':
    listener()
