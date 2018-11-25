#!/usr/bin/env python
import rospy
import numpy
import cv2
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

def listener():
    rospy.init_node('moveForward', anonymous=True)
    rospy.Subscriber('moveForward',Bool,onMoveForward)
    rospy.Subscriber('scan',LaserScan,onScan)
    rospy.spin()

def onScan(data):
    if(data.ranges[0]<0.5):
        stopTurtleBot()

def onMoveForward(data):
    print("got message")
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    msg = Twist()
    msg.linear.x = 0.1
    pub.publish(msg)
    rospy.sleep(5.)
    stopTurtleBot()

def stopTurtleBot():
    print("STOP")
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    msg = Twist()
    msg.linear.x = 0
    pub.publish(msg)

if __name__ == '__main__':
    listener()
