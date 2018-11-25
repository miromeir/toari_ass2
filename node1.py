#!/usr/bin/env python
import rospy
import numpy
import cv2
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32, Float32
from geometry_msgs.msg import Vector3

def moveForward():
    pub = rospy.Publisher('moveForward', Bool, queue_size=10)
    msg=Bool()
    msg.data=True
    pub.publish(msg)

def turnAround(alpha):
    pub = rospy.Publisher('turnAround',Int32, queue_size=10)
    msg = Ing32()
    msg.data = alpha
    pub.publish(msg)

def showMenu():
    print("Please select an option:\n")
    print("(1)Move Forward")
    print("(2)Turn Around")
    print("(3)Distance to object with color X")
    print("(4)Find object with color X")

def getUserInput():
    selection = input("Enter your selection:")
    if(selection == 1):
        moveForward()

    if(selection == 2):
        turnAround(getAlphaParam())

    if(selection == 3):
        getDistance(getColorParam())

def getAlphaParam():
    return input("Enter angle for rotation:")

def getDistance(bgr):
    pub = rospy.Publisher('getDistance',Vector3, queue_size=10)
    msg = Vector3()
    msg.x = bgr[0]
    msg.y = bgr[1]
    msg.z = bgr[2]
    pub.publish(msg)

def getColorParam():
    b = int(input("Enter B value:"))
    g = int(input("Enter G value:"))
    r = int(input("Enter R value:"))
    return (b,g,r)

def onDistanceResponse(data):
    print("\n\nDistance from selected color:"+str(data.data))
    showMenu()
    print("Enter your input:")

if __name__ == '__main__':
    rospy.init_node('ass2_gui', anonymous=True)
    rospy.Subscriber('distance_response',Float32,onDistanceResponse)
    while 1:
        showMenu()
        getUserInput()
