#!/usr/bin/env python
import rospy
import numpy as np
import math
import cv2
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from std_msgs.msg import Bool, Int32, Float32
from geometry_msgs.msg import Twist, Vector3
import image_geometry

image = []
camInfo = 0
color = [0,0,0]
distances = []

def onGetCameraInfo(data):
    global camInfo
    camInfo = data

def listener():
    rospy.init_node('getDistance', anonymous=True)
    rospy.Subscriber('getDistance',Vector3,onGetDistance)
    rospy.Subscriber('camera/image_raw',Image,onGetImage)
    rospy.Subscriber('camera/camera_info',CameraInfo,onGetCameraInfo)
    rospy.Subscriber('scan',LaserScan,onGetScan)
    rospy.spin()

def onGetDistance(data):
    global color
    color[0] = data.x
    color[1] = data.y
    color[2] = data.z

    if(camInfo!=0 and distances!=0 and len(image)!=0):
        camera = image_geometry.PinholeCameraModel()
        camera.fromCameraInfo(camInfo)
        center = detectCenter(image, color)

        if(center!=0):
            pX = center[0]
            pY = center[1]
            ray = camera.projectPixelTo3dRay(camera.rectifyPoint(center))
            print("projecting ("+str(ray[0])+","+str(ray[1])+","+str(ray[2])+")")
            alpha = math.atan2(ray[0],ray[2])
            print("alpha:"+str(math.degrees(alpha)))
            if(alpha < 0):
                alpha = -alpha
            else:
                alpha = math.floor(math.pi * 2 - alpha)

            print(distances.angle_min)
            distance_index = int((alpha - distances.angle_min) / distances.angle_increment)
            actual_distance = distances.ranges[distance_index]
            pub = rospy.Publisher('distance_response',Float32,queue_size=10)
            msg = Float32()
            msg.data = actual_distance
            pub.publish(msg)

def onGetScan(data):
    global distances
    distances = data

def onGetImage(data):
    global image
    global camInfo
    global color
    global distances

    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data,"bgr8")


def detectCenter(image,color):
    img_hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    color_hsv = np.array(color,dtype="uint8")
    color_hsv = np.array([[color_hsv]],dtype="uint8")
    color_hsv = cv2.cvtColor(color_hsv,cv2.COLOR_BGR2HSV)
    color_hsv = color_hsv[0][0]

    lower = (color_hsv[0]-20 if color_hsv[0]-20 > 0 else 0,
                30,
                30)
    upper = (color_hsv[0]+20 if color_hsv[0]+20 < 180 else 255,
                255,
                255)

    lower = np.array(lower, dtype = "uint8")
    upper = np.array(upper, dtype = "uint8")
    mask = cv2.inRange(img_hsv, lower, upper)
    im2, contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    sorted_by_perimiter = sorted(contours,key = lambda x: cv2.arcLength(x,False))

    if(len(sorted_by_perimiter)>0):
        selected_contour = sorted_by_perimiter[-1] #largest contour

        M = cv2.moments(selected_contour)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        return (cX,cY)
    return 0
if __name__ == '__main__':
    listener()
