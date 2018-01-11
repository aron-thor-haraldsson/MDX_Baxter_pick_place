#!/usr/bin/env python

import cv_bridge
import time
import cv2
import rospy
import os
os.chdir(os.path.dirname(os.path.realpath(__file__)))
import process_images


from baxter_interface import CameraController
from sensor_msgs.msg import Image

rospy.init_node('get_camera_image')

left_camera = CameraController('left_hand_camera')
left_camera.open()

camera_image = None

def get_img(msg):
    global camera_image
    camera_image = msg_to_cv(msg)    
    cv2.imshow('image',camera_image)
    
    cv2.waitKey(1)

def msg_to_cv(msg):
    return cv_bridge.CvBridge().imgmsg_to_cv2(msg)

camera_subscriber = rospy.Subscriber( 'cameras/left_hand_camera/image', Image, get_img)

while camera_image==None:
    pass

#print camera_image

rospy.spin()

