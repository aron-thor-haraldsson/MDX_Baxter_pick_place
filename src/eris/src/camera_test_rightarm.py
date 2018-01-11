#!/usr/bin/env python

import cv_bridge
import time
import baxter_interface
import cv2
import rospy

from baxter_interface import CameraController
from sensor_msgs.msg import Image

head_camera = baxter_interface.CameraController("head_camera")
head_camera.close()
left_camera = baxter_interface.CameraController("left_hand_camera")
left_camera.close()
right_camera = baxter_interface.CameraController("right_hand_camera")
right_camera.close()

rospy.init_node('get_camera_image')

#right_camera = CameraController('right_hand_camera')
right_camera.open()

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

