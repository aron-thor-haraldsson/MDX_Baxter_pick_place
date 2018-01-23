#!/usr/bin/env python

import cv_bridge
import time
import cv2
import rospy
import os
import numpy as np
os.chdir(os.path.dirname(os.path.realpath(__file__)))
import process_images
import converge


from baxter_interface import CameraController
from sensor_msgs.msg import Image

rospy.init_node('get_camera_image')

# Defines an empty classifier class
classifier = process_images.Classifier()
# Trains the classifier using locally stored images
classifier.set_train(False)
converger = converge.Converge()
converger.set_frame_dimensions()
left_camera = CameraController('left_hand_camera')
left_camera.open()

camera_image = None


def get_img(msg):
    global camera_image
    #global cam_height
    #global cam_width
    camera_image = msg_to_cv(msg)
    cam_height, cam_width, _ = camera_image.shape
    classifier.set_contour_size_limits(0.01, 0.3, cam_height, cam_width)
    classifier.classify_cam_frame(camera_image, ["gray", "increase_contrast", "increase_contrast", "open", "close"])
    cv2.imshow('image', camera_image)
    report = classifier.get_built_contour_report()
    converger.set_contour_report(report)

    converger.set_search_for_shape("TRI", 80)
    print converger.get_search_for_shape()
    print "--------------------"

    cv2.waitKey(1)

def msg_to_cv(msg):
    return cv_bridge.CvBridge().imgmsg_to_cv2(msg)

camera_subscriber = rospy.Subscriber( 'cameras/left_hand_camera/image', Image, get_img)

while camera_image==None:
    pass

#print camera_image

rospy.spin()

