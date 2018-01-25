#!/usr/bin/env python

## This file imports all the required files to turn a video feed into
## move commands that are then sent to be processed and sent to the Baxter arm

import cv_bridge
import time
import cv2
import rospy
import os
import numpy as np
import process_images
import move_message_generator

from baxter_interface import CameraController
from sensor_msgs.msg import Image
from std_msgs.msg import String

# This changes the current directory
# This is done in the case the machine learning model needs to be relearnt
# This will allow the imported 'process_images' to access the 'teach_images' folder
# If you want the ML model to be relearnt,
#   then change the 'classifier.set_train(False)' line below to
#   'classifier.set_train(True)'
os.chdir(os.path.dirname(os.path.realpath(__file__)))


rospy.init_node('track_shape')
pub = rospy.Publisher('/converge', String, queue_size=1)
# Defines an empty classifier class
classifier = process_images.Classifier()
# Trains the classifier using locally stored images
classifier.set_train(False)
# Creates a new generic Move_Message
move_message = move_message_generator.Move_Message()
# Sets the default dimensions for the Baxter arm camera
move_message.set_frame_dimensions()
# Initialises the Baxter camera
left_camera = CameraController('left_hand_camera')
left_camera.open()

camera_image = None
# The default value of preferred shape
# This will change depending on user input
preferred_shape = "CRO"

# Converts a video frame gotten from the Baxter arm camera to a frame that can be manipulated by opencv
def msg_to_cv(msg):
    return cv_bridge.CvBridge().imgmsg_to_cv2(msg)

# Called every time a new video frame is published by the Baxter arm camera
def get_img(msg):
    global camera_image
    # Converts the video frame to a format that can be manipulated by opencv
    camera_image = msg_to_cv(msg)
    cam_height, cam_width, _ = camera_image.shape
    # Defines the minimum and maximum allowed sizes of contours to be classified
    # These particular arguments limit it to within 1% to 30% of the video frame area
    classifier.set_contour_size_limits(0.01, 0.3, cam_height, cam_width)
    # Preprocesses the image according to the arguments passed
    # And classifies all detected contours
    classifier.classify_cam_frame(camera_image, ["gray", "increase_contrast", "increase_contrast", "increase_contrast", "open", "close"])
    # Shows the video frame with the contours and their classifications marked on it
    cv2.imshow('image', camera_image)
    # Generates a report of all detected contours and their coordinates
    report = classifier.get_built_contour_report()
    # Sends that report to be processed by 'move_message_generator'
    move_message.set_contour_report(report)

    global preferred_shape
    # Filters out all but the contour that the user desires
    move_message.set_search_for_shape(preferred_shape, 80)
    # Retreives a generated move command to be published
    cmd = move_message.build_move_command()
    if not rospy.is_shutdown():
        # Publishes the move command to 'cartesian_movement'
        pub.publish(str(cmd))
    cv2.waitKey(1)


# Gets called every time the user sends a new request
# This changes the global variable that determines which contour to search for
def user_input(data):
    global preferred_shape
    preferred_shape = data.data

# Subscribes to the video feed from the Baxter arm camera
rospy.Subscriber( 'cameras/left_hand_camera/image', Image, get_img)
# Subscribes to the 'user_interface'
rospy.Subscriber( '/user_input', String, user_input)

rospy.spin()
