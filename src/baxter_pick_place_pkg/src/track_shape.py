#!/usr/bin/env python

import cv_bridge
import time
import cv2
import rospy
import os
import numpy as np
os.chdir(os.path.dirname(os.path.realpath(__file__)))
import process_images
import move_message_generator
from sys import stdout


from baxter_interface import CameraController
from sensor_msgs.msg import Image
from std_msgs.msg import String

rospy.init_node('track_shape')
pub = rospy.Publisher('/converge', String, queue_size=1)
rate = rospy.Rate(100)
# Defines an empty classifier class
classifier = process_images.Classifier()
# Trains the classifier using locally stored images
classifier.set_train(False)
move_message = move_message_generator.Move_Message()
move_message.set_frame_dimensions()
left_camera = CameraController('left_hand_camera')
left_camera.open()

camera_image = None
preferred_shape = "CRO"


def get_img(msg):
    global camera_image
    #global cam_height
    #global cam_width
    camera_image = msg_to_cv(msg)
    cam_height, cam_width, _ = camera_image.shape
    classifier.set_contour_size_limits(0.01, 0.3, cam_height, cam_width)
    classifier.classify_cam_frame(camera_image, ["gray", "increase_contrast", "increase_contrast", "increase_contrast", "open", "close"])
    cv2.imshow('image', camera_image)
    report = classifier.get_built_contour_report()
    move_message.set_contour_report(report)

    global preferred_shape
    move_message.set_search_for_shape(preferred_shape, 80)
    cmd = move_message.build_move_command()
    if cmd[0] or cmd[1] or cmd[2]:
        pass
        #cartesian_movement.cartesian_move("left", "displace", [cmd[0], cmd[1], cmd[2]])
        #print "awv"
    if not rospy.is_shutdown():
        global rate
        pub.publish(str(cmd))
        rate.sleep()
    cv2.waitKey(1)


def msg_to_cv(msg):
    return cv_bridge.CvBridge().imgmsg_to_cv2(msg)
def user_input(data):
    global preferred_shape
    preferred_shape = data.data

rospy.Subscriber( 'cameras/left_hand_camera/image', Image, get_img)
rospy.Subscriber( '/user_input', String, user_input)

while camera_image==None:
    pass


rospy.spin()

