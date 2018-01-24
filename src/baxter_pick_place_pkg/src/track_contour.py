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

# Defines an empty classifier class
classifier = process_images.Classifier()
# Trains the classifier using locally stored images
classifier.set_train(False)
move_message = move_message_generator.Move_Message()
move_message.set_frame_dimensions()
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
    print report
    move_message.set_contour_report(report)

    move_message.set_search_for_shape("CIR", 80)
    cmd = move_message.build_move_command()
    print cmd
    if cmd[0] or cmd[1] or cmd[2]:
        pass
        #cartesian_movement.cartesian_move("left", "displace", [cmd[0], cmd[1], cmd[2]])
        #print "awv"
    print "--------------------"
    if not rospy.is_shutdown():
        pub.publish(str(cmd))

    cv2.waitKey(1)


def msg_to_cv(msg):
    return cv_bridge.CvBridge().imgmsg_to_cv2(msg)

camera_subscriber = rospy.Subscriber( 'cameras/left_hand_camera/image', Image, get_img)

while camera_image==None:
    pass

#print camera_image

rospy.spin()

