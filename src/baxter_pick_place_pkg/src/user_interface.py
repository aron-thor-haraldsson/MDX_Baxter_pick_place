#!/usr/bin/env python

import sys
import rospy
import time

from std_msgs.msg import String

pub = rospy.Publisher('/user_input', String, queue_size=1)

def main():
    while not rospy.is_shutdown():
        user_input = str(raw_input())
        pub.publish(user_input)

if __name__ == '__main__':
    rospy.init_node('user_interface')
    busy = False
    print "When the robot is idle at home position,"
    print "you can type a capital letter abbreviation"
    print "of the shape you desire to pick up."
    print "CIR for Circle, STA for Star, CRO for Cross,"
    print "SQU for square, TRI for triangle."
    print ""
    print "You can exit this program by pressing 'Ctrl' + 'C'"
    print " and pressing 'Enter'"
    print ""
    print "Please input 3 letter abbreviation now."
    sys.exit(main())
