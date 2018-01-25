#!/usr/bin/env python

import sys
import rospy
import time

from std_msgs.msg import Bool
from std_msgs.msg import String

busy = False
pub = rospy.Publisher('/user_input', String, queue_size=1)
#rate = rospy.Rate(100)
def ready_for_pick_and_place(data):
    global busy
    print "ready for next task"
    busy = False

def main():
    if not rospy.is_shutdown():
        global busy
        while busy:
            pass
        user_input = str(raw_input())
        pub.publish(user_input)
        busy = True
        #rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('user_interface')
    busy = False
    print "When the robot is idle at home position,"
    print "you can type a capital letter abbreviation"
    print "of the shape you desire to pick up."
    print "CIR for Circle, STA for Star, CRO for Cross,"
    print "SQU for square, TRI for triangle."
    print ""
    print "Please input 3 letter abbreviation now."
    #global rate
    #rate = rospy.Rate(100)
    rospy.Subscriber( '/user_feedback', String, ready_for_pick_and_place)
    main()
