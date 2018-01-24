#!/usr/bin/env python

import sys
import rospy
import baxter_interface
import tf
import struct
import numpy as np
import time

from geometry_msgs.msg import (
	PoseStamped,
	Pose,
	Point,
	Quaternion,
	)

from std_msgs.msg import Header
from std_msgs.msg import String

from baxter_core_msgs.srv import (
	SolvePositionIK,
	SolvePositionIKRequest,
	)
rospy.init_node("rsdk_ik_service_client")
# the default coordinates to use for the left arm if not specified
default_move = [0.069,0.841,0.1145]

flag_beginning = True
process_status = -1
gripper = baxter_interface.Gripper("left")
pub = rospy.Publisher('/user_feedback', String, queue_size=1)
rate = rospy.Rate(100)

# Checks wheter a given pose is valid
#   limb_arg takes the limb name, "left" or "right"
#   pose_arg takes the desired pose
#   returns false if no solution is found
#   returns a set of joint values for the limbs if a soloution is found
def ik_request(limb_arg,pose_arg):
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    ikreq = SolvePositionIKRequest()
    ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose_arg))
    ns = "ExternalTools/" + limb_arg + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    try:
        resp = iksvc(ikreq) # actual intent to find inverse kinematics solutions
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return False

    resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID): # Check if result is valid
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position)) # reformat to use in movement
        print("Valid IK Joint Solution found:\n{0}".format(limb_joints))
        return limb_joints
    else:
        rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
        return False

#  Sends a command to a Baxter arm to move
#   limb_arg takes the limb name, "left" or "right", default is "left"
#   move_type_arg takes the method of movement,
#       "move" takes the arm to specific coordinates
#       "displace" displaces the arm by the specified amount from the current position
#       default value is "move"
#   move_arg takes an array of 3 elements that will be used for movement
def cartesian_move(limb_arg="left", move_type_arg="move", move_arg=default_move):
    global process_status
    if process_status == -1:
        return -1
    if not limb_arg=="left" or not limb_arg=="right":
        limb_arg = "left"
    limb = baxter_interface.Limb(limb_arg)
    ik_pose = Pose() #create new pose from old one + displacements
    if not move_type_arg == "move" and not move_type_arg == "displace":
        move_type_arg = "move"
        move_arg = default_move
    if not len(move_arg) == 3:
        move_arg = default_move
    if move_type_arg == "move":
        ik_pose.position.x = move_arg[0]
        ik_pose.position.y = move_arg[1]
        ik_pose.position.z = move_arg[2]
    elif move_type_arg == "displace":
        current_pose = limb.endpoint_pose() # get current endpoint pose
        ik_pose.position.x = current_pose['position'].x + move_arg[0]
        ik_pose.position.y = current_pose['position'].y + move_arg[1]
        ik_pose.position.z = current_pose['position'].z + move_arg[2]
    # for orientation you can choose relative or absolute values
    ik_pose.orientation.x = -0.707   #np.pi/2    #current_pose['orientation'].x
    ik_pose.orientation.y = 0.707    #0.0        #current_pose['orientation'].y
    ik_pose.orientation.z = 0        #0.0        #current_pose['orientation'].z
    ik_pose.orientation.w = 0        #0.0        #current_pose['orientation'].w
    #print ik_pose.orientation
    joint_angles = ik_request(limb_arg,ik_pose) # call the ik solver for the new pose
    if joint_angles:
        limb.move_to_joint_positions(joint_angles) # move to new joint coordinates
def move_command(data):
    print "stuck"
    global process_status
    if process_status < 0:
        return -1
    "not stuck"
    string = data.data[1:-1]
    string_array = string.split(', ')
    if string_array == ["-1000", "-1000", "-1000"]:
        print "no shape detected, and thus no movement required"
        pass
    elif string_array == ["True", "True", "True"]:
        print "the gripper is now centered over the target"
    else:
        float_array = [float(string_array[0]), float(string_array[1]), float(string_array[2])]
        if float_array == [0.0, 0.0, 0.0]:
            print "the gripper is centered over the shape"
            # proceed to execute pick and place procedure
            pick_place()
        else:
            if process_status == 0:
                print "too close to determine approximate location"
                if float_array[0] == 0.0:
                    " attempt to center on x axis successful"
                    pick_place()
                else:
                    print " attempting to cetner on x axis"
                    cartesian_move("left", "displace", [float_array[0], 0.0, 0.0])
            else:
                print "shape detected, move Baxter arm towards shape"
                cartesian_move("left", "displace", float_array)
def pick_place():
    global process_status
    if process_status > 0:
        move_down()
        global process_status
        process_status -= 1
    elif process_status == 0:
        #move_down(0.16) #this was for the previous table in Ritterman building
        move_down(0.2) # this is for the tables in the Grove
        grab()
        move_up()
        move_home()
        #move_down(0.26) #this was for the previous table in Ritterman building
        move_down(0.29) #this is for the tables in the Grove
        release()
        move_up()
        move_home()
        global process_status
        process_status = -1
    print "process_status" + str(process_status)

def move_down(down_arg=0.05):
    z = -down_arg
    cartesian_move("left", "displace", [0, 0, z])
    print "moving down"
def move_up(up_arg=0.2):
    cartesian_move("left", "displace", [0, 0, 0.2])
def move_home():
    cartesian_move()
def grab():
    time.sleep(0.5)
    global gripper
    gripper.close()
    time.sleep(0.5)
def release():
    time.sleep(0.5)
    global gripper
    gripper.open()
    time.sleep(0.5)
def user_input(data):
    string = data.data
    global flag_beginning
    global process_status
    flag_beginning = True
    process_status = 3

def main():
    global process_status
    if process_status < 0:

    else:
        cartesian_move()
        global gripper
        gripper.calibrate()
        print "initalized"
        flag_beginning = False
    #print "before subscriber"
    rospy.Subscriber("/converge", String, move_command, queue_size=1)
    rospy.Subscriber("/user_input", String, user_input, queue_size=1)
    #print "after subscriber"
    rospy.spin()
    #print "after spin"
    #limb = 'left'
    #movement = [-0.2,0.0,0.0]#[0,0.303,-0.303]
    #moveCartesianSpace(limb,movement)
    #process_status = 3
    #flag_beginning = True

if __name__ == '__main__':
    global process_status
    process_status = -1
    sys.exit(main())

