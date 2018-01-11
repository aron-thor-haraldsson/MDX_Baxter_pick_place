#!/usr/bin/env python

import sys
import rospy
import baxter_interface
import tf
import struct
import numpy as np
     
from geometry_msgs.msg import (
	PoseStamped,
	Pose,
	Point,
	Quaternion,
	)
  
from std_msgs.msg import Header
     
from baxter_core_msgs.srv import (
	SolvePositionIK,
	SolvePositionIKRequest,
	)

def ik_request(limb,pose):
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    ikreq = SolvePositionIKRequest()
    ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
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
    

def moveCartesianSpace(limb_arg,displacement):
    rospy.init_node("rsdk_ik_service_client")
    limb = baxter_interface.Limb(limb_arg)
    current_pose = limb.endpoint_pose() # get current endpoint pose
    [dx,dy,dz] = displacement
    ik_pose = Pose() #create new pose from old one + displacements
    ik_pose.position.x = current_pose['position'].x + dx
    ik_pose.position.y = current_pose['position'].y + dy
    ik_pose.position.z = current_pose['position'].z + dz
    # for orientation you can choose relative or absolute values
    ik_pose.orientation.x = np.pi/2 #current_pose['orientation'].x
    ik_pose.orientation.y = 0.0 #current_pose['orientation'].y
    ik_pose.orientation.z = 0.0 #current_pose['orientation'].z
    ik_pose.orientation.w = 0.0 #current_pose['orientation'].w
    print ik_pose.orientation
    joint_angles = ik_request(limb_arg,ik_pose) # call the ik solver for the new pose
    if joint_angles:
        limb.move_to_joint_positions(joint_angles) # move to new joint coordinates

def main():
    limb = 'left'
    movement = [-0.2,0.0,0.0]#[0,0.303,-0.303]
    moveCartesianSpace(limb,movement)
 
if __name__ == '__main__':
   	 sys.exit(main())

