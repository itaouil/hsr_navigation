#!/usr/bin/python
# -*- coding: utf-8 -*-

import hsrb_interface
import rospy
import sys
from hsrb_interface import geometry

# Grasp force[N]
_GRASP_FORCE = 0.2

# TF name of the object to be grasped
_BOTTLE_TF = 'grasp_frame'

# TF name of the gripper
_HAND_TF = 'hand_palm_link'

# Preparation for using the robot functions
robot = hsrb_interface.Robot()
gripper = robot.get('gripper')
omni_base = robot.get('omni_base')
whole_body = robot.get('whole_body')

if __name__=='__main__':
    try:
        # Command to open the gripper
        gripper.command(1.2)
        print("Done0")

        # Transit to initial grasping posture
        whole_body.move_to_neutral()
        print("Done1")

        # Look at the hand after the transition
        whole_body.looking_hand_constraint = True
        print("Done2")

        # First approach grasping
        whole_body.move_end_effector_pose(geometry.pose(x=0.1, z=-0.1), _BOTTLE_TF)
        print("Done3a")

        # Second approach grasping
        whole_body.move_end_effector_pose(geometry.pose(x=0.1, z=-0.02), _BOTTLE_TF)
        print("Done3b")

        # Specify the force to grasp
        gripper.apply_force(_GRASP_FORCE)
        print("Done4")

        # Wait time for simulator's grasp hack. Not needed on actual robot
        rospy.sleep(2.0)
        print("Done5")

        # Transit to neutral position
        whole_body.move_to_neutral()
        print("Done8")

        # Rotate 180 degrees
        omni_base.go_rel(0.0, 0.0, 3.14)
        print("Done9")

        # Release object
        gripper.command(1.2)
        rospy.sleep(2.0)
        print("Done10")

        # Rotate to initial pose
        omni_base.go_rel(0.0, 0.0, 3.14)
        print("Done12")

        # Close gripper
        gripper.command(0.0)
        print("Done11")

        # Switch to moving configuration
        whole_body.move_to_go()
        print("Done12")

    except Exception as e:
        print("Exception occured: ", e)
        rospy.logerr('Failed to grasp')
        sys.exit()
