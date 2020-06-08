#!/usr/bin/python
# -*- coding: utf-8 -*-

# Imports
import sys
import rospy
import rospy
import hsrb_interface
from hsrb_interface import geometry
from hsr_navigation.srv import ActionService, ActionServiceResponse

# Grasp force[N]
GRASP_FORCE = 0.2

# TF name of the object to be grasped
OBJECT_TF = 'object_frame'

# TF name of the gripper
HAND_TF = 'hand_palm_link'

# Preparation for using the robot functions
robot = hsrb_interface.Robot()
gripper = robot.get('gripper')
omni_base = robot.get('omni_base')
whole_body = robot.get('whole_body')

def kick():
    """
        Kick action.
    """
    # Kick: phase 1 (approach)
    print("Kick: phase 1.")
    whole_body.move_end_effector_pose(geometry.pose(x=0.1, z=-0.3, ej=-1.57), OBJECT_TF)

    # Kick: phase 1 (kick)
    print("Kick: phase 2.")
    whole_body.move_end_effector_pose(geometry.pose(x=0.1), OBJECT_TF)

    # Log
    print("Kick: done.")

def grasp():
    """
        Grasping action.
    """
    # Grasp: phase 1 (approach)
    print("Grasp: phase 1.")
    whole_body.move_end_effector_pose(geometry.pose(x=0.1, z=-0.1), OBJECT_TF)

    # Grasp: phase 2 (grasp)
    print("Grasp: phase 2.")
    whole_body.move_end_effector_pose(geometry.pose(x=0.1, z=-0.02), OBJECT_TF)

    # Specify the force to grasp
    print("Grasp: closing gripper now.")
    gripper.apply_force(GRASP_FORCE)

    # Wait time for simulator's grasp hack. Not needed on actual robot
    print("Grasp: simulator hack.")
    rospy.sleep(2.0)

    # Transit to neutral position
    print("Grasp: neutral pose transition.")
    whole_body.move_to_neutral()

    # Rotate 180 degrees
    print("Grasp: 180 degree rotation.")
    omni_base.go_rel(0.0, 0.0, 3.14)

    # Release object
    print("Grasp: object drop.")
    gripper.command(1.2)
    rospy.sleep(2.0)

    # Rotate to initial pose
    print("Grasp: 180 degree rotation.")
    omni_base.go_rel(0.0, 0.0, 3.14)

    # Close gripper
    print("Grasp: closing gripper.")
    gripper.command(0.0)

    print("Grasp: done.")

def handle_action_request(service_msg):
    """
        Handles action request.
    """
    try:
        # Transit to initial grasping posture
        print("Action: transition to action configuration.")
        whole_body.move_to_neutral()

        # Command to open the gripper
        print("Action: opening gripper.")
        gripper.command(1.2)

        # Look at the hand after the transition
        print("Action: set hand constraint.")
        whole_body.looking_hand_constraint = True

        # Call action method
        if service_msg.req.action == "grasp":
            grasp()
        elif action == "kick":
            kick()

        # Switch to moving configuration
        print("Action: transition to moving configuration.")
        whole_body.move_to_go()

        print("Action: done.")

        # Return service response
        return ActionServiceResponse(True)

    except Exception as e:
        print("Exception occured: ", e)
        rospy.logerr('Failed to grasp')

        # Switch to moving configuration
        print("Action: transition to moving configuration.")
        whole_body.move_to_go()

        # Return service response
        return ActionServiceResponse(False)

def action_server():
    # Create node
    rospy.init_node('action_server')
    s = rospy.Service('action_service', ActionService, handle_action_request)

    # Sping action server
    print("Action: spinning.")
    rospy.spin()

# Execute
if __name__ == "__main__":
    action_server()