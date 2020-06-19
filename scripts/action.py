#!/usr/bin/python
# -*- coding: utf-8 -*-

# Imports
import sys
import rospy
import hsrb_interface
from hsrb_interface import geometry
from hsr_navigation.srv import ActionService, ActionServiceResponse

# Kick/Grasp variables
GRASP_FORCE = 0.2
OBJECT_TF = 'object_frame'
HAND_TF = 'hand_palm_link'
OBJECT_TF_GRASP = 'object_frame_grasp'
OBJECT_TF_KICK = 'object_frame_kick'

# Push variables
DISTANCE = 0.700

# Common variables
robot = hsrb_interface.Robot()
gripper = robot.get('gripper')
omni_base = robot.get('omni_base')
whole_body = robot.get('whole_body')

def push():
    """
        Push action.
    """
    # Perform push (first rotation)
    print("Push: first rotation.")
    omni_base.go_rel(0.0, 0.0, 3.14)

    # Perform push (push)
    print("Push: push action.")
    omni_base.go_rel(-DISTANCE, 0.0, 0.0)

    # Perform push (backtracking)
    print("Push: backtracking.")
    omni_base.go_rel(DISTANCE, 0.0, 0.0)

    # Perform push (final rotation)
    print("Push: final rotation.")
    omni_base.go_rel(0.0, 0.0, 3.14)

    # Log
    print("Push: done.")

def kick():
    """
        Kick action.
    """
    # Initial kicking setup
    manipulation_setup()

    # Command to open the gripper
    print("Action: closing gripper.")
    gripper.command(0.1)

    # Kick: phase 1 (approach)
    print("Kick: phase 1.")
    whole_body.move_end_effector_pose(geometry.pose(x=0.15, z=-0.15, ej=-1.57), OBJECT_TF_KICK)

    # Kick: phase 1 (kick)
    print("Kick: phase 2.")
    whole_body.move_to_joint_positions({'wrist_flex_joint': 1.0})
    # whole_body.move_end_effector_pose(geometry.pose(x=0.2, z=0.2), OBJECT_TF_KICK)

    # Log
    print("Kick: done.")

def grasp():
    """
        Grasping action.
    """
    # Initial grasping setup
    manipulation_setup()

    # Command to open the gripper
    print("Grasp: opening gripper.")
    gripper.command(1.2)

    # Grasp: phase 1 (approach)
    print("Grasp: phase 1.")
    whole_body.move_end_effector_pose(geometry.pose(x=0.1, z=-0.1), OBJECT_TF_GRASP)

    # Grasp: phase 2 (grasp)
    print("Grasp: phase 2.")
    whole_body.move_end_effector_pose(geometry.pose(x=0.1, z=-0.02), OBJECT_TF_GRASP)

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

def manipulation_setup():
    # Transit to initial grasping posture
    print("Action: transition to action configuration.")
    whole_body.move_to_neutral()

    # Look at the hand after the transition
    print("Action: set hand constraint.")
    whole_body.looking_hand_constraint = True

def handle_action_request(service_msg):
    """
        Handles action request.
    """
    try:
        # Call action method
        if service_msg.action == "grasp":
            grasp()
        elif service_msg.action == "kick":
            kick()
        elif service_msg.action == "push":
            push()

        # Switch to moving configuration
        print("Action: transition to moving configuration.")
        whole_body.move_to_go()

        # Command to open the gripper
        print("Action: closing gripper.")
        gripper.command(0.0)

        print("Action: done.")

        # Return service response
        return ActionServiceResponse(True)

    except Exception as e:
        print("Exception occured: ", e)
        rospy.logerr('Failed to grasp')

        # Switch to moving configuration
        print("Action: transition to moving configuration.")
        whole_body.move_to_go()

        # Command to open the gripper
        print("Action: closing gripper.")
        gripper.command(0.2)

        # Return service response
        return ActionServiceResponse(False)

def main():
    # Create service
    s = rospy.Service('action_service', ActionService, handle_action_request)

    # Sping action server
    print("Action: spinning.")
    rospy.spin()

# Execute
if __name__ == "__main__":
    main()