# #!/usr/bin/python
# # -*- coding: utf-8 -*-

# # Imports
# import sys
# import rospy
# from hsr_navigation.srv import ActionService, ActionServiceResponse

# def handle_detection_request(service_msg):
#     """
#         Handles action request.
#     """
#     try:
#         # Specify the path to your image
#         image = utils.read_image('images/image0.jpg')
#         predictions = model.predict(image)

#         # predictions format: (labels, boxes, scores)
#         labels, boxes, scores = predictions

#         # ['alien', 'bat', 'bat']
#         print(labels) 

#         #           xmin       ymin       xmax       ymax
#         # tensor([[ 569.2125,  203.6702, 1003.4383,  658.1044],
#         #         [ 276.2478,  144.0074,  579.6044,  508.7444],
#         #         [ 277.2929,  162.6719,  627.9399,  511.9841]])
#         print(boxes)

#         # tensor([0.9952, 0.9837, 0.5153])
#         print(scores)

#         # Call action method
#         if service_msg.action == "grasp":
#             grasp()
#         elif service_msg.action == "kick":
#             kick()
#         elif service_msg.action == "push":
#             push()

#         # Switch to moving configuration
#         print("Action: transition to moving configuration.")
#         whole_body.move_to_go()

#         # Command to open the gripper
#         print("Action: closing gripper.")
#         gripper.command(0.0)

#         print("Action: done.")

#         # Return service response
#         return ActionServiceResponse(True)

#     except Exception as e:
#         print("Exception occured: ", e)
#         rospy.logerr('Failed to grasp')

#         # Switch to moving configuration
#         print("Action: transition to moving configuration.")
#         whole_body.move_to_go()

#         # Command to open the gripper
#         print("Action: closing gripper.")
#         gripper.command(0.2)

#         # Return service response
#         return ActionServiceResponse(False)

# def main():
#     # Create service
#     s = rospy.Service('detection_service', ActionService, handle_detection_request)

#     # Sping action server
#     print("Detection: spinning.")
#     rospy.spin()

# # Execute
# if __name__ == "__main__":
#     main()

from detecto import core, utils, visualize

model = core.Model.load('/home/hrl/Downloads/hsr_model.pth', ['push', 'kick', 'grasp'])

visualize.detect_video(model, '/home/hrl/Downloads/test.mp4', '/home/hrl/Downloads/output.avi')
