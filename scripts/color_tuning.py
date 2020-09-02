# #!/usr/bin/env python3

import cv2
import sys
import rospy
import numpy as np
from sensor_msgs.msg import Image

# Import detecto library
from detecto import core, utils, visualize

# Drawing variables
THICKNESS = 2
COLOR_B = (255, 0, 0) 
COLOR_G = (0, 255, 0) 
COLOR_R = (0, 0, 255)

# Load the model
model = core.Model.load('/home/hrl/catkin_ws/src/hsr_navigation/scripts/model/model2.pth', ['push', 'kick', 'grasp'])

from cv_bridge import CvBridge, CvBridgeError

# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
    try:
        # Convert sensor msg Image into opencv image
        opencv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        opencv_image = opencv_image[...,::-1].copy()

        # Convert your ROS Image message to OpenCV2
        # opencv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        print("Detection: Run prediction.")

        # Run prediction
        predictions = model.predict(opencv_image)

        # Retrieve data from prediction
        labels, boxes, scores = predictions

        print("Detection: Labels:", labels)
        print("Detection: Scores:", scores)
        print("Detection: Boxes:", boxes)
        
        print("Detection: Populating service response.")

        # Populate service response
        for x in range(len(labels)):

            # Skip processing low score
            # detections or object that need
            # to be pushed
            if scores[x] < 0.53:
                continue

            # Get label corners (box)
            xmin = int(boxes[x][0])
            xmax = int(boxes[x][2])
            ymin = int(boxes[x][1])
            ymax = int(boxes[x][3])

            # Draw rectangles
            if (labels[x] == "push"):
                cv2.rectangle(opencv_image, (xmin, ymin), (xmax, ymax), COLOR_B, THICKNESS)
            
            if (labels[x] == "grasp"):
                cv2.rectangle(opencv_image, (xmin, ymin), (xmax, ymax), COLOR_G, THICKNESS)
        
            if (labels[x] == "kick"):
                cv2.rectangle(opencv_image, (xmin, ymin), (xmax, ymax), COLOR_R, THICKNESS)
            
        # Displaying the image  
        cv2.imshow("Detections", opencv_image)
        cv2.waitKey(3)

    except Exception as e:
        print(e)

def main():
    rospy.init_node('image_listener')

    # Define your image topic
    image_topic = "/hsrb/head_rgbd_sensor/rgb/image_rect_color"

    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)

    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()