#!/usr/bin/python3
# -*- coding: utf-8 -*-

# Imports
import cv2
import sys
import rospy
import numpy as np
from hsr_navigation.msg import Point, Points, Points2D, Labels
from hsr_navigation.srv import DetectionService, DetectionServiceResponse

# Import detecto library
from detecto import core, utils, visualize

# Drawing variables
THICKNESS = 2
COLOR_B = (255, 0, 0) 
COLOR_G = (0, 255, 0) 
COLOR_R = (0, 0, 255)

firstTime = True

def handle_detection_request(msg):
    global firstTime

    """
        Predicts affordances for given image.
    """
    try:
        # print("Detection: Convert image.")

        # Convert sensor msg Image into opencv image
        opencv_image = np.frombuffer(msg.image.data, dtype=np.uint8).reshape(msg.image.height, msg.image.width, -1)

        # print("Detection: Load model.")

        # Load the model
        model = core.Model.load('/home/hrl/catkin_ws/src/hsr_navigation/scripts/model/model2.pth', ['push', 'kick', 'grasp'])

        # print("Detection: Run prediction.")

        # Run prediction
        predictions = model.predict(opencv_image)

        # Retrieve data from prediction
        labels, boxes, scores = predictions

        print("Detection: Labels:", labels)
        print("Detection: Scores:", scores)
        print("\n")
        # print("Detection: Boxes:", boxes)
        
        # print("Detection: Populating service response.")

        # Service response
        points2d = Points2D()
        labelsOut = Labels()

        # Populate service response
        for x in range(len(labels)):

            # Skip processing low score
            # detections or object that need
            # to be pushed
            if scores[x] < 0.5 or labels[x] == "push":
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

            # Get np mgrid for label corners
            X, Y = np.mgrid[xmin:xmax, ymin:ymax]

            # Vertically stack mgrid
            stack_grid = np.vstack((X.ravel(), Y.ravel()))

            # Create points list
            points = Points()

            for y in range(stack_grid.shape[1]):
                # Create 2d point
                point = Point()
                point.x = stack_grid[0][y]
                point.y = stack_grid[1][y]
            
                # Append point to points msg
                points.data.append(point)
                                    
            # Append label
            labelsOut.data.append(labels[x])

            # Append points to points2d
            points2d.data.append(points)

        # # Displaying the image
        # if firstTime:
        #     firstTime = False
        # else:
        # cv2.imshow("Detections", opencv_image)
        # cv2.waitKey(0)

        # Return service response
        return DetectionServiceResponse(labelsOut, points2d)

    except Exception as e:
        print("Detection: Exception occured: ", e)

def main():
    rospy.init_node('detection_service_py')

    # Create service
    s = rospy.Service('detection_service', DetectionService, handle_detection_request)

    # Sping action server
    print("Detection: spinning.")
    rospy.spin()

# Execute
if __name__ == "__main__":
    main()