# #!/usr/bin/env python3

import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Instantiate CvBridge
bridge = CvBridge()

lowH, lowS, lowV = 43, 91, 0
highH, highS, highV = 88, 223, 76

lower_hsv = np.array([lowH, lowS, lowV])
higher_hsv = np.array([highH, highS, highV])

def image_callback(msg):
    print("Received an image!")

    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")

        hsv = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, lower_hsv, higher_hsv)

        cv2_img = cv2.bitwise_and(cv2_img, cv2_img, mask=mask)

        cv2.imshow('image', cv2_img)
        cv2.waitKey(3)
    except CvBridgeError as e:
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