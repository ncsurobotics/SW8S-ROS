#!/usr/bin/env python

"""
v2f_output_tb.py

Dan Hayduk
January 11, 2021

This file creates a ROS node to test the main node's output
ability. It subscribes to the 'frame' topic and displays each
frame as it is received with the OpenCV2 method imshow.
"""

import rospy
import numpy as np
import cv2 as cv

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def frame_handler(data):
    # Convert ROS Image to OpenCV Image
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

    cv.imshow('Frames from Main Node', cv_image)
    cv.waitKey(10)

def v2f_testbench():
    rospy.init_node('v2f_testbench', anonymous=True)
    rospy.Subscriber('frame', Image, frame_handler)
    
    rospy.spin()

if __name__ == "__main__":
    v2f_testbench()