#!/usr/bin/env python

"""
video2frames.py

Dan Hayduk
January 11, 2021

This file creates a ROS node that takes a video file name
as an input and outputs the video's individual frames in a
sequence.

The input is a String containing the full file name of the 
video and should be published to the topic 'video_name'

The output is of type Image from the sensor_msgs ROS message
library. Several images are published at a rate of 25 images
per second, but the publisher has a queue size of 10, so any
receiving node will need to have a process for quickly 
processing/storing images as they are published.
The images are each published to the topic 'frame'

The video file must be available in this node's workspace,
or else a simple error message will be logged to the console
stating that the file could not be opened. The video file can
be in any format that can be opened by OpenCV2. This node 
also needs to have access to the rospy, cv2, std_msgs and 
sensor_msgs libraries.
"""

import rospy
import cv2 as cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def video_handler(data):
    pub = rospy.Publisher('frame', Image, queue_size=10)
    rate = rospy.Rate(25)

    cap = cv.VideoCapture(data.data)

    if not cap.isOpened():
        rospy.loginfo("Couldn't read the file...")

    while cap.isOpened():
        ret, frame = cap.read()
        # if frame is read correctly, ret is True

        if not ret:
            break
        
        bridge = CvBridge()
        message = bridge.cv2_to_imgmsg(frame, encoding="passthrough")
        pub.publish(message)

        rate.sleep()

    cap.release()

def video2frames():
    rospy.init_node('video2frames', anonymous=True)
    rospy.Subscriber('video_name', String, video_handler)
    
    rospy.spin()

if __name__ == "__main__":
    try:
        video2frames()
    except rospy.ROSInterruptException:
        pass