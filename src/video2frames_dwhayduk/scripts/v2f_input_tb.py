#!/usr/bin/env python
import rospy
from std_msgs.msg import String

"""
v2f_input_tb.py

Dan Hayduk
January 11, 2021

This file creates a ROS node to test the main node's input
ability. It prompts the user to press Enter, then publishes
the string 'gate2.avi' to the topic 'video_name' indicating
the file name of the video for the main node to process.
"""

def publishVideoName():
    rospy.init_node('v2f_input_testbench', anonymous=True)
    pub = rospy.Publisher('video_name', String, queue_size=10)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        message = "gate2.avi"
        garbageString = raw_input("Press Enter to play the video.")
        pub.publish(message)
        rate.sleep()

if __name__ == "__main__":
    try:
        publishVideoName()
    except rospy.ROSInterruptException:
        pass