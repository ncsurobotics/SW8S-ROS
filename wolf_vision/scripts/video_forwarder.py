#!/usr/bin/env python
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time
import sys

def forwarder(file_name):
    rospy.init_node('camera_forwarder', anonymous=False)
    rate = rospy.Rate(10)

    video = cv2.VideoCapture(file_name)
    bridge = CvBridge()

    frame_width = int(video.get(3))
    frame_height = int(video.get(4))
   
    size = (frame_width, frame_height)

    image_pub = rospy.Publisher('wolf_camera1/image_raw', Image, queue_size=10)
    writer = cv2.VideoWriter('test' + str(time.time()) + '.avi', 
                         cv2.VideoWriter_fourcc(*'MJPG'),
                         10, size)

    while not rospy.is_shutdown() and video.isOpened():
        ret, frame = video.read()
        if ret == False:
            break
        
        final = frame
        #final = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        writer.write(final)

        image_pub.publish(bridge.cv2_to_imgmsg(final, "bgr8"))
        rate.sleep()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python video_forwarder.py file_name")
        exit(1)
    print("Calling forwarder with {} file", sys.argv[1])
    forwarder(sys.argv[1])
    
