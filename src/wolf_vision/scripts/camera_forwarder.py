#!/usr/bin/env python
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time

def forwarder():
    rospy.init_node('camera_forwarder', anonymous=False)
    rate = rospy.Rate(10)

    camera = cv2.VideoCapture(0)
    bridge = CvBridge()

    frame_width = int(camera.get(3))
    frame_height = int(camera.get(4))
   
    size = (frame_width, frame_height)

    image_pub = rospy.Publisher('wolf_camera1/image_raw', Image, queue_size=10)
    writer = cv2.VideoWriter('test' + str(time.time()) + '.avi', 
                         cv2.VideoWriter_fourcc(*'MJPG'),
                         10, size)

    while not rospy.is_shutdown():
        ret, frame = camera.read()
        
        final = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        writer.write(final)

        image_pub.publish(bridge.cv2_to_imgmsg(final, "bgr8"))
        rate.sleep()

if __name__ == '__main__':
    forwarder()
    
