#!/usr/bin/env python
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time

def forwarder():
    fps = 30
    width = 640
    height = 480

    rospy.init_node('camera_forwarder', anonymous=False)
    
    # Note: publishing at 2x FPS of camera prevents reading of old frames
    # via rviz and other tools (common strategy for realtime streams)
    rate = rospy.Rate(fps * 2)

    camera = cv2.VideoCapture(2)
    bridge = CvBridge()

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    camera.set(cv2.CAP_PROP_FPS, fps)

    # frame_width = int(camera.get(3))
    # frame_height = int(camera.get(4))
    # size = (frame_width, frame_height)
    size = (width, height)

    image_pub = rospy.Publisher('wolf_camera1/image_raw', Image, queue_size=1)
    writer = cv2.VideoWriter('test' + str(time.time()) + '.avi', 
                         cv2.VideoWriter_fourcc(*'MJPG'),
                         fps, size)

    while not rospy.is_shutdown():
        ret, frame = camera.read()
        
        final = cv2.rotate(frame, cv2.ROTATE_180)
        #final = frame
        writer.write(final)

        image_pub.publish(bridge.cv2_to_imgmsg(final, "bgr8"))
        rate.sleep()

if __name__ == '__main__':
    forwarder()
    
