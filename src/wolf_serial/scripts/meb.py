#!/usr/bin/env python
import rospy
import serial
import time
from std_msgs.msg import String   
from std_msgs.msg import Float64

ser = serial.Serial('/dev/ttyACM1')

def display():
    pub = rospy.Publisher('wolf_meb', Float64, queue_size=10)
    rospy.init_node('meb', anonymous=True)
 
    #rate = rospy.Rate(2) # 10hz
  
    while not rospy.is_shutdown():
          if (ser.read() == '1'):
              print("killed")
              pub.publish(0.0)
          else:
              pub.publish(1.0)
if __name__ == '__main__':
        display()