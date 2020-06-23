#!/usr/bin/env python
import rospy
import serial
import time
from std_msgs.msg import String   

ser = serial.Serial('/dev/ttyACM0')

def display():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('meb', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        print(ser.name)
        print(ser.readline())
        #WaitFor7E
        ser.write('X')
        print(ser.readline())
        #WaitForFF
        ser.write('A')
        print(ser.readline())
        #Read
        ser.write('C')
        print(ser.readline())
        #Address
        ser.write('T')
        print(ser.readline())
        
        while True:
            while ser.readline()[0] == "9":
                depthString = ser.readline()[2:6]
                killString = ser.readline()[6]
                print("depth: " + depthString)
                if killString == "0":
                    killStatus = "operating"
                else: killStatus = "killed"
                print("kill status: " + killStatus)           
        rate.sleep()

if __name__ == '__main__':
        display()
 
