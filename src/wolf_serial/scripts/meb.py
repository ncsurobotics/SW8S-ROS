#!/usr/bin/env python
import rospy
import serial
import time
from std_msgs.msg import String   
from std_msgs.msg import Float64

ser = serial.Serial('/dev/ttyACM0')

def display():
    pub = rospy.Publisher('wolf_meb', Float64, queue_size=10)
    rospy.init_node('meb', anonymous=True)
 
    rate = rospy.Rate(1) # 10hz
  
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
            pub.publish(1.0)
            rate.sleep()
           # while ser.readline()[0] == "9":
            #  depthString = ser.readline()[2:6]
             # killString = ser.readline()[6]
              #print("depth: " + depthString)
              #if killString == "0":
               # killStatus = "operating"
                #pub.publish(1.0)
              #else: 
               # killStatus = "killed"
                #pub.publish(0.0)
              #print("kill status: " + killStatus) 

if __name__ == '__main__':
        display()
 
