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
 
    rate = rospy.Rate(2) # 10hz
  
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
            #reading values such as 900144.00
              #this would result in 144 for depth value and "killed" for kill status
              #the starting index will need to be editing if values are expected
              #to be greater than 999 (e.g. 903144.00 would currently still be read as 144
            data = str(ser.readline())
            
            while data[0] == "9":
                depthString = data[3: data.index(".")]
                killString = data[7]
               #killString = str(1.0)
                print("depth: " + str(depthString))
                print("killstring: " + str(killString))
                #update data
                data = ser.readline() 
                killStatus = "undertermined"
                rate.sleep()
                
                if killString[0] == "0": 
                    killStatus = "killed"
                    pub.publish(0.0)      
                
                elif killString[0] == "1": 
                    killStatus = "operating"
                    pub.publish(1.0) 
               
                print("kill status: " + killStatus) 
             

if __name__ == '__main__':
        display()
 
