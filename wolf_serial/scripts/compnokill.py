#!/usr/bin/env python                                                                                                                
import Jetson.GPIO as GPIO
import rospy
from std_msgs.msg import Bool

class Killswitch:
    pin_number = 13

    def __init__(self):
        # init ROS                                                                                                                   
        rospy.init_node('killswitch', anonymous=False)
        rate = rospy.Rate(20)

        #init GPIO                                                                                                                   
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin_number, GPIO.OUT, initial=GPIO.LOW)
        GPIO.output(self.pin_number, GPIO.HIGH)

if __name__ == '__main__':
    killswitch = Killswitch()
