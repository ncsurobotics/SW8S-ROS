#!/usr/bin/env python                                                                                                                
import Jetson.GPIO as GPIO
import rospy
from std_msgs.msg import Bool

class Killswitch:
    pin_number = 13

    def switch_callback(self, data: Bool):
        if data.data == 0:
            GPIO.output(self.pin_number, GPIO.LOW)
        else:
            GPIO.output(self.pin_number, GPIO.HIGH)

    def __init__(self):
        # init ROS                                                                                                                   
        rospy.init_node('killswitch', anonymous=False)
        rate = rospy.Rate(20)

        #init GPIO                                                                                                                   
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin_number, GPIO.OUT, initial=GPIO.LOW)

        rospy.Subscriber("killswitch", Bool, self.switch_callback)

        rospy.spin()

if __name__ == '__main__':
    killswitch = Killswitch()
