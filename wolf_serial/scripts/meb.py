#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
import serial
import re

serial_device = '/dev/ttyACM3'
serial_rate = 9600
kill_regex = 'Armed: (\d+)'

class MEB:
    def __init__(self):
        # init ROS
        rospy.init_node('meb', anonymous=False)
        rate = rospy.Rate(20)
        kill_pub = rospy.Publisher("hardware_killswitch", Bool, queue_size=10)

        #serial read loop
        with serial.Serial(serial_device, serial_rate, timeout=None) as ser:
            while not rospy.is_shutdown():
                line = ser.readline().decode('ascii')
                kill_match = re.search(kill_regex, line)
                if kill_match:
                    kill_pub.publish(int(kill_match.group(1)) == 1)  

                rate.sleep()


if __name__ == '__main__':
    meb = MEB()
