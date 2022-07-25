#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, Float64
import serial
import re

serial_device = '/dev/ttyACM3'
serial_rate = 57600
kill_regex = 'Armed: (\d+)'
gyro_regex = 'GyroZ: (\d+)'

class MEB:
    def __init__(self):
        # init ROS
        rospy.init_node('meb', anonymous=False)
        rate = rospy.Rate(100)
        kill_pub = rospy.Publisher("hardware_killswitch", Bool, queue_size=10)
        gyro_pub = rospy.Publisher("external_gyro_z", Float64, queue_size=10)

        #serial read loop
        with serial.Serial(serial_device, serial_rate, timeout=None) as ser:
            while not rospy.is_shutdown():
                try:
                    line = ser.readline().decode('ascii')
                except:
                    line = ""
                kill_match = re.search(kill_regex, line)
                gyro_match = re.search(gyro_regex, line)
                if kill_match:
                    kill_pub.publish(int(kill_match.group(1)) == 1)  
                if gyro_match:
                    gyro_pub.publish((3.1415 * float(gyro_match.group(1))) / 180.0)  

                rate.sleep()


if __name__ == '__main__':
    meb = MEB()
